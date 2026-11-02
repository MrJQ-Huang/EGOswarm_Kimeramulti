# 实验部署与启动说明

本仓库保存现有 AMOV 与 UROS 双机实验的源码和配置。默认路径、IP、容器名及部分 underlay 来自这两台设备，需要先调整；尚未验证从全新系统一键构建和复现实验。

## 系统与依赖

当前实验采用 Ubuntu 20.04、ROS Noetic、GCC 9、OpenCV 4.2、PCL 1.10 和匹配配置的 GTSAM 4.2。AMOV 在 ARM64 本机运行，UROS 的原生 Kimera 与 EGO 环境在 x86-64 Docker 中运行，挂载实验根目录到 `/experiment`。

EGO 需要 ROS1 的运行版本；不要误用 UROS 宿主机另一个 ROS2 工作区。当前运行 EGO 源提交为 `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20`。上游依赖与本地补丁见 `dependencies/`，稠密 overlay 的历史构建记录见 `experiment/native_full_stack_20261102/build_recipe.json` 和 `source_lock.json`。

`build_recipe.json` 中的归档文件名描述既有构建过程。源码归档、编译产物与预构建 Docker 镜像不随仓库上传，不能直接假设这些文件已存在。先按固定提交与补丁准备 underlay，再编译 dense overlay；两种架构独立编译。

## 获取固定版本源码

在仓库根目录预览清单，再获取当前主机对应的源码与补丁：

```bash
python3 tools/fetch_sources.py --host uros --list
python3 tools/fetch_sources.py --host uros --destination /tmp/swarm_sources
```

AMOV 使用 `--host amov`。脚本按固定 SHA 获取 38 个仓库并应用本主机补丁，拒绝覆盖已有源码目录；只准备源码，不编译、不启动服务。可以用 `--component <id>` 单独获取一个组件。构建前还需处理清单中的 `post_fetch_notes`、系统依赖和 underlay/overlay ABI。

实际源码完整构建尚未在全新系统复测；`build_recipe.json` 保留已部署环境的构建说明。UROS VIO 保留两个稀疏特征防护分支，AMOV VIO 源保持其固定提交，双方补丁分别记录。

## 配置现有环境入口

检查并适配下列文件中所有绝对路径与网络地址：

- `experiment/native_full_stack_20261102/manage_native_stack.py`
- `experiment/native_full_stack_20261102/env_amov_full.bash`
- `experiment/native_full_stack_20261102/env_uros_full.bash`
- `experiment/harness/env_amov.bash` 与 `env_uros.bash`
- `experiment/harness/multimodal/backend.launch` 及其参数文件
- `experiment/configs/{amov,uros}/multimodal/` 中各机的 `vio.launch`、`robot_names.yaml` 与 `params/`

现有部署路径为 AMOV `/home/amov/Drone/two_pc_ego_20261030`，UROS `/home/uros/Drone/egoswarm/experiments/two_pc_ego_20261030`；仓库的 `experiment/` 对应该实验根目录。管理脚本默认 UROS 地址 `192.168.5.21`、AMOV 地址 `192.168.5.6`，共享 master 端口为 `11431`。脚本还对白名单地址与端口做了校验，换网段时应同步修改。

UROS 默认容器名为 `kimera-native-full-uros-20261102` 与 `ego-native-full-uros-20261102`。原生环境入口依赖各机既有 Kimera underlay；仓库保留入口不代表该环境已在新机器安装。

将 `experiment/` 的内容同步到各机实验根目录后，再将 `experiment/configs/<host>/multimodal/` 的内容覆盖同步到该机 `<实验根目录>/multimodal/`。管理器实际读取后者；两机 VIO 参数不同，不能都直接使用仓库 `experiment/multimodal/` 中的默认 UROS 配置。同步源码与配置时保留主机原有的 `runs/`、工作区和环境文件，不使用无边界的删除同步。

## 构建与接口检查

在启动仿真前完成以下检查：

1. `rospack` 实际解析到预期 underlay 和 overlay，`ldd` 无缺失库。
2. 两机交换的 `PoseGraph` 等消息 MD5 相同，OpenCV、Eigen 与 GTSAM ABI 一致。
3. 存在原生 `tsdf_server`、`voxblox_mesh_frontend_node` 与 `kimera_pgmo_node`；不能用 generic mesh frontend 代替 Voxblox frontend。
4. 固定版本 PGMO 使用现代 `MESH` 配置，原始关键帧 ID 连续且未被重编号。
5. 两机时钟同步、ROS_IP 可达，ROS 动态端口可双向连接；共享 master 可达不代表所有 topic 都可达。

仓库根目录的纯 Python 契约与插值测试：

```bash
python3 -m unittest discover -s experiment/harness/multimodal -p 'test_*.py'
```

这组测试覆盖接口与数值约定，原生依赖还需单独构建、运行上游检查并通过实际 ROS 输入验收。

当前运行启用了单独验证的 OpenGV 异常防护补丁。必须在本架构重新编译并建立校验文件后才传 `--opengv-guard`；不能从另一台主机复制 `.so`。未通过质量评估的 depth nearest 候选不属于当前运行基线。

## 启动顺序

以下是现有管理器的操作流程，需先完成路径与环境配置。两机使用相同的新 `run-id` 与同一个 UTC epoch `cutoff`，每机只运行一套实验。管理器保存进程归属，不能并发修改同一主机的管理状态文件。

1. 停止上一轮所属节点，确认 recorder 已落盘；保留原始数据和诊断证据。新一轮使用空输出目录及新进程状态，不需要清空 Linux 内核页缓存。
2. UROS 启动 `master`，配置 `/use_sim_time=false` 与当前运行标识；确认没有上一轮的 latched 数据源。
3. 两机依次启动 `record`、`adapter`、`heartbeat`、`backend`、`dense`、`vio`、`supervisor`；UROS 另启 `world`、`trajectory` 和 `global-view`。
4. `dense` 使用 `--mode joint --require-cross --cloud-rate 1 --cutoff ...`；VIO 与 backend 在防护库就绪时使用 `--opengv-guard`。
5. 用 `/two_pc/scenario_control` 发出本轮 `reset`：场景 `broad_loop`，`continuous_body=true`，同一 `run_id`。等待两机连续运动仿真体、规划器和各原生 SLAM 节点注册。
6. 两机启动 `sensors`，完成初始化与预热，检查原生 VIO key 0、IMU 和局部 TSDF 的真实积分，再由 UROS 启动 `mission --laps 2`。
7. 任务期间检查消息年龄、实际积分、队列与版本回执。结束时先停止新增传感器、前端及协同后端输入，限时排空已接收工作；再保存原生状态并对冻结结果做两次哈希核对，最后正常结束 recorder，使录制文件完整落盘。

例如，在已配置的 UROS 主机上检查指定轮次：

```bash
NATIVE_STAGE=/home/uros/Drone/egoswarm/experiments/two_pc_ego_20261030/native_full_stack_20261102
python3 "$NATIVE_STAGE/manage_native_stack.py" status --host uros --run-id YOUR_RUN_ID
```

管理器的 `stop` 只针对指定轮次所属进程，可用 `--roles` 限定；共享 master 默认保留，只有显式 `--stop-master` 才停止。不要用全局 `pkill` 代替归属检查。

## RViz 与数据

分别打开两机局部点云、全局点云和轨迹视图。局部 fixed frame 为 `sim_0/vio_odom` 与 `sim_1/vio_odom`；全局为 `kimera_world`。本机局部显示使用 `/sim_N/native_dense/raw_local_vertices`，全局只使用 `/visualization/native_mesh/global_cloud`，颜色青色与橙色区分来源。

数据保存到原部署根目录的 `native_full_stack_20261102/runs/<run-id>/<host>/`。至少保留 RGB-D、IMU、LiDAR、标定、原始关键帧图、协同约束与权重、PGMO 实际输入、原生 mesh、状态回执和任务时间记录。录制原始传感器需要预留磁盘空间，日志与 raw mesh 频率不能替代实际建图验收。

当前持续建图和同轮全局发布仍有阻塞与版本问题，见[已知问题](known_issues.md)。
