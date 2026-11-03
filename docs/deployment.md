# 实验部署与启动说明

本仓库保存现有 AMOV 与 UROS 双机实验的源码和配置。默认路径、IP、容器名及部分 underlay 来自这两台设备，需要先调整；尚未验证从全新系统一键构建和复现实验。

当前采用本文末尾的“三台主机集中计算入口”：设备只运行仿真、EGO 控制、日志和无损上行，本机运行全部Kimera/Voxblox与按profile录制。前面的 `manage_native_stack.py` 流程保留为历史环境说明，不要混用两套入口启动同一轮。

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

## 三台主机集中计算入口

`experiment/three_host_20261102/` 提供分工管理入口。AMOV（robot 0）和 UROS（robot 1）各自只运行 EGO、连续仿真体、传感器、心跳和无损上行。设备保留日志和进程管理元数据，不启动 rosbag、VIO CSV、TSDF 或地图导出。本机 `192.168.5.14` 运行接收分发、两套 VIO/adapter/TSDF/mesh frontend、LCD/DPGO/PGMO、唯一任务目标调度、全部显示、录制和后处理。

本机默认使用 `kimera-central-hub-gpu-20261103` ROS1 容器，挂载 `/home/mrjq311/swarm_kimera_runtime/experiment` 到 `/experiment`。该容器复用固定的原生 underlay，以 NVIDIA CDI 接入显卡、X11 和驱动；已准备环境的实际 OpenGL 检查见 `dependencies/locks/hub_gpu_runtime.json`。CDI 安装参考 [NVIDIA CDI 文档](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/cdi-support.html)。保留清单引用的 `nvidia-cdi-hook`，主机驱动升级后需重新生成 CDI，不复用旧驱动路径。`HUB_RENDER_MODE=nvidia` 为默认，显式 `software` 可用于排查。显卡负责 OpenGL，现有原生 SLAM 算法仍为 CPU 实现。

同步实验源码时保留既有 underlay/overlay 和构建产物；两机 `<root>/multimodal/` 仍按 `configs/<host>/multimodal/` 覆盖，不能用 UROS 默认配置覆盖 AMOV。当前集中 VIO 读取本机 `configs/amov|uros`，运行副本通过容器内 `rospack` 解析原生 VIO 启动路径，不使用 AMOV 的绝对路径。每份启动文件显式传入 `/experiment/configs/<host>/multimodal/flags`，保留各设备旧实验实际使用的四份 flags；部署时必须包含该目录，不能以其他数据集默认值代替。启动前在选定容器内检查四文件并计算 SHA256，写入该 VIO 的 `native_inputs.json`；同轮重启会核对这些依赖及独立复制的七份 YAML，发生变化时要求新 run ID。此前未生成 `native_inputs.json` 的运行目录可初建该文件。

本机观察 EGO 真实轨迹需要匹配的 `quadrotor_msgs/PositionCommand` 和 `traj_utils/Bspline`。仓库提供只生成消息的工作区，构建时不生成本机 EGO 规划器或控制库：

```bash
docker exec kimera-central-hub-gpu-20261103 \
  bash /experiment/three_host_20261102/env_hub.bash \
  catkin_make -C /experiment/three_host_20261102/control_messages \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF
```

`env_hub.bash` 自动加载该工作区。消息来源、原始字节哈希及运行 MD5 见 `control_messages/source_provenance.json`。`PositionCommand` 应为 `4712f0609ca29a79af79a35ca3e3967a`，`Bspline` 应为 `4f7510f7be6fba2868a5e74dd6e8163b`。

`manager_hub.py` 管理本机角色，`manager_local.py --host amov|uros` 管理对应设备。所有管理器使用同一新 `--run-id`、同一 `--cutoff` 及 `http://192.168.5.14:11431`。使用 `--dry-run` 可查看实际命令，且不创建运行目录或启动进程。

新一轮实际启动顺序如下；静态检查通过不等于已运行实验：

1. 本机启动 `master`，设置 `/use_sim_time=false`、`/rosout_disable_topics_generation=true` 和当前运行标识。
2. 两机启动 `uplink/heartbeat/supervisor`，不启动 sensors；本机启动 `ingress0/1`、`record`、`tsdf-witness`、`adapter0/1`、`local-frontend0/1`、`backend0/1`、`dense0/1`、`global-view/trajectory/world`。默认 `record --record-profile map` 保存结果和少量遥测；需原始输入重放时显式选 `replay`，全部中间诊断选 `debug`。录制预算覆盖启动与完整任务，例如 `--duration 1800`。
3. 一次 reset 创建两台连续仿真体，然后本机启动 `vio0/1`。不可在保留 SLAM 状态时重新 reset 仿真体。
4. `validation/validate_bootstrap_subscriptions.py` 要求设备私有传感器发布者尚未启动、实际body遥测已直接接入uplink、本机ingress已预发布标准话题、所有真实 TCPROS 连接和来源正确，且原生 key 0 的消费者已连接；通过后才启动两机 `sensors`。
5. 验证静止预热、七路小状态、原生 key 0/gauge、真实 TSDF 积分、EGO 唯一控制链及拓扑，再打开本机四个 RViz，由本机启动唯一 `mission --laps 2`。目标序列可分别配置，任务只发布目标 `PoseStamped`，实际路径、避障与控制由设备上的 EGO Planner 和 traj_server 完成。
6. 传感器启动后另行保存 `/sim_0/sensors/calibration` 与 `/sim_1/sensors/calibration` 参数到本机；这些是 ROS 参数，不会被 rosbag 自动当作话题录制。CameraInfo和原始输入在replay/debug模式才录包；map模式在线消费它们并只保留结果。
7. 收尾时先停目标调度与设备 sensors/uplink，排空已接收工作，再按角色保存本机原生状态并正常关闭本机recorder，运行最终地图导出。设备不需要传回任何原始录制包。

实际body状态直接从edge连续仿真体以默认30 Hz单份上行，hub任务/显示/录包通过`--central-telemetry`统一消费本地入口，禁止直接再订阅跨LAN `/drone_N_visual_slam/odom`。RGB-D使用原始ROS字节无损压缩，IMU独立100 Hz；每台设备每种上行通道只有本机 ingress 一个远端订阅者。禁止 VIO、录包或调试器直接订阅 `/edge/sim_N/...` 原始话题。标准 `/sim_N/...` 话题由本机分发。图像源龄上限默认 2 秒，配对缓存默认 8 帧；丢弃与序号间隙可从 uplink/ingress 状态读取，不修改传感器时间戳掩盖延迟。

本机可分别设置 `--rviz-fps 30`、`--trajectory-rate 20`、`--cloud-rate 5`、`--mesh-period 1`。高频机体显示不会反复重建整条历史路径；路径和状态输出另有限频。实际刷新、吞吐和地图更新能力仍需在后续运行中测量。

`hub_backend.launch` 的 `--external-sensor-stream` 使 PGMO 后端不再订阅原始 RGB-D/IMU，也不发布第二份 TSDF 输入；独立回投进程和 TSDF 同在本机。原有观测轮次协议及全局版本校验保留，局部图可用不代表全局同轮融合已通过。两份本机 VIO 的 `visualize/use_rviz` 参数用于 ROS 输出路径，保持开启；它们不负责弹出远端查看器。


### 输入优先级与最终地图保存

中心入口默认相机5 Hz、LiDAR1 Hz/每4点取1点、实际body遥测30 Hz。`manager_local.py`支持 `--camera-rate`、`--lidar-rate`、`--lidar-stride`、`--telemetry-rate` 显式覆盖，两端物理模型不随上传稀疏化改变。EGO本地5米/10 Hz range cloud从各自scene直接生成，与稀疏上传LiDAR独立。新协议v2需三端同时部署、使用新run/cutoff冷启动，不能在旧v1运行中只重启一端。

默认map录包不具备RGB-D离线重跑能力；原始输入需显式选择replay。停止重复录制中间点云/网格/完整显示Path仅减少本机磁盘和序列化，真正的LAN优化是单份遥测与稀疏辅助LiDAR。

保存最终地图（在本机运行，ROS-free导出；先正常闭合recorder，并按需要使用其结束时间作为cutoff）：

```bash
python3 experiment/harness/multimodal/export_final_global_map.py \
  --run-directory /path/to/runs/RUN/mrjq311 \
  --output /path/to/final_map \
  --cutoff-wall RECORD_END_EPOCH
```

导出保留native pair原始坐标、三角面、颜色和证据。青/橙来源色的PLY/PCD是可视化派生输出；原生三角mesh另存。没有有效matched pair时应明确失败，不能拿场景先验或独立对齐的局部云冒充联合地图。

本轮没有修改0.6秒遥测保护或EGO原生250毫秒peer轨迹时差门限。新数据通道的持续飞行验收必须结合源龄、实际回调间隔、状态与原生积分完成记录；静止预热、socket标记或单元测试通过不能代替运动验收。


2026年11月3日已完成一次本机集中版本的完整两圈运动验收，20段原生EGO证明齐全，末圈联合地图、全程真实TSDF积分和运动期间资源均已独立核验。见[验收摘要](evidence/motion_acceptance_20261103.json)。这不扩大为真机或任意网络条件保证，原生观测匹配协议的共享原子轮次限制保留。
