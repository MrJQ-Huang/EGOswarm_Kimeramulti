# 环境部署与入口选择

当前工作入口是 [distributed/manager.py](../experiment/three_host_20261102/distributed/manager.py)，采用两端本地原生 SLAM、三个独立 master 和本机结果 observer。分阶段操作见[分布式部署与启动](distributed_deployment.md)。本页保留共同依赖与集中 baseline 的说明，不应混用两种 profile 的节点、master、run 或停止参数。

第五轮 `distributed_continuity_20261104_153318` 已完成两圈 20/20 航段、continuity 修复核验与三主机正常关闭，RViz 已关闭。两端沿用 0.20 m TSDF 与原 FPS/CPU 参数；功能改造完成，AMOV accepted mesh 最大更新间隔仍为 22.8619 秒，强实时性能未通过。第四轮数据另保留为修复前证据。

本仓库主要复用现有 AMOV/UROS 环境。路径、IP、容器与部分 underlay 有设备约定；尚未证明从全新系统一键构建与复现实验。

## 系统、源码与 ABI

部署使用 Ubuntu 20.04、ROS Noetic、GCC 9、OpenCV 4.2、PCL 1.10 及匹配 Eigen/预积分配置的 GTSAM 4.2。AMOV 为 ARM64 本机环境；UROS 为 x86-64，原生 Kimera 与 EGO 使用独立 ROS1 容器。不要误用 UROS 宿主机的 ROS2 工作区，也不能跨 CPU 架构复制二进制。

| 主机 | 实验根目录 | 运行环境 |
| --- | --- | --- |
| AMOV | `/home/amov/Drone/two_pc_ego_20261030` | 原生 Kimera underlay 与 `native_full_stack_20261102/amov_ws` overlay；本机 ROS1 EGO |
| UROS | `/home/uros/Drone/egoswarm/experiments/two_pc_ego_20261030` | `kimera-native-full-uros-20261102` 与 `ego-native-full-uros-20261102`，实验目录挂载为 `/experiment` |
| 本机 | `/home/mrjq311/swarm_kimera_runtime/experiment` | `kimera-central-hub-gpu-20261103`，目录挂载为 `/experiment` |

仓库 `experiment/` 对应各机实验根目录中的内容。保留既有 underlay/overlay、构建产物、历史 `runs/` 和证据；同步源码时不要使用无边界删除。当前 distributed manager 直接从 `configs/amov|uros/multimodal/` 复制该机参数及 flags，并记录输入 hash；不能用 UROS 默认参数覆盖 AMOV。历史管理器读取 `<实验根>/multimodal/` 的约定另见下文。

原生依赖来源、固定提交与兼容补丁位于 `dependencies/`，构建记录位于 `experiment/native_full_stack_20261102/build_recipe.json` 与 `source_lock.json`。当前 PGMO 是现代 MESH API 兼容移植，不是 Kimera-Multi v1.0.0 官方依赖组合。构建记录中的源码归档、镜像与二进制不随仓库上传，不能假设目标设备已有这些文件。

```bash
python3 tools/fetch_sources.py --host uros --list
python3 tools/fetch_sources.py --host uros --destination /tmp/swarm_sources
```

AMOV 选择 `--host amov`。脚本只按锁定源码和本机补丁准备源树，不编译、不启动；已有目标目录不会被直接覆盖。构建前核对清单的系统依赖、`post_fetch_notes` 和 underlay/overlay ABI。UROS 的稀疏特征防护与 AMOV 的 VIO 基线差异已分别记录，不能擅自把两机补丁合并成同一套。

## 启动前的共同检查

1. 检查 `rospack` 实际路径、原生可执行文件及 `ldd`；必须有 `tsdf_server`、`voxblox_mesh_frontend_node`、`kimera_pgmo_node`，不能以 generic mesh frontend 代替 Voxblox 接口。
2. 对全部交换类型核验 type/MD5，包括 PoseGraph、BoW/VLC、DPGO 自定义矩阵和 PublicPoses、KimeraPgmoMesh、Bspline；不能只比较外层字节载体类型。
3. 核验相机/机体坐标约定、每机标定、关键帧 ID 与时间戳，没有使用 actual 填充 VIO 或重建输入。
4. 核验三机时钟偏差与各机 ROS_IP；同一个 master 端口号不表示共用 master。检查对应 profile 的固定 TCP 端口，而不是只测试 master 可达。
5. 先关闭上一轮所属角色、核对 OS 进程归属与 latched 数据源，再生成新 run/cutoff。不要在保留 SLAM 状态时 reset 仿真体。

`master` 启动返回前须通过本轮进程归属、master XML-RPC `getPid` 及 `/run_id` UUID 就绪检查，才能并发启动 roslaunch。第四轮第一次冷启动曾因只确认 Popen 返回而未等待 master bound，触发多个 roslaunch 自启 master 和 UUID 冲突；该次在传感器与运动前拦截。加入屏障后，新 run 的三机冷启动与双端预热均通过，不能通过忽略 UUID 错误继续使用旧 run。

OpenGV 防护库需在本架构独立构建、测试并建立 `runtime_gate.sha256` 后才使用 `--opengv-guard`。不能复制另一架构的 `.so` 或校验文件。未通过质量评估的 nearest-depth 候选不属于运行基线。

ROS1 EGO 消息在需要的 native/observer 环境内加载。仓库提供仅生成消息的工作区，来源见 `control_messages/source_provenance.json`；它不构建 hub 上的 EGO 控制程序：

```bash
docker exec kimera-central-hub-gpu-20261103 \
  bash /experiment/three_host_20261102/env_hub.bash \
  catkin_make -C /experiment/three_host_20261102/control_messages \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF
```

核验 `PositionCommand` MD5 为 `4712f0609ca29a79af79a35ca3e3967a`、`Bspline` 为 `4f7510f7be6fba2868a5e74dd6e8163b`。当前环境入口按存在性加载消息 workspace；构建结果仍须按各环境实际解析路径检查。

本机容器以 NVIDIA CDI 接入 OpenGL、X11 和驱动；实际配置见 `dependencies/locks/hub_gpu_runtime.json`。保留其引用的 hook，驱动升级后重新生成 CDI，参考 [NVIDIA 文档](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/cdi-support.html)。GPU 用于显示，现有原生 SLAM 没有因此变为 CUDA 实现。

## 当前分布式入口

所有角色使用 `distributed/manager.py --host amov|uros|hub`，各自主机执行。它有纯 `--dry-run`，实际启动后按 run、role、本地 master 和 distributed profile 标记核验进程归属。`observer` 使用 `--channels map`，`observer-telemetry` 使用 `--channels telemetry`，是两个独立 ROS 进程。

- AMOV/UROS：`master supervisor reset adapter local-frontend backend dense peer results vio sensors mission witness resource`。
- 本机：`master observer observer-telemetry quality rviz rviz-local0 rviz-local1 rviz-actual resource gpu`。
- 状态/关闭：`status`、`stop`，`--roles` 仅用于 `stop`。

每个命令均要求 `--run-id` 与 `--cutoff`；共同 gauge 默认是 `<run>.robot0-initial`，也可显式指定。`dense` 使用 online/no-persist，设备不存在 raw recorder 角色。**distributed `stop` 不指定 `--roles` 会关闭该 run 全部所属角色，包括 master；没有旧管理器的 `--stop-master` 选项。** 分阶段启动、独立 observer 停止测试及安全收尾详见[专门操作说明](distributed_deployment.md)。

第五轮新增 latched `/sim_N/native_dense/verified_body_path`，原 `approved_body_path` 的 topic/type/caller 不变。新 observer/导出器先按 receipt hash 选择 verified path，也兼容只有旧 `body_path` 的档案；兼容读取不允许放松 run/boot/gauge/source/hash 核验。部署应同步 native、result server、observer 与 exporter，不能只看到新 topic 名就认定最新 pending path 已被使用。

第五轮 174 个托管运行源码/配置文件在本地、AMOV、UROS、本机运行目录逐项 SHA256 一致；该范围不包含不同架构的二进制、underlay 或历史运行资料。最终 Noetic 294 项测试无 skip 通过。GitHub 当前仍只有 baseline snapshot，分布式 feature 正在本地准备提交，不代表已经推送发布。

## 保留的集中 baseline

`manager_hub.py` / `manager_local.py` 是 2026-11-03 集中计算 baseline：两端只控制、仿真和上行，本机在共享 `http://192.168.5.14:11431` 上运行两份 SLAM、任务、显示及录制。它不证明两机分担 SLAM 计算，与 distributed 的三个 master 严格分开。

集中 baseline 的初始化顺序为：

1. 本机 master；两端 uplink/heartbeat/supervisor，传感器先不启动。
2. 本机 ingress、adapter、local-frontend、backend、dense、witness、显示与按 profile 选择的 recorder。
3. reset 创建连续仿真体，本机 VIO 启动；冷启动订阅/真实连接/原生 key 0 消费者检查通过后，再启两端 sensors。
4. 静止预热与真实积分、gauge、EGO 唯一控制链检查通过后，由本机唯一 mission 发目标。
5. 停目标与传感器/上行、排空工作，保存本机原生结果并正常关闭 recorder，再导出地图。

该入口 VIO 在本机读取 `configs/amov|uros/multimodal/`，记录 flags/YAML/launch 输入 hash。设备 `<root>/multimodal/` 的历史副本只用于相应旧入口；保留旧原生 `manage_native_stack.py` 时需按主机配置维护该副本，不能把它当作 distributed 的实际读取路径。

集中录制 profile：`map` 保存结果和紧凑遥测，`replay` 增加 RGB-D/IMU，`debug` 保存全部中间诊断。map 模式不能重放完整原始输入；默认停止重复录包不代表停止在线传输。两端 private sensor 只有 uplink 消费，本机 ingress 分发标准话题，VIO/录包/显示不得直接重复跨 LAN 订阅设备 raw 流。此要求仅适用于集中 baseline，当前 distributed 的 raw 消费者是边端本地原生节点。

集中 baseline 的最终导出保留原生 pair、面、颜色、receipt 与来源；青/橙点云只是机器人来源色的派生显示：

```bash
python3 experiment/harness/multimodal/export_final_global_map.py \
  --run-directory /path/to/central/runs/RUN/mrjq311 \
  --output /path/to/final_map --cutoff-wall RECORD_END_EPOCH
```

该导出器针对旧 native-pair 事务目录，不直接用于 distributed observer 结果树。没有有效 pair 时不能用场景先验或分别拟合的局部地图补出联合结果。2026-11-03 集中版本一次完整运动结果见[验收摘要](evidence/motion_acceptance_20261103.json)。

`native_full_stack_20261102/manage_native_stack.py` 另保留为更早的两端原生部署入口，有其共享 master、录包、停止与固定 mesh 诊断语义；不属于当前分布式启动流程。2026-11-02 的历史积压/空全局显示见[已知问题](known_issues.md)，不要把旧状态当作当前 source 的结论。

## 验证

```bash
python3 -m unittest discover -s experiment/three_host_20261102/tests -p 'test_*.py'
python3 -m unittest discover -s experiment/harness/multimodal -p 'test_*.py'
```

软件契约测试、原生构建、ROS 冷启动、完整运动、观察端退出/新缓存重建与几何质量分别留证。第五轮完整运动窗口为 496.58 秒，实际路径 AMOV/UROS 为 159.8820/159.1399 m；三主机 closure 均为 true、`remaining=[]`。一次团队 SE(3) 的 body path ATE 为 0.1991626/0.2081018 m，配对支持率为 99.9051%/99.9268%。连续性档案审计全部检查通过；运动后以全新 RAM 缓存重启 map observer，复核 shared map 与两端 verified path 关联成功，遥测进程保持运行，未连续测恢复时延。这不等于飞行中全部结果接收退出或 peer 网络分区测试。

AMOV accepted mesh 最大更新间隔仍有 22.8619 秒（首次共享接纳后的 P95 13.5174 秒），整机平均 CPU 90.75%；UROS 相应最大值为 4.9907 秒，TSDF 运动窗完成数为 775/805。功能通过不能写为严格性能全部达标。[分布式核验摘要](evidence/distributed_acceptance_20261104.json)和[当前限制](known_issues.md)给出范围；完整本机数据位于 `/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/`，核心统计见[final_report.json](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_report.json)。
