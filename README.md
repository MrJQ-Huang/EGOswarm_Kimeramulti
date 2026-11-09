# EGO-Swarm 与 Kimera-Multi 双机协同建图实验

当前工作入口是 [distributed/manager.py](experiment/three_host_20261102/distributed/manager.py)：AMOV、UROS 各自运行 EGO-Swarm、连续仿真体、传感器及完整原生 Kimera 计算链；两机直接交换协同消息。本机使用第三个独立 ROS master，只接收结果、显示、存档和离线评价。设备不录制原始传感器或导出地图文件，SLAM 所需历史状态保留在内存。

**功能改造与共享地图连续性修复已完成，AMOV 强实时性能仍未达标。** 第五轮 `distributed_continuity_20261104_153318` 完成两圈 20/20 航段，运动 496.58 秒；AMOV/UROS 实际路径为 159.8820/159.1399 m，最小机间分离为 2.04876/2.06328 m。两端原生链路持续工作，本机完成原色联合地图、显示与评价；三主机本轮角色均已正常关闭，RViz 已关闭。见[分布式核验摘要](docs/evidence/distributed_acceptance_20261104.json)与[完整运行报告](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_report.json)。

第五轮沿用两端 `--voxel-size .20` 与原 FPS/CPU 参数。运动期间 AMOV/UROS 实际 TSDF 完成 775/805 次；首次共享接纳后的 accepted mesh 更新间隔中位数为 3.7527/1.9996 秒、P95 为 13.5174/2.9459 秒、最大值为 22.8619/4.9907 秒。AMOV 整机平均 CPU 为 90.75%，仍有计算排队，不能把持续积分、功能通过或平均近 30 Hz actual 接收写成地图零延迟、强实时或全部性能达标。0.20 m 是原生 TSDF 分辨率配置（默认 0.15 m），不是 observer 抽样、ICP 或丢弃原生面。

连续性修复在同一有效 PGMO/gauge 会话内区分 last-verified 与 pending path，新增 latched `verified_body_path`；新 path 待核验不会无谓撤销已建立的共享 gauge。第五轮连续性档案审计全部检查通过；运动后以全新 RAM 缓存重启 map observer，复核 shared map 与两端精确 hash 关联的 verified path 成功，未连续测恢复时延。它没有证明最新 pending path 已被使用，也不是共享原子 optimizer/mesh generation。第四轮最大 31.183 秒 accepted 更新间隔等修复前证据仍保留在[已知问题](docs/known_issues.md)。

第五轮一次共同 SE(3) 的 body path ATE 为 AMOV/UROS 0.1991626/0.2081018 m，配对支持率为 99.9051%/99.9268%；独立局部 VIO 基线为 0.39959/0.32367 m。联合原生导出保留 109,117 个顶点槽，其中 67,033 个被面引用，以及 146,846 个三角面和原色，见[完整 PLY](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_native_map/global_native_mesh.ply)和[导出来源清单](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_native_map/export_manifest.json)。面引用顶点到仿真解析表面的平均误差/RMSE 为 0.122169/0.166256 m；有限可见真值采样在 0.15/0.30/0.50 m 阈值的覆盖为 70.05%/95.61%/99.91%。这些是有采样与距离定义限制的仿真评价，不是原论文完整评价复现；评价只复用轨迹的一次团队 SE(3)，不修改地图、不反馈给 SLAM。

第二轮旧 combined observer 在飞行期间实际断开 24.1505 秒，两端阶段从 18 推进到 20，支持飞行独立于观察端的结论；第四轮 map-only 实际断开 23.6117 秒，发生在运动完成后，两端仍为 20/20，TSDF 完成数增加 42/43，只证明该时段局部积分与 peer 继续，不能当作第四轮飞行中断连测试。两次显式等待目标均为 22 秒，实际时长另含检查开销。

首次运行 `distributed_native_20261104_132619` 因 UROS 独立来源回执匹配队列溢出，在完成 9/20 航段后受控停止。第二轮已采用 metadata 与原始 payload 绑定的本机伴随流，以及有界历史 body path 缓存和评价真值覆盖等待；首次失败仍作为历史证据保留，见[已知问题](docs/known_issues.md)。

## 当前分工

| 部分 | 当前分布式 profile |
| --- | --- |
| AMOV / UROS | 分别是 robot 0 / 1，各有独立 master、仿真与本地 EGO Planner、traj_server |
| 本地定位与建图 | 每机原生 VIO、图坐标适配、RGB-D 回投、Voxblox TSDF、mesh frontend、LCD、DPGO、PGMO |
| 机器人间通信 | 自有直接 TCP peer bridge；控制、特征、EGO 轨迹分通道，包含请求、响应、ACK 及恢复契约 |
| 任务 | 各机本地任务状态机只发布目标 PoseStamped，互通阶段状态；轨迹、避障、重规划由 EGO 完成 |
| 本机 | 独立 map / telemetry 两个 observer 进程、原色三角网格显示、结果存档与离线评价；不重新估计、配准或优化地图 |
| 结果链路 | 完整原生 ROS 字节快照；可选有基线的无损字节块 delta，失基线补全量；actual/VIO 遥测独立通道 |
| 语义与 GPU | 当前为 geometry-only，无语义标签推理；本机 GPU 用于 OpenGL，原生 SLAM 仍使用 CPU |

规划仍使用仿真机体里程计与本地障碍观测，Kimera 位姿/地图尚未接回 EGO 导航闭环。actual 仅供仿真控制、任务保护、显示和离线评价，不是 VIO、LCD、DPGO 或 TSDF 的估计输入。

本项目使用固定版本 Kimera 原生组件、现代 PGMO MESH API 兼容移植和自有传输/接口代码，并非 Kimera-Multi v1 官方依赖组合的原封部署。peer bridge 参考上游消息清单，**不是原作者的 Remote Topic Manager**；源码版本、ABI、消息 MD5 和来源契约仍需核验。

## 使用入口

1. 阅读[架构与数据流](docs/architecture.md)，确认规划、估计、共享 gauge 和显示的边界。
2. 按[通用环境部署](docs/deployment.md)准备两种架构的依赖，再按[分布式部署与分阶段启动](docs/distributed_deployment.md)操作。
3. 使用同一新 run ID、cutoff 与 gauge 配置，先 `--dry-run` 审查命令，再分阶段初始化、静止预热、运动与验收。
4. 用[已知问题与验收条件](docs/known_issues.md)判断结果；窗口刷新、共享标签或单次网格输出均不能代替持续积分与质量证据。

```bash
python3 experiment/three_host_20261102/distributed/manager.py results \
  --host amov --run-id REVIEW_ONLY --cutoff 1793760000 \
  --gauge-id REVIEW_ONLY.robot0-initial --delta --dry-run
python3 -m unittest discover -s experiment/three_host_20261102/tests -p 'test_*.py'
python3 -m unittest discover -s experiment/harness/multimodal -p 'test_*.py'
```

上述 cutoff 仅用于无副作用预览；实际实验必须重新生成共同 epoch。最终 Noetic 环境 294 项测试通过，无 skip；174 个托管运行源码/配置文件在本地与三主机逐项 SHA256 一致。GitHub 当前只有 baseline snapshot，分布式 feature 尚在本地准备提交，不能将其当作已经公开发布。软件契约测试与原生运行、双机运动及性能核验分别留证。

## 结果与保存

本机默认保留收到的完整原生 mesh ROS 字节、metadata、轨迹和紧凑 actual/VIO 数据；三角面与原始相机颜色均保留，不再逐 snapshot 生成 PLY。最终离线 PLY 导出或显式 `--snapshot-ply` 包含双精度 XYZ、完整面，以及保留原始 RGBA 数值的 `native_*` 字段；普通 8 位颜色是显示派生字段，完整 ROS 文件保留其余原生元数据。

同名 `kimera_world` 或相同 `gauge-id` 不等于已经建立共享坐标。两端真实跨机约束、完成的原生优化观测和网格 receipt 就绪前，各自局部地图在独立 `sim_N/vio_odom` 中显示。结果断连时保留 last-known 并标 stale。应用 snapshot ID、原子传输封装与共享原子优化轮次是不同概念。

本机质量评价对两机联合 body path **只拟合一次共同 SE(3)**，不逐机器人对齐联合结果、不估计尺度、不把评价变换反馈给估计或地图显示。VIO 的独立局部 gauge 仅作为明确标注的单机基线。未匹配样本、时间覆盖与缺失机器人必须报告。

## 仓库结构

```text
experiment/
  configs/{amov,uros}/multimodal/   # 每机 VIO 标定、参数与 flags
  harness/                         # EGO 仿真、传感器和原生 Kimera 接口
  native_full_stack_20261102/       # 构建记录、环境与历史管理器
  three_host_20261102/
    distributed/                   # 当前独立 master / 本地 SLAM / peer / observer
    manager_hub.py                  # 保留的集中 baseline
    manager_local.py                # 集中 baseline 的设备端入口
dependencies/                       # 固定源码、补丁与许可证
docs/                               # 架构、部署、状态与公开统计摘要
```

不上传 raw rosbag、实验点云/地图、编译产物、容器镜像或访问凭据。上游源码保留版本与补丁，在 ARM64/x86-64 上分别编译，不能互换二进制。

## 保留的集中 baseline 与历史结果

[manager_hub.py](experiment/three_host_20261102/manager_hub.py) / [manager_local.py](experiment/three_host_20261102/manager_local.py) 保留为集中计算 baseline；该模式由设备上行原始传感器、本机运行全部 SLAM。它与当前分布式 profile 互斥，不混用角色或沿用旧 run。

2026 年 11 月 3 日集中版本完成一次两圈 20 航段运动，用时 498.922 秒；实际路径 AMOV/UROS 为 160.288/159.045 米，飞行期间真实 TSDF 积分 2408/2414 次，最终联合结果含 202,567 顶点和 258,118 三角面。[集中版本验收摘要](docs/evidence/motion_acceptance_20261103.json)只证明该次集中仿真运行，不是当前边端分布式计算、严格原子优化、SLAM 精度或任意网络条件的验收。

2026 年 11 月 2 日历史双机版本虽然飞完约 500.45 秒，但 AMOV 末尾约 390 秒无新 TSDF 积分，全局同轮校验未通过。相关串行积压与空全局显示是该历史版本的证据，不能直接当作当前 source 的运行状态。[历史实验摘要](docs/evidence/concurrency_analysis.json)

## 上游工作

- [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
- [Kimera-Multi](https://github.com/MIT-SPARK/Kimera-Multi) 与[原论文](https://arxiv.org/abs/2106.14386)
- [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)、[Kimera-PGMO](https://github.com/MIT-SPARK/Kimera-PGMO)、[Voxblox](https://github.com/ethz-asl/voxblox)

第三方项目沿用各自许可证。再分发与引用应核对 `dependencies/` 中保存的来源、版本、许可证及补丁。
