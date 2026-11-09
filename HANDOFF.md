# EGO-Swarm 与 Kimera-Multi 项目交接

更新日期：2026-11-09，Asia/Shanghai。接手时先读本页，再按所接任务查阅链接。源码仓库为 `/home/mrjq311/swarm_kimera`；`swarm_kimera_runtime` 是运行副本、构建环境和实验档案，不能当成源码仓库或整体可删缓存。

## 接手先确认的事实

- 当前是**两端各自运行完整 Kimera，本机只接收结果**的分布式版本。早期“设备只发传感器、本机算全部 SLAM”的集中方案已保留为历史 baseline，不能混启。
- 2026-11-04 的本地分布式代码基准是提交 `4f8c54b`。通过 GitHub 应用重建提交时，提交 SHA 可能不同；须以 Git tree SHA 核对文件内容，并保留原始本地提交。2026-11-09 发布前，GitHub 的 `main` 与 `snapshot/centralized-baseline-20261104` 均指向集中版本 `e9d3540e148b2e15d53bb959452810d548d4af44`。仓库为 [MrJQ-Huang/EGOswarm_Kimeramulti](https://github.com/MrJQ-Huang/EGOswarm_Kimeramulti)；最新 HEAD 和远端分支以 `git status`、`git log` 和 `git ls-remote --heads origin` 为准。旧文档“准备提交”是历史措辞。
- 最近验收是 **2026-11-04** 的 `distributed_continuity_20261104_153318`：双圈 20/20 航段完成，功能与共享地图连续性核验通过，**AMOV 强实时性能未通过**。
- 2026-11-09 本机只读进程检查未发现 ROS master、RViz、离线查看器或分布式实验进程；两个 hub 容器仍运行，但仅有 init/sleep。两台远端本次未通过 SSH 实时复查；三机关闭回执属于 11 月 4 日历史记录。
- 最近讨论转向无线协议和室外 4G/5G。VSOA、Zenoh、RTM/ENet、蜂窝组网均处于**调研阶段**，没有部署、迁移或性能对比结果。
- 用户已要求将分布式代码和交接文档推送至 GitHub。通信调研建议不代表协议选型或迁移已完成；实验、清理、后续改造和发布范围按用户当次指令执行。

继续阅读：[架构](docs/architecture.md) → [分布式操作](docs/distributed_deployment.md) → [已知问题](docs/known_issues.md)。依赖与 ABI 查 [部署说明](docs/deployment.md)和 [依赖清单](dependencies/README.md)。

## 用户目标与必须保持的约定

1. AMOV、UROS 各控制自己的仿真无人机。任务程序可以指定不同航线的目标点，但导航、避障、轨迹生成及重规划必须由 EGO 完成；不得用脚本直接合成飞行轨迹或复制目标位置。
2. Kimera 的视觉回环与分布式优化负责建立共享坐标关系。不能仅凭两条局部轨迹、仿真出生位姿、真值、人工平移或 ICP 代替视觉对齐。
3. 用户最终关注联合全局地图；轨迹及可见光相关信息的时效优先，允许点云稀疏。当前可见光信息用于本地 VIO、跨机视觉匹配和网格原色；没有为此恢复向本机连续上传全部原始图像。
4. 两端允许代码、配置、进程 state、日志和紧凑诊断落盘；不录原始图像/IMU bag，不导出 VIO CSV、图、PLY、TSDF 或网格二进制。SLAM 必需的关键帧、图、TSDF 和重发状态仍保留在 RAM，不能将“不落盘”解释为“无状态”。本机可存收到的结果与评价数据。
5. 同步以本机仓库为准，覆盖设备差异前备份；保留架构特定依赖、编译产物和历史 runs。ARM64 与 x86-64 二进制不能互拷。
6. 实验完成后正常关闭。GitHub 的分支状态以实际查询为准；后续发布范围依照用户当次指令。

## 主机与环境

以下 IP 是当前代码中的局域网配置，换网络后须重新核验，不能视作室外蜂窝地址。

| 主机 | 身份及地址 | 实验根目录 | 环境 |
| --- | --- | --- | --- |
| AMOV | robot 0，`192.168.5.6`，SSH 用户 `amov` | `/home/amov/Drone/two_pc_ego_20261030` | ARM64 Orin NX，原生 ROS1 Noetic；历史运行 4 核在线、15W |
| UROS | robot 1，`192.168.5.21`，SSH 用户 `uros` | `/home/uros/Drone/egoswarm/experiments/two_pc_ego_20261030` | x86-64 i7-7820HQ；EGO 与 Kimera 分别在 ROS1 容器中 |
| 本机 hub | `192.168.5.14` | `/home/mrjq311/swarm_kimera_runtime/experiment` | 32 逻辑 CPU、128 GB RAM、RTX 5060 Ti；结果显示与评价 |

- 三机各自独立 ROS master，端口均为 `11431`；同一端口号不表示共享 master。
- UROS 容器：`ego-native-full-uros-20261102`、`kimera-native-full-uros-20261102`，实验根目录挂为 `/experiment`。不要误用宿主机 ROS2 工作区。
- 当前 hub 容器：`kimera-central-hub-gpu-20261103`；另有空闲 `kimera-three-host-hub-20261102`。前者名字含 central，但也承载当前结果 observer，不能据名称判断计算架构。
- hub 现有挂载是 runtime experiment → `/experiment`，没有源码仓库 `/source` 挂载；修改仓库不等于已经部署到容器。
- 本机宿主环境本次未发现 `/opt/ros/noetic/setup.bash`；ROS 验证需使用正确环境，不能把宿主机缺 ROS 写成代码回归。
- GPU 当前用于 OpenGL 显示；原生 SLAM 使用 CPU，未实现 CUDA 建图或语义推理。当前为 `geometry_only`，不是完整语义分割复现。
- 认证信息不写入交接或仓库；使用已有 SSH 配置和会话中已授权的认证方式。

## 当前数据流

```text
每台边缘机：本地目标 → EGO → B-spline → traj_server → 连续仿真体
                                                    ↓
                                      本地 RGB-D / stereo / IMU
                                                    ↓
                         VIO → 图坐标适配 / 回投 → TSDF / mesh frontend
                                                    ↓
                              LCD ↔ 对端 LCD；DPGO ↔ 对端 DPGO
                                                    ↓
                              各边缘机 PGMO / LMO 校正各自局部网格
                                                    ↓
                       结果 server → 本机 map observer → RViz / 存档
                       actual/VIO → 本机独立 telemetry observer
```

EGO 规划当前使用仿真机体里程计和本地障碍观测，**Kimera 尚未接入 EGO 导航反馈闭环**。actual 可供仿真控制、任务保护、显示及离线评价，不能喂入 VIO/LCD/DPGO/TSDF 冒充估计结果。仿真场景也不能直接当作重建地图。

共享 frame 为 `kimera_world`，gauge 以 robot 0 初始位姿为锚；它是任意参考系，不等于 GPS/地理坐标。同名 frame 或相同 `gauge-id` 不证明对齐成功。共享证据就绪前，局部地图保留在各自 `sim_N/vio_odom`；本机不做 ICP、独立配准、优化或焊接来补出联合地图。

### 当前跨机协议

| 链路 | 当前实现 | 主要语义 |
| --- | --- | --- |
| AMOV ↔ UROS | robot 0 拨号 robot 1 的 `11640/TCP`，control/feature/ego 三条独立全双工 socket | 自有 peer bridge，传原始 ROS 序列化字节与来源 metadata，开启 `TCP_NODELAY` |
| 各机 → hub 地图 | 边缘 server `11740/TCP`，hub 主动连接 | 各 stream 只保留最新快照，zlib；可选 64 KiB 字节块无损 delta，失基线补全量 |
| 各机 → hub 遥测 | 边缘 server `11741/TCP`，hub 主动连接 | actual/VIO，独立于地图连接与 observer 进程，默认上限 30 Hz |
| 机内 ROS1 | TCPROS；XML-RPC 管理 | 各自主机图；SSH 只用于管理，不承载实验数据 |

peer 白名单包括 BoW/VLC 请求响应、回环及 ACK、DPGO 矩阵/公共位姿/权重/状态等，以及 EGO B-spline 和任务状态；原始图像/IMU 留在边端。只转发位姿话题无法替代完整协同协议。

control/feature 使用有界 RAM 队列、序号、ACK/重试、去重和 run/cutoff/boot/type/MD5/hash 校验；EGO 有新鲜度策略。ACK 表示对端 proxy 发布返回，**不等于原生算法完成消费**。同进程重连可保留状态；peer boot 改变、关键数据过期或可靠队列溢出需新 run，未实现无盘进程重启续图。

map 与 telemetry 已拆成两个 observer 进程，避免巨大 mesh 在同一 Python GIL 下拖慢轨迹。地图原始 ROS 字节保留顶点、原色、三角面及元数据；差分支持历史顶点修改、删除/截断和拓扑变化，不是只追加新点。

### 共享地图连续性修复

同一有效 native PGMO/gauge 会话内，保留最后已核验 path；新的 path pending 不再无谓撤销共享地图。新增 latched `verified_body_path`，observer/导出器按 mesh receipt 的精确内容 hash 关联路径，不能只取 latest。

`last-verified` 不证明最新 pending path 已使用；metadata 与 payload 的伴随原子封装、应用 snapshot ID 和持续 gauge 均不等于原生 optimizer/mesh 共享原子轮次。目前证据仍是观测性、非原子关联，不能放松 source/boot/hash 校验来补出成功。

## 最新实验与性能证据

权威入口：[分布式验收 JSON](docs/evidence/distributed_acceptance_20261104.json)、[已知问题](docs/known_issues.md)。

最新 run：`distributed_continuity_20261104_153318`。共同 cutoff 为 `1793777598.251462`，运动窗口为 `1793777684.850971` 至 `1793778181.4345658`，共 496.584 秒。这些值只用于定位历史证据，不能复用为新实验 epoch。

| 11 月 4 日完整运动指标 | AMOV | UROS |
| --- | ---: | ---: |
| 已完成航段 | 20/20 | 20/20 |
| 实际路径 | 159.882 m | 159.140 m |
| 联合 body path ATE | 0.19916 m | 0.20810 m |
| 运动窗原生 TSDF 完成次数 | 775 | 805 |
| 首次共享接纳后 accepted mesh 更新间隔 P95 | 13.517 s | 2.946 s |
| 同口径最大更新间隔 | 22.862 s | 4.991 s |
| 整机平均 CPU | 90.75% | 38.82% |

联合 ATE 仅拟合一次团队 SE(3)，不逐机对齐、不估尺度；评价变换不反馈给地图或估计。最终联合网格保留 109,117 个顶点槽，其中 67,033 个被面引用，以及 146,846 个原色三角面。均为仿真条件下的结果；验收字段 `strict_realtime_certified=false`。

本轮配置：camera 5 Hz、IMU 100 Hz、目标 TSDF cloud 2 Hz、mesh period 2 s、map 上限 0.5 Hz、telemetry 上限 30 Hz、双圈、delta 开启、已核验 OpenGV guard 开启。两端 TSDF voxel 显式为 **0.20 m**，CLI 默认仍为 **0.15 m**。RViz 绘制 FPS 不等于传感器或建图更新率。

历史验证：Noetic 环境 `make test` 共 294 passed、0 skip；174 个托管运行源码/配置跨本地与三主机 SHA256 一致。这是 11 月 4 日证据，不含架构特定 underlay/二进制，也不代表今天重新跨机核验。

### 通信复析与优先瓶颈

11 月 6 日分析使用上述 11 月 4 日数据，未重新采集。先看本机 [HTML 报告](/home/mrjq311/swarm_kimera_runtime/communication_review_20261106/report.html)。

- 专用约 60 s TCP 窗口：hub map 接收均值 **11.841 Mbit/s**，telemetry **0.851 Mbit/s**；地图约占两者合计的 **93.3%**。
- 同窗口发送端内核平滑 RTT P95：AMOV/UROS peer **32.938/40.079 ms**，map **62.295/31.332 ms**，telemetry **21.019/12.849 ms**。
- 同窗口 actual 到达约 **29.148/29.852 Hz**，数据龄 P95 **92.321/20.295 ms**，最大 callback 间隔 **162.912/71.785 ms**。
- 邻近运动末 AMOV 本地 control 观察/处理队列等待高水约 **9.349 s**，不是飞控执行延迟；结合 CPU 与地图更新间隔，须优先排查本地处理/调度，不能直接认定 TCP 是主因。
- `mesh` 与 `accepted_mesh` 的 canonical 内容重叠约 **98.5%/97.1%**，两者当前独立回传，值得按需去重；这不是已经实现或测得的同等比例网络节省。保留关键协同消息，不为省流量删掉视觉回环或 EGO 输入。

### 统计口径不能混用

- 数据龄＝接收回调时间−源 stamp，含时钟偏差、估计处理、排队和传输；不是纯网络单向时延。
- mesh Header 使用原生 `vertex_stamps.back()`；它的年龄不是相机到屏幕的显示时延。
- RTT 是约 60 s 内的内核平滑样本；重传字节/TX 比例不是丢包率。netdev 流量、TCP 流量与 PHY 空口占用也不是同一层，不能直接相加。
- 100 Hz actual 源被 latest-only 策略限到约 30 Hz，不是网络丢失了 70%。双源戳配对率不是硬件同步采集率。
- UROS 的 selected PID CPU/RSS 只覆盖 native 容器，漏掉 EGO 容器；整机 CPU/可用 RAM 才可按相同口径比较。
- 11 月 6 日 RTT 复析采用线性插值分位数，与原验收摘要的顺序统计口径略有差别；整组引用同一来源，不混表。
- 尚无完整 PCAP、PHY/MCS/RSSI/airtime/MAC retry、隔离无线容量、可靠校时误差界、端到端 native 消费 ACK、逐 topic 压缩 wire bytes 或真实屏幕延迟。新测量不能补写成 11 月 4 日事实。

## 重要故障和恢复边界

| 历史问题 | 已完成处理及仍须保持的边界 |
| --- | --- |
| 首次分布式运动 9/20 航段后来源认证队列溢出 | 原消息与独立 receipt 改为 metadata＋原始 payload 伴随封装；禁止新旧端混搭 |
| combined observer 的大地图处理拖慢遥测 | map/telemetry 已分进程，不能据此声称 AMOV 地图积压全部解决 |
| 冷启动 master 未 bound，多个 roslaunch 争抢 UUID | manager 加入本轮 master PID/XML-RPC/UUID 屏障；不能忽略冲突继续旧 run |
| 新 path pending 导致共享图被撤销、旧 path 关联丢失 | last-verified continuity 与 latched verified stream；仍需精确 hash 和原生身份关联 |

第二轮在飞行中退出旧 combined observer，任务继续 stage 18→20；第四轮 map-only 退出发生在运动结束后；第五轮新缓存 map observer 重启也在运动结束后。三者范围不同，**observer 退出不等于 peer 网络分区**。未验证完整 peer 失联、进程重启无盘续图、真实飞控、硬件同步或 Kimera 驱动 EGO 闭环。

## 室外通信调研的结论与未决事项

以下是最近讨论记录与后续设计方向，没有成为当前运行实现。

### 原论文与作者实现

| 来源 | 已确认内容 | 一手链接 |
| --- | --- | --- |
| Kimera-Multi ICRA 2021 与 TRO 2022 | 描述 ROS 通信及 BoW、按需关键点/描述子、回环和公共位姿交换；未明确写 RTM/ENet/UDP | [ICRA](https://arxiv.org/pdf/2011.04087)、[TRO](https://arxiv.org/html/2106.14386v2) |
| Kimera 2023 实机扩展，第 III、IV 节 | RTM＝Remote Topic Manager，基于 ENet/UDP，选定流可靠有序；跟踪连接和延迟/丢包；实机为 2.4 GHz 无线网络 | [作者论文](https://arxiv.org/html/2304.04362v1) |
| Kimera 当前官方仓库 | 跨机 peer-to-peer RTM；RTM 仍标 Coming Soon/审批，不能承诺完整作者实现可直接安装 | [官方 README](https://github.com/MIT-SPARK/Kimera-Multi#system-architecture--breakdown) |
| EGO-Swarm ICRA 2021，第 V-B 节 | 广播共享未来 B-spline，典型轨迹小于 0.5 KB；生成即发并周期重复；稳定连接用于校时和顺序启动 | [原论文](https://arxiv.org/pdf/2011.04183) |
| EGO 论文时期作者桥 | UDP 8081 广播单条轨迹/odom，TCP 8080 串链 MultiBsplines；不能把它写成全 UDP | [固定版本源码](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/blob/fea42fc2b891f73516c2597468b8c0fddb7848a8/src/planner/rosmsg_tcp_bridge/src/bridge_node.cpp) |

作者 EGO 当前接收代码有 `abs(now-start_time)>0.25s` 拒收检查，受时钟偏差、处理和网络共同影响；不是论文给出的蜂窝时延保证，也未确认是本项目当前阈值。[作者代码](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/blob/master/src/planner/plan_manage/src/ego_replan_fsm.cpp#L223-L233)

### 候选协议与蜂窝拓扑

- **ENet** 与 Kimera 实机通信路径最接近，提供可靠/不可靠消息、多通道、分片重组、RTT/丢包统计和节流。可靠流仍可能有通道内排序等待，不能泛称“UDP 就没有积压”。[作者设计](https://github.com/lsalzman/enet/blob/master/docs/design.dox)
- **VSOA** 同时使用 TCP/UDP；quick 走 UDP，大块 Stream 仍走独立 TCP。普通 URL＋payload 上限约 256 KiB，quick 约 65 KB；官方标明 quick 不适用 NAT 网络。蜂窝场景需要另验网关/隧道，不能推成所有 UDP 都不能穿 NAT。Python SDK 要求至少 3.10，C/C++ SDK 包及本机适配需进一步核验。[通信说明](https://docs.acoinfo.com/vsoa/basicdev/vsoa.html)、[Stream](https://docs.acoinfo.com/vsoa/premiumdev/vsoa_framework/basicdev/vsoa_framework_stream_manual.html)、[Python SDK](https://github.com/acoinfo/vsoa-python)
- **Zenoh** 可在 Wi-Fi/4G/5G 提供的 IP 网络上运行，支持 peer/router/client 拓扑；有 ROS1 桥，但核心版本、桥版本和所需 UDP/QUIC 能力须一起固定验证，不能把最新核心功能当成现成桥功能。[部署](https://zenoh.io/docs/getting-started/deployment/)、[ROS1 桥](https://github.com/eclipse-zenoh/zenoh-plugin-ros1)
- DDS/Fast DDS 与 eCAL 是对照候选；大网格分片与恢复仍是关键，best-effort 丢一片可能损失整条消息，不能一律将大地图改为裸 UDP。[Fast DDS](https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html)、[eCAL](https://eclipse-ecal.github.io/ecal/stable/advanced/layers/tcp.html)

室外没有现成 LAN 时，讨论的纯蜂窝方案是：AMOV、UROS 各自 SIM 上网，双方和 hub 都主动连公网可达通信节点，节点只转发，Kimera/EGO 仍本地计算。VPN 是另一种建立可达地址的方案；不要默认三层虚拟网络能自动承载原 LAN 广播。蜂窝 CGNAT 下两个 SIM 不等于可直连，需记录实际直连/中继路径。[CGN 说明](https://www.rfc-editor.org/info/rfc6598/)、[连接类型](https://tailscale.com/docs/reference/connection-types)

如允许额外近距离机间无线链路，可优先承载 EGO 邻机轨迹，蜂窝负责远程地图/监控。完全依赖公网时则须设计逻辑广播、轨迹时效、校时和断连行为。两种方案都未实施，当前固定 IP 拨号程序不能插卡后直接复用。公网转发保留分布式计算，但会引入通信节点依赖。

## 文件入口与档案位置

| 任务 | 文件 |
| --- | --- |
| 角色编排、环境、进程归属与关闭 | [manager.py](experiment/three_host_20261102/distributed/manager.py)、[runtime_control.py](experiment/three_host_20261102/distributed/runtime_control.py) |
| peer 白名单、可靠传输与来源契约 | [peer_topics.py](experiment/three_host_20261102/distributed/peer_topics.py)、[peer_bridge.py](experiment/three_host_20261102/distributed/peer_bridge.py)、[peer_wire.py](experiment/three_host_20261102/distributed/peer_wire.py) |
| 结果快照、delta、接收与导出 | [result_stream.py](experiment/three_host_20261102/distributed/result_stream.py)、[result_wire.py](experiment/three_host_20261102/distributed/result_wire.py)、[observer.py](experiment/three_host_20261102/distributed/observer.py)、[export.py](experiment/three_host_20261102/distributed/export.py) |
| 原生网格、PGMO 与 gauge 关联 | [native_dense_bridge.py](experiment/harness/multimodal/native_dense_bridge.py) |
| 本地任务、运动与传感器 | [local_mission.py](experiment/three_host_20261102/distributed/local_mission.py)、[continuous_body.py](experiment/harness/multimodal/continuous_body.py)、[sim_sensors.py](experiment/harness/multimodal/sim_sensors.py) |
| 质量、通信和遥测测量 | `distributed/{quality_collector,quality,mesh_quality,transport_metrics,telemetry_metrics,audit}.py` |
| 依赖版本及补丁 | `dependencies/{source_manifest.json,patches,locks}`、`experiment/native_full_stack_20261102/{source_lock.json,build_recipe.json}` |

运行索引为 [current_distributed_run.json](/home/mrjq311/swarm_kimera_runtime/bootstrap/current_distributed_run.json)，记录最近 run 和参数；不把它的 phase 当作当前 OS 进程真相。

最新完整档案根目录：

```text
/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/
```

其中 `final_report.json`、`closure_receipts.json`、`deployed_source_verification.json`、`continuity_runtime_audit.json`、`fresh_observer_restart.json`、`trajectory_quality.json`、`mesh_quality.json`、`transport_metrics.json`、`telemetry_metrics_60s_heavy.json` 是主要入口。`observer/` 保存原生 `.rosbin` 与 metadata；`edge_evidence/` 保存两端日志/指标；`final_native_map/global_native_mesh.ply` 和 `export_manifest.json` 为最终原色地图与来源。

通信复析位于 `/home/mrjq311/swarm_kimera_runtime/communication_review_20261106/`：`report.html`、`transport_analysis.json`、`host_network_analysis.json`、`pipeline_analysis.json`、`telemetry_stream_analysis.json`，以及 CSV、图表和分析脚本。同名 `.zip` 为分享包。这些结果和上面的原始档案均未纳入 Git。

`swarm_kimera_runtime` 还含 bootstrap、underlay/overlay、`provenance_source_configs/` 和 `cleanup_manifests/`；`/home/mrjq311/swarm_kimera_sync_20261103_131947` 备份目录本次也仍存在。早期“可重建即可清理”的授权不能扩大为删除当前唯一实验档案。清理时逐项区分源码、构建缓存、唯一历史证据与有其他副本的派生文件。

## 操作与验证入口

先做无进程副作用的检查：

```bash
cd /home/mrjq311/swarm_kimera
git status --short --branch
git log -1 --format='%h %s'
python3 experiment/three_host_20261102/distributed/manager.py --help
python3 experiment/three_host_20261102/distributed/manager.py results \
  --host amov --run-id REVIEW_ONLY --cutoff 1793760000 \
  --gauge-id REVIEW_ONLY.robot0-initial --delta --dry-run
```

`REVIEW_ONLY` 和旧 cutoff 仅用于预览。`--dry-run` 仅支持启动角色预览；与 `stop` 或 `status` 组合会直接报错，不执行该动作。`status` 不启停角色，但会打开/创建 manager lock，应在对应主机的正确实验根目录执行。

获准开始新实验后，完整流程以 [分阶段操作](docs/distributed_deployment.md) 为准，不要把下列摘要当作无条件一键命令：

1. 三机共用一次生成的新 run/cutoff/gauge；分别启动 master 并通过 PID/XML-RPC/UUID 就绪检查。
2. 两端 supervisor、一次 reset；依次准备 adapter、local-frontend、backend、dense、peer、results、witness、resource，然后 VIO。传感器暂不启动。
3. hub 启动独立 map/telemetry observer、质量与资源采集和需要的 RViz。
4. 两端启动 sensors，静止预热；核验原生 key 0、真实 TSDF completion、LCD/DPGO/PGMO、peer 来源和局部 gauge。共享 gauge 由真实跨机视觉约束及优化建立，不要求静止预热时已经共享；第五轮首次共享接纳在运动开始约 65–69 秒后。
5. 仅生成一次共同的未来 `START_AT`，两端以 `mission ... --laps 2 --start-at "$START_AT"` 放行。重现第五轮时使用 `local-frontend ... --voxel-size .20`，results 显式 `--delta`；OpenGV guard 仅在各自架构已核验时启用。省略号代表前述共同身份参数，不是可直接复制的 shell 命令。
6. 完成后两端先 `stop --roles mission`，确认 planner/traj_server 停止、continuous_body hold，再收尾输入与后端、保存本机结果并关闭剩余角色。核对 closure 和残留进程。

distributed `stop` 不带 `--roles` 会关闭该 run 的全部所属角色，**包括 master**；没有旧管理器的 `--stop-master`。禁止全局 `pkill`。只停 `observer` 仅停止地图，全结果接收退出需要同时停 `observer observer-telemetry`。不要在保留 SLAM 历史的同一 run 中 reset、换配置或重启 peer 假装续图。

修改运行代码后的契约测试入口为：

```bash
make test
# 等价于两组 unittest discover；需匹配的 ROS1 Noetic Python 环境
```

历史日志为 `/home/mrjq311/swarm_kimera_runtime/bootstrap/distributed_final_noetic_tests.log`。确认测试实际读取修改后的仓库，不能直接在旧 runtime 副本运行后声称新源码通过；现有容器没有 `/source` 挂载。纯文档交接不要求重跑飞行或原生构建。

`SOURCE_MANIFEST.json` 和 `docs/evidence/publication_validation.json` 对应 11 月 4 日 260 文件验证快照；本次只新增本页，不改写该历史清单或验收记录。后续发布新源码时，应生成新的验证记录，并保留旧证据。

## 后续工作的承接顺序

下一步由用户的新指令确定；最近讨论尚未决定通信迁移方案。若继续通信优化，建议按以下依赖推进：

1. 先分解 AMOV 本地处理、队列与网络耗时，精简不必要的结果镜像流，保持 EGO/遥测独立和来源校验。
2. 明确部署为现场直连、纯蜂窝公网路由还是双链路，再选择 RTM 风格 ENet、Zenoh 或 VSOA；核对 ROS1、ARM64/x86、SDK/桥版本与可获取依赖。
3. 用同载荷、同频率和同计算负载做对照；若改 C++ 桥，增加 C++ TCP 基线，分离语言重写与协议收益。
4. 比较数据龄 P95/P99、AMOV CPU、逐类 wire bytes、队列、完整网格刷新、断连恢复与时钟误差。控制/视觉约束需可靠，过期轨迹需显式处理，地图 delta 失基线须重同步。
5. 按新的明确范围执行原生集成、双圈运动和故障恢复验收，再处理用户要求的 GitHub 发布。不要用单元测试、小包 ping、相同 frame 标签或一次最终地图代替整链路证据。
