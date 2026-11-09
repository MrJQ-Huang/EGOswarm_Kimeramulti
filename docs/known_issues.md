# 已知问题与验收状态

当前工作入口是分布式 `distributed/manager.py`。**第五轮功能改造与共享连续性修复完成；AMOV 强实时性能未通过（`strict_realtime=false`）。** 第五轮完整运动、地图/路径关联、新观察进程缓存恢复和三主机关闭均有证据；最大 22.8619 秒的 accepted mesh 更新间隔仍必须保留。集中 baseline 与前四轮历史资料分别留证。[分布式核验摘要](evidence/distributed_acceptance_20261104.json)

## 第五轮：功能与连续性修复完成，保留 CPU 时效限制

`distributed_continuity_20261104_153318` 沿用两端 0.20 m TSDF 和第四轮 FPS/CPU 参数，完成两圈 20/20 航段、496.58 秒运动。三主机 closure 均为 true、`remaining=[]`，RViz 已关闭。最终 Noetic 294 项测试无 skip 通过；174 个托管运行源码/配置文件在本地及三主机逐项 SHA256 一致，架构特定二进制、underlay 和历史数据不在比较范围。GitHub 当前仍只有 baseline snapshot，分布式 feature 正在本地准备提交。

| 完整运动窗口指标 | AMOV | UROS |
| --- | ---: | ---: |
| 实际累计路径 | 159.8820 m | 159.1399 m |
| 各端记录的最小机间分离 | 2.04876 m | 2.06328 m |
| 原生 TSDF 实际积分完成 | 775 次 | 805 次 |
| 第一次共享接纳距运动开始 | 69.026 s | 65.392 s |
| accepted mesh 接纳次数 | 82 | 219 |
| 首次共享接纳后更新间隔中位数 | 3.7527 s | 1.9996 s |
| 首次共享接纳后更新间隔 P95 | 13.5174 s | 2.9459 s |
| 完整运动统计最大 accepted 更新间隔 | 22.8619 s | 4.9907 s |
| 联合 body path ATE（一次团队 SE(3)） | 0.1991626 m | 0.2081018 m |
| 运动窗源时间配对支持率 | 99.9051% | 99.9268% |
| 独立局部 VIO 基线 ATE | 0.39959 m | 0.32367 m |

这些统计均来自完整运动窗口。更新间隔分位数在首次共享接纳后计算，初始等待单列；最大值也核对到运动结束的边界。TSDF 持续积分与 accepted mesh 更新是不同事件，不能用前者掩盖后者的长缺口。原生 mesh Header 是 `vertex_stamps.back()`，不是发布时间或相机到显示时延。

第四轮审查确认，新 path 提交时清除 echo 会不必要地撤销已建立的共享 gauge，使真实 PGMO mesh 也被挡住。第五轮已修复：同一有效 gauge/PGMO 会话保留 last-verified 状态，receipt 标明较新 path 是否 pending；latched `verified_body_path` 使 observer/导出按精确 hash 取得真正已核验路径。不可变档案连续性审计 `all_checks_passed=true`，含原生 XYZ/三角面确实变化、双方跨机完成证明、稳定 PGMO 实例与 gauge epoch、verified path 精确 hash/来源关联。最新提交、原生已核验使用与已发布地图仍须区分，不能声称最新 pending path 已被使用，也不提供精确 optimizer/mesh generation 证明。

运动后以全新 RAM 缓存重启 map observer，稍后复核 shared map 与两端 exact-hash verified path 关联成功；telemetry observer 保持运行。本次没有连续监测首次恢复时刻，重启到复核的时间间隔不能当作恢复耗时或纯网络时延，也不证明瞬时恢复。此核验不等于飞行中全结果接收退出或 peer 网络分区。

新结果流 topic 为 `/sim_N/native_dense/verified_body_path`，`nav_msgs/Path`、caller `/sim_N/native_dense_bridge`，不替换原 `approved_body_path` 契约。latched verified path 独立于有界历史缓存，可在 observer 新连接时提供最后已核验版本；旧档案只有 `body_path` 时仍按 receipt hash 读取，缺匹配内容时拒绝，不能用 latest 替代。

T0+300 秒起的 60 秒重载窗口中，actual 到达频率为 AMOV/UROS 29.148/29.852 Hz，source age P95 为 92.321/20.295 ms，最大 callback gap 为 162.912/71.785 ms。双 actual 最新源戳采样 eligible 覆盖为 99.9333%，其中源戳差严格小于 100/50 ms 的比例为 96.9313%/72.8486%。这些统计包含主机时钟偏差，不能当作单向时延或同步采集帧。四路采集文件均核对 RID，无 callback 拒绝或传输计数器重置，hub map/telemetry TCP payload 平均接收约 11.841/0.851 Mbps；RTT 与重传计数不等于单向时延或丢包率。

| 全运动资源观测 | AMOV | UROS | 本机 |
| --- | ---: | ---: | ---: |
| 整机平均 CPU busy | 90.75% | 38.82% | 2.31% |
| 选定实验进程 RSS 总和 P95 | 4.25 GiB | 3.38 GiB | 0.81 GiB |

RSS 是进程 RSS 相加，共享页可能重复，不是独占物理内存。CPU 是整机 busy，包含其他工作，三机分别有 4/8/32 个可用逻辑 CPU。GPU 平均利用率为 4.92%，显存 P95 为 551 MiB，统计覆盖该 GPU 的所有任务；NVIDIA OpenGL 用于 RViz，原生 SLAM 仍使用 CPU。AMOV 的高 CPU 与队列积压、22.8619 秒峰值更新间隔限制仍在，功能改造完成不等于强实时性能通过，也不支持把问题全归因于 UROS 内存。

联合原生导出保留 109,117 个顶点槽、67,033 个面引用顶点和 146,846 个三角面及原色，原生 XYZ/颜色只拼接、面索引只偏移。schema 2 几何评价对全部面引用顶点计算到仿真解析表面的最近无符号距离，团队平均/RMSE 为 0.122169/0.166256 m；有限可见真值采样到最近面引用顶点在 0.15/0.30/0.50 m 内的覆盖为 70.05%/95.61%/99.91%。未引用缓冲槽只从主评价与覆盖 KDTree 排除，原始地图/导出没有删除它们。

几何评价只复用 body path 的一次团队 SE(3)，不做 ICP、缩放或各机独立拟合，也不把变换/真值反馈给 SLAM。覆盖采用每机最多 150 个时间采样视角、80×60 相机射线及 0.1 m 真值 voxel 去重；它是顶点距离近似，不是精确点到三角面距离或均匀表面积采样，也未评价语义或复现原论文全部协议。地图 receipt 已在上游核验，几何工具本身不重新证明原生优化。

完整本机数据位于 `/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/`：

- [最终运行报告](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_report.json)、[任务摘要](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/completed_missions.json)、[三主机关闭回执](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/closure_receipts.json)。
- [连续性档案审计](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/continuity_runtime_audit.json)、[新观察进程恢复](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/fresh_observer_restart.json)、[部署源码核对](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/deployed_source_verification.json)。
- [联合轨迹质量](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/trajectory_quality.json)、[schema 2 网格质量](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/mesh_quality.json)、[重载采集报告](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/heavy_capture_report.json)。
- [完整原色 PLY](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_native_map/global_native_mesh.ply)、[原生 ROSbin/receipt/hash 来源清单](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_native_map/export_manifest.json)。

## 第四轮：双圈与联合地图链路完成，保留时效限制

`distributed_final_20261104_150046` 冷启动和双端预热检查通过，完成两圈 20/20 航段，约 498.2 秒。三台主机 closure 均为 true、`remaining=[]`，RViz 已关闭。该轮源码 273 项测试通过；它是 continuity 修复前资料，第五轮修复结果见上节。

| 本轮观测 | AMOV | UROS |
| --- | ---: | ---: |
| 实际累计路径 | 159.33465 m | 158.92723 m |
| 各端记录的最小机间分离 | 1.96705 m | 2.01534 m |
| 联合 body path ATE（一次团队 SE(3)） | 0.1980719 m | 0.20176735 m |
| 运动窗源时间配对支持率 | 99.87187% | 99.89957% |
| 独立局部 VIO 基线 ATE | 0.398898 m | 0.335378 m |
| actual 到达频率（T0+300 秒起的 60 秒窗口） | 28.965 Hz | 29.880 Hz |
| actual source age P95（同一窗口） | 50.965 ms | 19.667 ms |
| 最大 callback 接收间隔（同一窗口） | 413.898 ms | 75.823 ms |

双 actual 最近源时间差小于 100 ms 的比例为 eligible sampler ticks 的 98.9287%，小于 50 ms 为 89.9230%；eligible 覆盖全部 ticks 的 99.5667%。这是独立 50 Hz sampler 对最新已收源戳的统计，不是同步采集或逐帧配对。source age 包含主机时钟偏差、处理、排队和传输；AMOV 还存在负值，不能将其称作真实单向网络时延。平均近 30 Hz 不能掩盖最大约 414 ms 的接收间隔。

AMOV 重负载下监测峰值 accepted mesh 源龄约 12.2 秒、关联 path 源龄约 45.2 秒。网格处理热点优化、移除三路旧派生 PointCloud、复用原始序列化、observer 变化驱动 Marker 与默认不逐 snapshot 生成 PLY 已参与本轮运行，仍不能宣称地图零延迟或严格实时指标全部达标。两端原生 TSDF 均使用 0.20 m voxel size，默认配置仍为 0.15 m；这是本地重建分辨率配置，不是观察端抽稀、ICP 或删面。

第四轮联合地图保留 107,002 个原生顶点槽和 146,707 个三角面及原色。导出只拼接两端原生 XYZ/颜色、偏移面索引，不焊接、不滤点、不拟合地图。顶点槽计数不等于独立有效几何面积或地图精度；原生 ROSbin、receipt 与非原子来源限制均保留。

第四轮仅 map observer 退出，遥测 observer 保持运行。按 `disconnected_wall` 到 `restarted_wall` 计算，实际断开 23.6117 秒，显式等待目标为 22 秒，其余是检查开销。该窗口发生在运动已完成之后：两端阶段均为 20→20，原生 TSDF 完成数增加 42/43，peer 继续 ready 且无 fatal。它证明该时段地图观察进程退出不阻断局部积分，不能冒称第四轮飞行中或全部结果接收断开测试。

完整运行资料在本机以下路径；原始图像、IMU、边端图和地图没有因此开启落盘：

- [任务完成摘要](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/completed_missions.json)、[三主机关闭回执](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/closure_receipts.json)。
- [联合轨迹质量](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/trajectory_quality.json)、[重载遥测统计](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/telemetry_metrics_60s_heavy.json)、[hub 传输统计](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/transport_metrics.json)。
- [map observer 退出审计](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/map_observer_independence.json)、[第四轮完整原色 PLY](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/final_native_map/global_native_mesh.ply)、[导出来源与 hash 清单](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_final_20261104_150046/hub/final_native_map/export_manifest.json)。

## 第四轮第一次冷启动：起飞前拦截

`distributed_final_20261104_145734` 的 master 启动只等待 Popen 返回，没有等待 XML-RPC bound；快速并发 roslaunch 后多个进程尝试自启 master，产生 `/run_id` UUID 冲突。该轮在传感器、运动前即失败拦截并关闭，不是另一次飞行失败，也未混入最终第四轮指标。manager 增加本轮所属 master PID、`getPid` 与 UUID 就绪屏障后，使用新 RID `distributed_final_20261104_150046` 冷启、预热和完整运动成功。

## 第三轮：运动与遥测改善，mesh 实时性未通过

`distributed_fast_20261104_142541` 完成两圈 20/20 航段，约 503.6 秒，全部关闭。map 与 telemetry 已采用独立 observer 进程。

| 本轮观测 | AMOV | UROS |
| --- | ---: | ---: |
| actual 源龄 P95（60 秒窗口） | 50.55 ms | 15.78 ms |
| 最大 callback 接收间隔 | 168.31 ms | 74.78 ms |
| 联合 body path ATE（一次团队 SE(3)） | 0.20715 m | 0.18920 m |
| 运动窗源时间配对支持率 | 99.969% | 99.903% |
| 独立局部 VIO 基线 ATE | 0.34169 m | 0.37079 m |

双 actual 最近源时间差小于 100 ms 的比例为 eligible sampler ticks 的 99.30%；eligible 覆盖全部 ticks 的 99.9667%。source age 包含时钟偏差和处理/排队，仍不是单向网络延迟。联合轨迹只用一个 SE(3)，不能把每机独立拟合基线与联合结果混淆；覆盖及本轮仿真条件限制仍保留。

AMOV 局部 raw/PGMO mesh 仍积压几十秒，所以不能以遥测变快或运动完成认定地图实时通过。观察层曾每秒重复序列化未变 shared Marker、逐 snapshot 生成 PLY，并重复构造 native/accepted 镜像面。当前改为变化驱动发布、默认原生 ROSbin/metadata 归档和有界 geometry 复用，保持全部原色/面与严格 receipt/source 校验。

上述优化与两端统一 `--voxel-size .20` 已在第四轮使用；第四轮结果见前节。第三轮的数十秒积压仍保留为该次配置的历史结果，不能因后续运行完成而删除。

## 第二轮：运动完成与 observer 独立性，实时性能未通过

`distributed_atomic_20261104_140544` 双端预热检查通过，完成两圈 20/20 航段，运动约 498.6 秒。旧 combined observer 从断开到重启实际为 24.1505 秒（显式等待目标 22 秒，另含检查开销），该窗口发生在飞行期间：两端本地任务从 stage 18 推进到 20，AMOV/UROS 原生 TSDF 完成计数增长 47/49 次。该证据支持飞行独立于观察端，不能把 observer 退出混为两机 peer 失联，或把此窗口归到第四轮。

60 秒 actual 接收测量虽平均约 30 Hz，但源龄 P95 约 2.45/2.56 秒，最大 callback 接收间隔 0.84/0.80 秒，双 actual 新鲜配对 sampler 覆盖仅约 66.6%。source age 包含时钟偏差、上游处理和排队，不是纯单向网络时延；平均频率不能掩盖这些长间隔。AMOV 存在处理积压，因此本轮实时性能不验收，也不据运动完成直接宣称 SLAM 精度通过。

同一 observer 进程中的 mesh/Marker/PLY 构造与序列化可能通过 GIL 拖慢遥测；该机制是当前修复依据，完整耗时占比仍需实测。该问题在第三轮通过独立 map/telemetry 进程复验，遥测指标见上节；第三轮地图处理仍有单独瓶颈。map status 对独立 telemetry 只报告 `status_available_here=false`，不能把它解释为连接断开。未来全结果接收退出测试须停 `observer` 与 `observer-telemetry` 两个 role；第二轮旧 combined 只需停止一个。

## 首次分布式运动：失败、未验收

运行 `distributed_native_20261104_132619` 于 2026 年 11 月 4 日开始运动，T0 为 `1793770672.9387455`。两机完成 stage 0–8，共 9/20 航段；stage 9 已发目标但未完成，未完成计划的两圈。

| 最后任务摘要 | AMOV | UROS |
| --- | ---: | ---: |
| 实际运动累计路径 | 77.848239 m | 76.079982 m |
| 最小已观测机间分离 | 2.213925 m | 2.226412 m |
| 最后摘要 wall | 1793770908.159760 | 1793770908.002777 |
| 已完成航段 | 9 | 9 |

UROS `dense.log` 在 `1793770828.020320` 报告 `Remote native control authentication queue overflow`；原生 dense bridge 退出触发 required 进程组关闭，PGMO 同组退出。EGO 和独立局部 TSDF 此后继续运行，不表示 PGMO 已恢复。根进程随后受控停止两端任务、planner/traj_server 并保持仿真体，`Peer mission aborted` 是此次停测的连带结果，不是原始故障。

首次故障来自原始控制消息与独立来源回执的异步匹配队列。第二轮已改为 metadata 与原始 payload 绑定的本机伴随流，保留原始 source/type/MD5/hash/序号核验与明确 proxy caller；原子消息封装不等于原生优化器拥有共享原子轮次。修复和单元测试通过不构成完整运动复验通过。

该轮还有本机质量数据关联缺项：只保留 latest body path 时，较新的 path 可能覆盖迟到 accepted mesh 所指向的旧版本，导致联合评价缺少一端。observer 已增加有界内容 hash 缓存，仍仅发布 receipt 指定的原生 path，不能用放松关联或逐机器人对齐补出“完整”结果。

证据保存在各端 `three_host_20261102/distributed/runs/<run>/<host>/` 下的 `mission_summary.json`、`dense.log`、`dense/bridge/bridge_events.jsonl`、`peer/peer_bridge_status.json` 和 `tsdf_witness/integration_events.jsonl`。全程累计积分包含静止预热，飞行次数必须按运动时间窗口过滤，不能直接引用累计值。停止当次可能先发布 aborted 而尚未到摘要文件的定时写入点，因此旧 `flying` JSON 不证明进程仍在飞；还需核对最后状态、受控停止回执和 OS 进程。

只读预热短采样中 AMOV/UROS CPU 约 62.5%/22.3%，可用内存约 11.736/10.844 GB，采样间隔无 swap-in/out 增量，未发现 raw/图/mesh/CSV 意外落盘。它们是该时刻的资源观察，不是全程飞行百分位，也没有支持将故障归因为 UROS 内存耗尽。

## 当前范围与性能限制

- 真实跨机约束、原生优化完成观测、原生路径使用及 mesh receipt 分别核验；相同 gauge 标签或同名 frame 不能代替共享初始化证据。
- continuity 修复不能把旧 verified path 作为新提交路径的使用证明；第五轮已报告完整运动 accepted 更新间隔，AMOV 最大 22.8619 秒仍构成时效限制，不能只以最终快照判性能通过。
- 原生 DPGO/PGMO 消息仍没有共享原子 optimizer/mesh generation ID。当前在线绑定明确是观测性、非原子证据，不能写为严格同步提交。
- PGMO 使用软约束。原生 keys/stamps/有限位姿检查与位姿残差统计不等于逐 pose 精确数值 ACK，非零历史顶点位移也不是每轮运行的强制条件。
- peer TCP 重连与进程重启不同。当前同进程 RAM 队列支持重连；boot 变化、过期关键数据或可靠队列溢出要求新 run，未实现无盘进程重启续图。
- observer 断开与 peer 失联不同。第二轮飞行中旧 combined 退出、第四轮运动后 map-only 退出及第五轮运动后新 map 进程缓存恢复各有证据；本轮未把两进程同时退出或 peer 网络分区当作已测。
- 无语义标签推理，未验证真实飞控、硬件时间同步、多机真实标定或 SLAM 对 EGO 的导航反馈闭环。
- 联合 ATE/RPE 必须一次共同 SE(3) 对齐，并报告样本与时间覆盖。目标到达误差、回到起点距离、传输 hash 一致均不是 SLAM 精度指标。

## 2026-11-03 集中 baseline：一次运动验收

集中版本完成两圈 20 航段，498.922 秒；实际路径 160.288/159.045 米，真实 TSDF 积分 2408/2414 次，最长完成间隔 1.259/0.579 秒，最终地图覆盖末圈。[集中验收摘要](evidence/motion_acceptance_20261103.json)

该结果只对应当次本机集中计算仿真，不证明当前边缘分布式计算、任意网络条件或严格原子轮次已经验收。其闭合位置误差也不应冒充整条轨迹的 ATE/RPE。

## 2026-11-02 旧双机版本：历史故障

旧版本两圈飞行持续 500.4466 秒，AMOV/UROS 路径约 160.21/158.46 米，但飞行期间实际 TSDF 积分只有 47/84 次。AMOV 飞行末尾约 390 秒无新积分；最终两机接受请求序号 40/25，权重摘要不同，全局同轮校验通过次数为 0。[历史并发分析](evidence/concurrency_analysis.json)

当时桥接层的图/网格、序列化、RGB-D 配对和 IMU/里程计存在共享调度与锁，观察到约 56 秒旧时间戳输出、队列丢弃和长间隔。该版本的串行积压和空全局显示不能直接当作当前 online/source 状态；当前采用独立局部回投进程和在线 PGMO，需要以新运行重新验证。

历史资源分析显示：AMOV Orin NX 当轮仅 4 核在线、15W，CPU 中位数约 93%；UROS i7-7820HQ 有 8 逻辑 CPU、整机约 59%，但旧 dense bridge 近单核满载。两机仍有约 8–10 GB 可用内存。历史数据不支持“都是 UROS 内存不够”，也不能推导升级硬件即可修复协议问题。[历史资源定义与统计](evidence/performance_analysis.json)

## 后续复验仍须保留的检查

1. 冷启动拓扑、service、MD5、真实来源、native key 0 与局部 gauge 正确；错误 epoch/boot/source 不能进入估计。
2. 静止与全程飞行均有新的原生 TSDF 完成，检查源龄、最大间隔、队列丢弃和资源；不能只看 mesh 定时器。
3. 两端 LCD/DPGO/PGMO 均持续，来源认证队列不积压；原生跨机约束、完成轨迹、路径使用与 mesh 关联有证据。
4. 完成计划两圈 20 航段；observer 断连/恢复不要求中心重启或重新派发目标，不把 peer 断连混为同一测试。
5. 本机完整原色三角面、全量/delta 基线恢复与 last-known/stale 可追溯；边端只有允许的日志/配置/state 写入。
6. 对完整飞行窗口报告一次共同 SE(3) 的联合轨迹质量、每机指标与缺失覆盖；不足时明确不通过，不用隐藏配准补齐。
