# 分布式部署与分阶段启动

入口：`experiment/three_host_20261102/distributed/manager.py`。本页针对设备完整本地原生 Kimera、三个独立 master 和本机结果 observer。旧集中管理器可保留作 baseline，但不能在同一轮混启。第五轮 `distributed_continuity_20261104_153318` 已完成 20/20 航段、连续性修复核验、联合原生地图导出及三主机关闭；功能改造完成，AMOV 强实时性能仍未通过。见[分布式核验摘要](evidence/distributed_acceptance_20261104.json)、[完整报告](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_report.json)与[实测限制](known_issues.md)。

## 主机、端口与环境

| 主机 | robot | ROS master | 额外网络服务 |
| --- | --- | --- | --- |
| AMOV `192.168.5.6` | 0 | 本机 `:11431` | peer TCP 11640；结果 map 11740、telemetry 11741 |
| UROS `192.168.5.21` | 1 | 本机 `:11431` | 同上 |
| 本机 `192.168.5.14` | observer | 本机 `:11431` | 主动连接两端结果 server，不中转 peer |

peer 的 control/feature/ego 三条独立全双工连接由 robot 0 直接拨号 robot 1。两端 master/node 注册、参数和本地 service 仍在各自主机内；无需通过 hub 查询或重发布远端 master 状态。`vlc_frame_query` 和 `request_pose_graph` 是本机原生依赖，不能漏启。

先完成[通用依赖与 ABI 部署](deployment.md)。AMOV 使用 ARM64 原生 underlay/overlay；UROS 使用现有 Kimera/EGO ROS1 容器，本机使用 NVIDIA observer 容器。所有交换类型及 MD5 必须一致，包括自定义 DPGO/PublicPoses、BoW/VLC、PoseGraph、KimeraPgmoMesh 和 Bspline。当前通信层是自有兼容 TCP peer bridge，不是原作者 UDP RTM；参考固定上游话题清单不等于原样复现其通信栈。

同步的是源码、配置和对应架构构建结果，不复制另一主机二进制，不覆盖历史 run。VIO 实际读取 `configs/<host>/multimodal/`；manager 记录该 run 的参数、flags 和原生包路径。同一 run 中改变配置、reset 仿真或重启 peer 会破坏状态连续性，应关闭后用新 run/cutoff。

## 一次生成共同身份

在协调终端生成一次，再把三项值原样带到三个主机终端。不要每机各自调用时间函数。

```bash
RUN_ID="distributed_$(date -u +%Y%m%d_%H%M%S)"
CUTOFF="$(python3 -c 'import time; print(time.time())')"
GAUGE_ID="${RUN_ID}.robot0-initial"
printf '%s\n' "$RUN_ID" "$CUTOFF" "$GAUGE_ID"
```

cutoff 是本轮最早可接受的源时间，不是预设的起飞时刻；`--start-at` 单独控制任务放行。相同 gauge 字符串仅标识配置，不证明两机已经共享坐标。

以下示例在各主机本地执行。按主机选择 `HOST` 和根目录，保持共同 `RUN_ID/CUTOFF/GAUGE_ID`：

```bash
# AMOV；UROS 改为 HOST=uros、其实际宿主机实验根目录
HOST=amov
EXP_ROOT=/home/amov/Drone/two_pc_ego_20261030
# 本机：HOST=hub；EXP_ROOT=/home/mrjq311/swarm_kimera_runtime/experiment
MGR="$EXP_ROOT/three_host_20261102/distributed/manager.py"
COMMON=(--host "$HOST" --run-id "$RUN_ID" --cutoff "$CUTOFF" --gauge-id "$GAUGE_ID")
python3 "$MGR" master "${COMMON[@]}" --dry-run
```

已核验本架构 OpenGV 防护库后，可向设备的原生角色命令增加 `--opengv-guard`；缺少或未核验该库时不要照抄。以下每一步都有就绪检查，不将整段命令一次性无条件跑完。

## 1. 独立 master 与本地仿真体

三台主机各启动本机 master：

```bash
python3 "$MGR" master "${COMMON[@]}"
```

等待 manager 的本轮所属 master PID、XML-RPC `getPid` 和 `/run_id` UUID 就绪屏障成功返回，才启动后续 roslaunch。Popen 创建成功不等于 master 已 bound。第四轮第一次 preflight 因此竞态触发 UUID 冲突，在传感器/运动前拦截；修复后使用新 run 冷启成功。不要忽略冲突或复用失败 run。

AMOV/UROS 分别启动 supervisor，执行一次 reset，等待 `reset_delivered`、本地 continuous_body 和 EGO 唯一发布者就绪：

```bash
python3 "$MGR" supervisor "${COMMON[@]}"
python3 "$MGR" reset "${COMMON[@]}"
python3 "$MGR" status "${COMMON[@]}"
```

reset 是有限的一次本地动作，不是持续后台角色；不要在 SLAM 已保留地图后重置机体。任务目标仅由本机的 local_mission 发布，实际规划与控制链仍是 EGO → Bspline → traj_server → continuous_body。

## 2. 原生消费者、peer 与结果 server

两端先启动 key 0 的消费者、local frontend、协同后端及 online/no-persist PGMO，再启动 VIO。传感器此时保持未启动：

```bash
for role in adapter local-frontend backend dense peer results witness resource; do
  python3 "$MGR" "$role" "${COMMON[@]}"
done
python3 "$MGR" vio "${COMMON[@]}"
```

使用字节块差分时，在启动 `results` 的那一条命令增加 `--delta`，不要重复启动同一 role。默认完整快照也可运行。当前默认相机 5 Hz、TSDF 目标输入 2 Hz、mesh 周期 2 秒、map 上限 0.5 Hz、actual/VIO 结果遥测上限 30 Hz；分别用 `--camera-rate`、`--cloud-rate`、`--mesh-period`、`--map-rate`、`--telemetry-rate` 配置对应角色。`local-frontend --voxel-size` 配置两端原生 TSDF 分辨率，默认 0.15 m；第四、五轮显式使用两端相同的 0.20 m，改变本地重建分辨率，不能视作 observer 抽稀或 ICP。重现第五轮时仅在该角色启动命令增加 `--voxel-size .20`，并记录配置。IMU 为本地 100 Hz，EGO 障碍观测不因辅助 LiDAR 的上传稀疏化改变。

检查两端 peer `session_ready=true`、`fatal=null`、schema/MD5/双方 boot 正确及原生订阅者就绪。确认 source caller 与本机 proxy 分开，metadata+原始 payload 的伴随流可被原生 dense 认证。原始消息与独立 receipt 的旧双话题匹配路径曾导致队列溢出，不能继续使用旧端与新端混搭。

## 3. 启动本机 observer 与评价采集

本机执行：

```bash
python3 "$MGR" observer "${COMMON[@]}"
python3 "$MGR" observer-telemetry "${COMMON[@]}"
python3 "$MGR" quality "${COMMON[@]}"
python3 "$MGR" resource "${COMMON[@]}"
python3 "$MGR" gpu "${COMMON[@]}"
for role in rviz-local0 rviz-local1 rviz-actual rviz; do
  python3 "$MGR" "$role" "${COMMON[@]}"
done
```

两个独立 observer 进程分别主动连接设备：`observer` role 以 `--channels map`、节点 `/distributed_result_observer` 处理 11740；`observer-telemetry` role 以 `--channels telemetry`、节点 `/distributed_telemetry_observer` 处理 11741。遥测进程不处理 mesh/Marker/PLY 或覆盖地图状态，避免同进程 GIL 竞争。手动 `--channels all` 仅保留旧 combined 诊断兼容；两个独立进程使用同一本机 master，由各自 manager role 精确关闭。map status 标注 telemetry 由外部进程处理，并不表示其断连。各 native 类型先校验 MD5，完整 ROS 字节发布到 `/observer/sim_N/{local_mesh,mesh,accepted_mesh,status,actual,vio,optimized_path,body_path,verified_body_path,optimized_graph}`。quality 只订阅 actual、VIO 和经过 mesh receipt 关联的 `/observer/sim_N/shared_body_path`，不录 RGB-D/IMU。

四个 RViz 的 fixed frame 分别为 `sim_0/vio_odom`、`sim_1/vio_odom`、实际机体 `world`、共享 `kimera_world`。局部地图各自独立；共享证据未就绪时显示等待状态，不能给两个局部图各自拟合一个变换再叠上去。默认显示完整 local/shared 三角面和相机原色，不是以青/橙来源色代替原生颜色。未变 shared Marker 与 body path 不按 1 Hz 重发巨大 payload，只有版本/action/boot/gauge 或关联内容 hash 变化时才发布；freshness/status 仍持续更新。observer CLI `--all-mesh-faces` 可恢复未默认订阅的 native/accepted 镜像 faces。30 FPS 是绘制上限，不是相机、积分或 mesh 更新频率。

## 4. 静止传感器与原生预热

原生服务与 key 0 消费者均就绪后，才在两端执行：

```bash
python3 "$MGR" sensors "${COMMON[@]}" --camera-rate 5
```

验证本地 IMU/RGB-D 有效、原生 VIO key 0、camera/body graph 约定与局部 gauge 正确、真实 TSDF 已开始积分。读取每端 `tsdf_witness/integration_latest.json` 的实际 completion 与年龄；mesh 定时发布次数不能替代积分。

运行 `distributed/audit.py --robot-id N --run-id "$RUN_ID" --cutoff "$CUTOFF" --duration 20 --output ...` 时须使用对应 `env_edge.bash`；UROS 在 native 容器内运行。它检查本机图和短时间输入/状态，保存的是紧凑审计 JSON，不是原始数据。audit/静止预热通过并不意味着完整运动通过。

## 5. 同时放行本地任务

完成预热后，在协调终端生成一次稍后的 UTC epoch，例如当前时间后 10 秒，将完全相同值带到两端：

```bash
START_AT="$(python3 -c 'import time; print(time.time()+10)')"
# 在 AMOV 与 UROS 分别执行；START_AT 必须相同
python3 "$MGR" mission "${COMMON[@]}" --laps 2 --start-at "$START_AT"
```

local_mission 在本地检查 actual/VIO 新鲜、sensor ready、peer/任务阶段状态再放行，目标通过本机 EGO 执行。任务阶段保护与 peer 失联保护独立于 observer；本机质量或显示进程不能下发轨迹替代 EGO。

持续记录任务阶段/实际路径、EGO 本地/peer B-spline、唯一 PositionCommand 链、两端 VIO/LCD/DPGO/PGMO、原生 TSDF 完成、最大源龄/队列、CPU/RSS、换页及允许的文件写入。任何 dense/PGMO fatal 都应判为该链失败，即使 EGO 和 TSDF 继续工作。根协调器负责受控停测；只读观察程序不自行重置或重启。

运动期间可在本机运行 60 秒紧凑遥测测量，不存原始 ROS 流或位姿，也不向 SLAM 发布：

```bash
docker exec kimera-central-hub-gpu-20261103 \
  env NATIVE_ROS_MASTER_URI=http://192.168.5.14:11431 \
  bash /experiment/three_host_20261102/env_hub.bash \
  python3 /experiment/three_host_20261102/distributed/telemetry_metrics.py \
  --run-id "$RUN_ID" --duration 60 \
  --output "/experiment/three_host_20261102/distributed/runs/$RUN_ID/hub/telemetry_metrics_60s.json"
```

工具默认校验 `/distributed_telemetry_observer` caller；测旧 combined 模式时显式增加 `--expected-caller /distributed_result_observer`。输出包含 actual/VIO 接收频率、callback 计数、最大间隔、首尾静默和 source age；source age 包含两机时钟偏差、上游处理/排队及传输，**不是单向网络时延**，负值保留。双 actual 源时间差按默认 50 Hz 独立 sampler 取各端最新已收时间戳，报告严格小于 50/100 ms 的比例；分母为两端接收均新鲜的 sampler ticks，并单列 missing/stale 覆盖，不能当作同步采集帧或逐 callback 配对。分位数超过固定标量缓存后使用明确标注的均匀 reservoir，原始消息不会被保存。

## 6. observer 退出、重连与结果保存

只停止 `observer` 会停地图接收，独立 actual/VIO 仍可运行。全结果接收断连测试需停止两个本机 role；均不停止 peer 或两端任务：

```bash
python3 "$MGR" stop "${COMMON[@]}" --roles observer observer-telemetry
python3 "$MGR" observer "${COMMON[@]}"
python3 "$MGR" observer-telemetry "${COMMON[@]}"
```

第二轮旧 combined 实际断开 24.1505 秒，只涉及一个 observer role，发生在飞行期间：stage 18→20，TSDF 完成数 +47/+49。第四轮 map-only 实际断开 23.6117 秒，telemetry 保持运行，且发生在运动完成之后：stage 20→20，TSDF +42/+43。两次显式等待目标均为 22 秒，实际断开时长另含检查开销。不能混用两次时段、进程范围，或将第四轮写为飞行中全观察端退出测试。观察进程退出期间无新健康状态，冻结的 RViz 标签不能证明数据仍新鲜。

第五轮另在运动完成后以全新 RAM 缓存重启 map observer，稍后复核 shared map 与两端精确 hash 关联的 verified path 成功，telemetry observer 保持运行。`fresh_observer_restart.json` 记录的是重启及后续核验时刻，未连续测量首次恢复时延；两时刻之差不能当作实际恢复耗时，也不证明瞬时恢复。该核验不等于飞行中全观察端退出或 peer 失联。
故障注入需记录实际断连窗口并核验两端阶段进展、积分与 peer 状态。不能将 observer 退出等同于机器人间网络分区。结果断连期间边端只替换有界 latest，不等待本机磁盘；恢复首包全量，delta 缺 base 补全量。中间 mesh 版本可能被有意覆盖，“无损”指每个已发送快照字节保真，并非保证每次原生发布都被归档。

本机保存到 `<hub-root>/three_host_20261102/distributed/runs/<run>/hub/observer/<run>/sim_N/<result-boot>/`。每个 map stream 目录默认保留 `<sequence>_<snapshot>.rosbin`、同名 metadata 和 `latest.json`；遥测以可解析 frames 追加。默认不逐 snapshot 生成 PLY，降低地图接收/显示阻塞。最终离线导出原始 ROS mesh，或显式使用 observer CLI `--snapshot-ply` 逐份导出时，仍保留完整三角面、双精度 XYZ、原始 RGBA `native_*` 字段及便于常见查看器读取的派生 8 位颜色；原生元数据始终完整保存在 ROSbin。

共享地图必须匹配真实 native receipt。`shared_body_path` 按 receipt 的路径 hash 从当前 run/boot/gauge 的独立 verified stream 或有界历史缓存选择；不能只取 latest，也不能放松 hash 来补缺质量数据。第五轮的 latched `/sim_N/native_dense/verified_body_path` 保留最后已核验的原生 path，较新 `approved_body_path` pending 时不会覆盖它；receipt 明确区分 last-verified 与 pending，不能声称新提交已被 PGMO 使用。导出兼容旧档案的 `body_path` 目录，仍须精确内容/hash及来源匹配。应用 snapshot ID、持续共享 gauge 与伴随消息的原子封装均不构成原生 optimizer/mesh 共享原子轮次。

## 7. 安全关闭与离线评价

先在两端停止 mission，等待本地 planner/traj_server 停止及 continuous_body hold，再停传感器、新输入、后端和结果服务；最后核对进程归属并关闭 master。本机先保留完整结果与质量采集，再停查看器/observer/采集进程。

```bash
# 两端先执行，确认实际机体保持；不要将此步骤当作只停一个目标发布者
python3 "$MGR" stop "${COMMON[@]}" --roles mission
# 其余指定角色收尾后，可关闭该 run 所有剩余角色，包括本地 master
python3 "$MGR" stop "${COMMON[@]}"
python3 "$MGR" status "${COMMON[@]}"
```

distributed manager 的 `stop` **默认包含 master**，没有旧管理器的 `--stop-master`。它只选择本轮、该 role、该 master 和本 profile 的 OS 进程；残留进程须报告，不能用全局 `pkill` 或隐藏重启冒充正常闭合。边端关闭后只保留文本日志/状态/配置，不导出 raw bag、图、VIO CSV、PLY 或 TSDF；SLAM 内存历史随进程退出丢失，重启用新 run。

在本机对 `quality_poses.jsonl` 运行离线评价，按真实源时间限定飞行窗口：

```bash
python3 experiment/three_host_20261102/distributed/quality.py \
  /path/to/hub/quality_poses.jsonl --output /path/to/hub/quality_metrics.json \
  --start-stamp-ns FLIGHT_START_NS --end-stamp-ns FLIGHT_END_NS
```

联合 body path 只使用两机共同的一个 SE(3) 对齐，不估尺度，不逐机器人拟合联合结果；该评价变换不返回显示或估计。报告 ATE、约 1 秒 RPE、每机/团队统计、配对与未配对样本及时间覆盖；缺机器人时明确 `missing_robot`。VIO 可以按各自局部 gauge 单独评价，但必须标为基线，不能用该操作掩盖联合坐标错误。

第五轮完成两圈 20/20、496.58 秒运动与三主机关闭，closure 均为 true、`remaining=[]`，RViz 已关闭。联合输出保留 109,117 个原生顶点槽，其中 67,033 个被面引用，以及 146,846 个三角面和原色，见[完整 PLY](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_native_map/global_native_mesh.ply)与[来源清单](/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/final_native_map/export_manifest.json)。本机完整结果目录为 `/home/mrjq311/swarm_kimera_runtime/experiment/three_host_20261102/distributed/runs/distributed_continuity_20261104_153318/hub/`。第四轮修复前资料继续保留，见[已知问题](known_issues.md)。

最终 Noetic 294 项测试无 skip 通过；174 个托管运行源码/配置文件在本地及三主机逐项 SHA256 一致，架构特定二进制和 underlay 不在此比较范围。GitHub 仍只有 baseline snapshot，分布式 feature 在本地准备提交。软件契约测试、冷启动、运动、观察端恢复与质量分别留证，不能互相代替。

第五轮连续性不可变档案审计全部检查通过，已完成逻辑挡流修复核验；最新 pending path 仍不能当作已被 PGMO 使用，持续 gauge 仍不是精确 optimizer generation 证明。运动窗 TSDF 完成 775/805 次，首次共享接纳后的 accepted mesh 更新间隔中位数为 3.7527/1.9996 秒、P95 为 13.5174/2.9459 秒、最大为 22.8619/4.9907 秒。AMOV 平均整机 CPU 90.75%，因此强实时性能仍未通过。完整运动指标替代截至约 435 秒的中途近似值；一次团队 SE(3) 轨迹、几何质量与资源定义见[已知问题](known_issues.md)及[分布式核验摘要](evidence/distributed_acceptance_20261104.json)。
