# EGO Swarm 与 Kimera Multi 双机协同建图实验

本项目在 AMOV、UROS 上各运行一架虚拟无人机，由设备本地 EGO-Swarm 执行轨迹规划、避障和控制；原始传感器经无损上行送到本机，集中运行两套视觉惯性定位、跨机器人回环、轨迹优化和原生稠密建图。设备不录制原始数据，本机负责显示和后处理，默认保存地图结果与少量遥测；原始输入重放和完整调试录制按需开启。

2026年11月3日，本机集中版本完成了一次两圈、20航段的双机运动验收，用时498.922秒，无安全中止；两路真实TSDF持续积分，并导出了覆盖末圈的202,567顶点、258,118三角面联合地图。控制证明、传输与资源指标见[验收摘要](docs/evidence/motion_acceptance_20261103.json)。当前协同绑定仍使用原有native观测匹配，没有新增共享原子轮次协议；历史问题和运行边界见[已知问题](docs/known_issues.md)。

## 当前实现范围

| 部分 | 当前实现 |
| --- | --- |
| 双机运动 | 每台主机从相同本地场景运行EGO、连续仿真体及传感器；EGO互发未来轨迹，实际状态单份30 Hz回传本机 |
| 规划输入 | 仿真里程计与障碍观测；尚未将 Kimera 位姿和地图反馈为规划输入 |
| Kimera 前端 | 两套原生 VIO 在本机消费无损分发的 RGB-D 与 IMU，设备只生成和发送观测 |
| 协同轨迹 | 本机两份 Kimera-Multi LCD 与 DPGO 链路，交换描述信息、约束和优化消息 |
| 局部稠密几何 | 本机运行两套原生 Voxblox TSDF、原生 mesh frontend，默认目标输入 5 Hz、网格周期 1 秒 |
| 轨迹修正网格 | 固定版本 Kimera-PGMO 的现代 MESH 接口兼容移植 |
| LiDAR | 默认仅上传1 Hz、每4个实测点取1点；当前Kimera RGB-D链不依赖此流，EGO本地障碍观测保持原密度 |
| 展示 | 本机 NVIDIA OpenGL、默认 30 FPS 绘制上限和 20 Hz 机体位置显示；地图版本校验保留 |

此实现使用 Kimera 系列的原生计算组件和项目自有的接口桥接、事务校验及显示节点，并非 Kimera-Multi v1 官方依赖组合的原封部署。当前配置输出几何地图，未接入语义标签推理。

两机分别执行 EGO 规划与控制，SLAM 集中在本机。任务脚本只发目标 PoseStamped，B-spline 与 PositionCommand 由各设备的 EGO Planner 和 traj_server 产生。目前没有验证共享 master 失联后的恢复能力，集中部署也不作为多机分担 SLAM 计算的证明。

## 数据优先级与保存

机体遥测直接取自设备本地连续仿真体，独立于图像渲染；本机任务、轨迹和场景显示统一消费入口分发的 `/sim_N/ground_truth/odometry`，该实际机体状态不接入Kimera估计。遥测与IMU使用独立优先标记通道，可见光与必要深度继续精确配对，LiDAR作为可稀疏的辅助流。默认相机5 Hz、遥测30 Hz、IMU100 Hz、LiDAR1 Hz/点步长4；无线接入点可能不执行优先标记，实际时效仍须验收。

`manager_hub.py record --record-profile map`默认只保存地图输出、少量轨迹与诊断，不反复录制TSDF输入云、各层完整网格和显示Marker/完整Path。`replay`增加RGB-D/IMU原始重放输入，`debug`保留全部中间诊断。在线VIO/TSDF所需输入不因停止录制而停止传输。

最终地图使用 `experiment/harness/multimodal/export_final_global_map.py` 从通过校验的native pair导出PLY/PCD和原生三角网格，并保留两份native mesh、回执、标定与来源。显示云的青/橙颜色仅表示机器人来源，不能代替原生网格的相机颜色。已有实验录包不自动删除。

## 仓库结构

```text
experiment/
  configs/{amov,uros}/multimodal/   # 两机各自的 VIO 参数与 launch 配置
  harness/                         # 双机仿真、场景、任务与传感器
    multimodal/                    # VIO 接口、协同后端、稠密桥接与 RViz 配置
  multimodal/                      # 原部署的 VIO 参数和 launch 输入
  native_full_stack_20261102/       # 进程管理、构建记录及环境入口
  three_host_20261102/              # 本机集中计算、设备控制/上行、拓扑与冷启动校验
dependencies/                      # 上游版本锁定与本地补丁
docs/
  architecture.md
  deployment.md
  known_issues.md
  evidence/                        # 公开的本轮实验和资源统计摘要
```

不上传 rosbag、点云实验数据、编译产物、容器镜像或访问凭据。上游依赖保留来源、固定提交和补丁，需在目标架构上独立编译；AMOV 与 UROS 的二进制不能互换。

## 使用前阅读

1. 阅读[架构与数据流](docs/architecture.md)，确认估计、规划和展示的边界。
2. 按[部署说明](docs/deployment.md)检查工作区、路径、网络与 ABI，再启动节点。
3. 用[已知问题](docs/known_issues.md)中的验收条件判断结果；窗口持续刷新不代表地图持续积分。

当前入口是本机 [manager_hub.py](experiment/three_host_20261102/manager_hub.py) 和设备 [manager_local.py](experiment/three_host_20261102/manager_local.py)，均支持不启动进程的 `--dry-run`。原 `manage_native_stack.py` 保留为历史部署入口，不属于当前集中工作流。路径、IP、容器和依赖仍需按[部署说明](docs/deployment.md)准备。

纯 Python 契约与 IMU 测试可在仓库根目录运行 `python3 -m unittest discover -s experiment/harness/multimodal -p 'test_*.py'`。该组测试通过不代表 ROS 原生节点或长时间双机融合通过。

## 历史实验状态

2026年11月3日集中版本的AMOV/UROS实际路径为160.288/159.045米，末圈闭合误差2.28/0.30厘米。飞行窗口机体遥测约29.71/29.29Hz，源龄P95约29.36/12.21毫秒；真实TSDF积分2408/2414次，最长完成间隔1.259/0.579秒。最终native pair和两份原网格已验真，末圈地图覆盖已确认。该结果证明这一次仿真运行，不代表真机、所有网络条件或严格原子优化协议均已验收。

2026 年 11 月 2 日，双机完成两圈约 500.45 秒的闭环任务，实际运动路径约为 AMOV 160.21 米、UROS 158.46 米。飞行期间原生 TSDF 实际积分次数分别为 47 和 84，AMOV 飞行末尾约 390 秒没有新积分；重复发布旧网格不能算作建图更新。

该轮没有观察到通过同轮校验的全局地图。两机最后接受的优化请求序号为 40 和 25，权重摘要也不同，全局展示按校验规则保持空白。以上结果不能作为成功复现实时双机全局融合的依据。[实验摘要](docs/evidence/concurrency_analysis.json)

## 后续工作

- 在本次运动验收基础上增加重复轮次、长时运行和受限网络条件验证。
- 量化原生VIO轨迹与联合地图的几何精度，保留输入年龄和真实积分监测。
- 为两机优化结果与网格实现共同轮次提交，避免各机长期只接受不同版本。
- 验证真实传感器噪声、网络失联恢复及共享master故障处理。

## 上游工作

- [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
- [Kimera-Multi](https://github.com/MIT-SPARK/Kimera-Multi)
- [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)
- [Kimera-PGMO](https://github.com/MIT-SPARK/Kimera-PGMO)
- [Voxblox](https://github.com/ethz-asl/voxblox)

第三方项目沿用各自许可证。引用和再分发时应同时核对上游许可证及本仓库保存的版本与补丁信息。
