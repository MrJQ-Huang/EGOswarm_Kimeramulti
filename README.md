# EGO Swarm 与 Kimera Multi 双机协同建图实验

本项目在两台局域网主机上各运行一架虚拟无人机，用 EGO-Swarm 执行局部规划与队友避碰，同时运行视觉惯性定位、跨机器人视觉回环、分布式轨迹优化及原生稠密网格后端。

这是正在排查问题的实验代码快照。最近一次双机闭环飞行完成，但持续局部建图和同轮全局地图发布未通过验收，详见[已知问题](docs/known_issues.md)。

## 当前实现范围

| 部分 | 当前实现 |
| --- | --- |
| 双机运动 | 每台主机运行 EGO-Swarm、连续运动仿真体和传感器；网络共享未来轨迹 |
| 规划输入 | 仿真里程计与障碍观测；尚未将 Kimera 位姿和地图反馈为规划输入 |
| Kimera 前端 | 原生视觉 RGB-D 与 IMU 输入、VIO 关键帧和视觉回环 |
| 协同轨迹 | Kimera-Multi LCD 与 DPGO 链路，交换描述信息、约束和优化消息 |
| 局部稠密几何 | 原生 Voxblox TSDF、原生 Voxblox mesh frontend |
| 轨迹修正网格 | 固定版本 Kimera-PGMO 的现代 MESH 接口兼容移植 |
| LiDAR | 仿真生成并录制，当前不替代视觉地点识别或作为该 Kimera 前端 |
| 展示 | 各机局部原生网格顶点、同轮校验后的全局原生顶点及运动轨迹 |

此实现使用 Kimera 系列的原生计算组件和项目自有的接口桥接、事务校验及显示节点，并非 Kimera-Multi v1 官方依赖组合的原封部署。当前配置输出几何地图，未接入语义标签推理。

两机分别计算规划和 SLAM，但实验使用共享 ROS1 master、场景发布和任务调度节点；目前没有验证通信基础设施无中心或 master 失联后的恢复能力。

## 仓库结构

```text
experiment/
  configs/{amov,uros}/multimodal/   # 两机各自的 VIO 参数与 launch 配置
  harness/                         # 双机仿真、场景、任务与传感器
    multimodal/                    # VIO 接口、协同后端、稠密桥接与 RViz 配置
  multimodal/                      # 原部署的 VIO 参数和 launch 输入
  native_full_stack_20261102/       # 进程管理、构建记录及环境入口
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

主要进程入口是 [manage_native_stack.py](experiment/native_full_stack_20261102/manage_native_stack.py)。它保留现有双机部署的绝对路径、IP 和容器名，需要先配置；本快照不提供全新机器一键复现保证。

纯 Python 契约与 IMU 测试可在仓库根目录运行 `python3 -m unittest discover -s experiment/harness/multimodal -p 'test_*.py'`。该组测试通过不代表 ROS 原生节点或长时间双机融合通过。

## 最新实验状态

2026 年 11 月 2 日，双机完成两圈约 500.45 秒的闭环任务，实际运动路径约为 AMOV 160.21 米、UROS 158.46 米。飞行期间原生 TSDF 实际积分次数分别为 47 和 84，AMOV 飞行末尾约 390 秒没有新积分；重复发布旧网格不能算作建图更新。

该轮没有观察到通过同轮校验的全局地图。两机最后接受的优化请求序号为 40 和 25，权重摘要也不同，全局展示按校验规则保持空白。以上结果不能作为成功复现实时双机全局融合的依据。[实验摘要](docs/evidence/concurrency_analysis.json)

## 后续工作

- 拆分局部 RGB-D、IMU 与耗时图和网格处理的调度，建立消息年龄与队列上界。
- 查明 TSDF 变换查询、队列阻塞和缓存过期的具体失效路径。
- 为两机优化结果与网格实现共同轮次提交，避免各机长期只接受不同版本。
- 修复后先分别通过单机持续建图，再验证双机同轮变形和全局发布。

## 上游工作

- [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
- [Kimera-Multi](https://github.com/MIT-SPARK/Kimera-Multi)
- [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)
- [Kimera-PGMO](https://github.com/MIT-SPARK/Kimera-PGMO)
- [Voxblox](https://github.com/ethz-asl/voxblox)

第三方项目沿用各自许可证。引用和再分发时应同时核对上游许可证及本仓库保存的版本与补丁信息。
