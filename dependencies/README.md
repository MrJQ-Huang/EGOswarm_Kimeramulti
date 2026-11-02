# 固定版本源码与兼容补丁

`source_manifest.json` 记录 38 个源码获取项，包括 EGO ROS1、Kimera underlay、dense overlay 和独立 OpenGV 防护库。相同上游在不同工作区出现时分别锁定。完整源码由上游固定提交获取，不把编译目录或大型归档放入本仓库。

在仓库根目录运行 `python3 tools/fetch_sources.py --host uros --list` 或 `--host amov --list` 可预览；使用 `--destination` 获取源码。脚本只向新的目录写入，逐项 `git apply --check` 后应用对应主机的补丁，不编译或启动服务。构建顺序及系统库仍须按部署说明处理。

`patches/` 保存已部署兼容修改。AMOV 和 UROS 使用同一份 EGO ROS1 补丁，但 VIO 与部分 OpenCV 构建适配有差异。AMOV VIO 为锁定提交原版；UROS VIO 有稀疏特征防护分支。被否决的 nearest-depth 候选没有纳入。

`system_wrappers/` 是 UROS 既有构建上下文中的小型 catkin 系统库适配包。它们不替代相应原生计算库。

`locks/` 保留原始源码锁定证据。其中个别字段描述生成清单时的状态，例如 OpenGV 源锁的 `runtime_enabled: false`；本轮实际启用状态由环境入口的 `NATIVE_USE_OPENGV_GUARD=1` 决定。防护库及 `runtime_gate.sha256` 必须在目标架构重新构建、验证和生成。

`licenses/` 保留实际找到的上游许可证原文；来源、提交、哈希及未找到独立许可证文件的记录见 `license_origins.json`。每个上游的其余版权声明随固定版本源码一并取得。本仓库未将所有第三方内容重新授权为同一种许可证。

本快照通过源码、契约和清单检查，尚未在全新系统逐项完成 38 个获取项的完整构建。当前 Windows 环境的单项直接 Git 拉取测试因无法连接 GitHub 443 端口而未完成；代码上传通过已连接的 GitHub API 完成。这不代表运行时双机局域网故障。
