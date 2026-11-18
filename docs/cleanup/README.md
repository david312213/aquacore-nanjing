# 南京版清理与验证记录

本地与机载清理、同步、两任务编译和离线验证均已完成。机载工作空间为 `/home/drone24/aquacore-nanjing`，桌面链接保持不变。软件验证通过；实飞稳定性和投放精度仍需现场验收。

## 实际变更

- `flight` 从 51 个文件收敛为 20 个，ROS 包从 19 个减至 15 个。源码及资源约从 144.3 MiB 减至 5.1 MiB，减少约 139 MiB；不含 Git、构建目录、运行环境、航点、标定采集和日志。
- [移出旧路径完整清单](removed-files.json) 共 502 项，包含删除和迁移后的旧路径。旧 YOLO 模型、双目驱动、旧搜索三投、类别识别、KCF/H 降落、深度／神经网络过门、旧仿真和专属测试已从工作源码移出。
- 新建公共 `flight_core.py` 和 `native_segment.py`；普通航点与南京两圈任务直接复用起飞、导航、保持、坐标降落和原生参考代码，不再继承旧视觉任务引擎。任务管理器只保留南京任务和 EGO 工作进程。
- `flight/basic_entry.py`、`flight/entry.py` 分别迁至 `scripts/basic_flight_entry.py`、`scripts/flight_entry.py`；地面执行器工具迁至 `tools/payload/`。DF200 几何、四元数函数和点云读取迁入保留模块。
- 九步录点、两圈四投、闭环引用、普通试飞、红点自动高度及 XY 打印保留。四路投放只做模拟回执验证，没有操作机构。
- 飞行调参统一为 [flight/flight.yaml](../../flight/flight.yaml)；红点参数仍在 [flight/red_point.yaml](../../flight/red_point.yaml)。前三终端继续使用短命令。

## 软件验证

- 机载 355 项 Python 测试全部通过，无跳过；11 个离线 C++ 测试程序通过，另外 3 个已编译的 core CTest 目标通过。ROS 启动文件解析、规划参数测试和 15 个包的目录解析全部通过。
- 20 次入口预览与搬迁预览通过，涵盖含空格和中文的工作目录；15 个包、12 个内部相对链接及实际启动链已检查。
- [逐帧行为对照](behavior-comparison.json)：起飞 42、普通航点 63、配送 591，共 696 个周期，位置、速度、航向、状态、切段和动作输出与清理前完全一致。
- [生效参数对照](parameter-comparison.json)：基础和比赛两组各 37 项原有参数一致；地图总尺寸继续为原 EGO 的 100 × 50 × 3 m。
- [验证状态与标定校验值](verification.json)：定位外参、FAST-LIO、PX4 配置、MID360 网络配置、DF200 内参与下视启动配置均与清理前一致。没有新增任务准入、停稳、限龄或超时返航规则。
- 本地日志：`data/deployments/20261024-211713-cleanup/`；各测试详细日志在 `data/checks/`。最终机载构建日志：`/home/drone24/aquacore-nanjing/data/deployments/20261024-211713-cleanup/build-retry-2.log`；完整回归汇总为同目录的 `verification.log`（汇总全部测试及文档图片补齐后的复查，原始输出保留在 `offline-tests.log`）。本地副本在 `data/deployments/20261024-211713-cleanup/onboard-logs/`，逐项测试日志在 `onboard-checks/`。

## 备份与机载状态

本地备份：`/Users/david/Documents/ChatGPT/aquacore-nanjing-backups/20261024-211713-cleanup/`。

机载备份：`/home/drone24/aquacore-nanjing-backups/20261024-211713-cleanup/`。

`source.tar.gz` 和 `source-manifest.json` 保存清理前全部源码（包括未提交、未跟踪文件、配置及资源）；`recent-red-point.tar.gz` 补充保存清理过程中发现的六份最新红点文件，避免用旧备份覆盖新修改。最终本地版本另存为 `clean-final-source.tar.gz`，并附逐文件及压缩包 SHA256 清单。原始前后参数、控制周期对照和测试日志也在备份目录。

机载已完成最终源码的 SHA256、执行权限和相对链接核对，15 个 ROS 包全部解析到新工作空间，没有混入旧工程环境。旧 `build_ws` 已移到机载备份的 `old-build_ws/`；使用干净 Noetic 环境和 `-j2` 完成重建。桌面链接仍指向 `/home/drone24/aquacore-nanjing`。航点、标定采集数据和历史日志保留，旧工程 `/home/drone24/uav-aquacore` 未修改。

全量构建暴露出的旧测试问题已修复：core 测试不再引用不存在的 `upstream` 目录，模拟 ROS／投放诊断接口与当前控制器一致；移除不存在的 `commandStep` 测试引用，实际轨迹回调时序仍由 9 项测试覆盖。B-spline 测试按现有逐轴可行性检查断言，并覆盖超限情况，没有修改规划器限速算法或飞行控制数学。雷达／FAST-LIO／PX4 目录中 385 个源码文件与清理前逐文件校验一致。机载缺失的标定报告 `summary.png` 已补上传，并核对 SHA256；原标定参数保持不变。

过程中没有启动传感器、相机、控制器、飞行任务或执行机构，也没有写入 PX4 参数。

## 实飞待办

日常操作及调参见 [flight/README.md](../../flight/README.md)。实际起飞悬停、EGO 单点／多点跟踪、雷达在线状态和红点米制误差需要现场另行验收。南京比赛沿用巡航 1.4 m、门段 1.6 m、规划上限 1.1 m 的原值，其冲突仍待处理，没有在清理时隐式修改。基础试飞继续使用高度 1.0 m、速度 0.5 m/s、加速度 0.25 m/s²、规划上限 1.4 m。
