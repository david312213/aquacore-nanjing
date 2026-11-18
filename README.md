# AquaCore 南京无人机工作空间

保留 MID360 → FAST-LIO → 固定局部坐标转换 → EGO / 原生参考 → MAVROS → PX4 链路。已将南京录点任务与旧比赛搜索、类别识别、YOLO/KCF、H 降落、双目和神经网络过门流程拆开；原工程及清理前源码存于工作空间外的备份。

机载目录：`/home/drone24/aquacore-nanjing`。桌面同名链接只打开目录。

日常命令见 [flight 操作说明](flight/README.md)，飞行调参只编辑 [flight/flight.yaml](flight/flight.yaml)，红点识别与投影参数只编辑 [flight/red_point.yaml](flight/red_point.yaml)，预览和比赛视觉进程共用。机器安装标定和 PX4 参数分别保存在 [config](config/README.md)，本轮不改变。

- [基础起飞与普通录点试飞](flight/BASIC_FLIGHT.md)
- [连续固定轨迹、两圈预览与 EGO 回退](flight/FIXED_ROUTE.md)
- [九步录点和两圈比赛航线](flight/DELIVERY_ROUTE.md)
- [单点对准与指定口投放测试](flight/DROP_TEST.md)
- [录点后测试一至四次投放](flight/DROP_POINTS.md)
- [红点识别与坐标打印](flight/RED_POINT.md)
- [架构与状态机](docs/架构与状态机.md)
- [构建与离线验证](docs/构建与验证.md)
- [工具目录](tools/README.md)
- [版本与来源](docs/版本与来源.md)、[第三方组件](docs/第三方组件.md)

投放机构已完成，软件接入到点下降、一次红点开环对准、下降投放、15 秒识别／运动兜底与 MAIN6→7→6→7 顺序。软件回归与实飞验收分别记录。2026-10-26 已确认比赛地图上限改为 2.4 m，覆盖巡航 1.4 m、门段 1.6 m；起飞到导航等待阶段保持前段目标高度。定位停更原因、实际起飞悬停、EGO 跟踪和红点坐标误差仍需进一步验证。
