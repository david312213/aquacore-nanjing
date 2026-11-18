# 南京版日常操作

在机载工作空间内运行：

```sh
cd /home/drone24/aquacore-nanjing
```

前三个终端分别运行并保持打开，无需追加配置参数：

```sh
sh flight/1_sensors.sh
sh flight/2_localization.sh
sh flight/3_controller.sh
```

第 1 终端是 MAVROS、MID360、FAST-LIO；第 2 终端建立局部坐标与定位接口；第 3 终端启动原生 PX4 指令接口。前三终端不启动相机、识别或录像，也不自动起飞。

第 1 终端同时启用遥控器 **9 通道**：**中位 MAIN6/7 全闭合（各 900 μs），低位全打开（各 1350 μs），高位不发指令、保持现状**。启动后按首个有效档位执行一次，此后只在换挡时发送；同一档位不持续覆盖自动投放。两路在同一条命令中改变，高位不自动回位；终端显示飞控接受或拒绝回执。配置位于 `flight.yaml` 的 `common.rc_payload`，修改后重启第 1 终端生效。自动任务使用高位即可。

第 1 终端还启动**定位安全门**（2026-10-31 确认）：仅在解锁且 OFFBOARD 时生效，PX4 与 FAST-LIO 的位置差（`localization_status` 的 `px4_error_m`）**大于 0.5 m 并持续 0.5 s**，立即结束当前任务，经 `/px4ctrl/force_land` 请求当前点降落，与 `safe_land.sh` 同一路径；触发后锁定到上锁为止，不自动恢复。控制器未接受时每秒重试，遥控切出 OFFBOARD 后不再请求；7 通道急停照常优先。正常飞行最大差值约 0.37 m。参数在 `flight.yaml` 的 `common.localization_gate`，修改后重启第 1 终端；状态见 `/airseafly/native/localization_gate`。历史记录回放：`python3 tools/px4/replay_localization_gate.py`。

| 命令 | 用途 |
|---|---|
| `sh flight/position.sh` | 实时显示局部 XYZ；Ctrl-C 退出，`--once` 只看一次 |
| `sh flight/takeoff.sh --dry-run` | 查看起飞参数；去掉 `--dry-run` 执行起飞后悬停 |
| `sh flight/record_points.sh` | 普通录点：先录进场点（引导 3 个，Enter 提前结束），再录闭环点；空格记 XY、x 撤销、q 保存 |
| `sh flight/fly_points.sh --dry-run` | 普通固定航线预览；去掉 `--dry-run` 起飞、连续两圈、首点悬停 |
| `sh flight/fly_points.sh --preview-html data/previews/fixed-route.html` | 仅保存完整固定轨迹交互预览，不启动飞行 |
| `sh flight/hold.sh` | 停止任务推进并保持当前位置 |
| `sh flight/safe_land.sh` | 请求当前位置降落，显示实际状态 |
| `sh flight/record_route.sh` | 南京分段录点：进场点（引导 3 个）→ 巡航闭环 → 投放环 → 返航点，Enter 结束各段；h HOME，1/2/3 选类型、空格记点，--resume 续录；旧共用一套点用 --shared |
| `sh flight/4_delivery_route.sh --dry-run` | 查看圈数（`delivery_route_options.laps`，默认 2）、到点下降与一次红点对准、四投及兜底参数；去掉 `--dry-run` 执行 |
| `sh flight/drop_test.sh --x 1 --y 2 --slot 1 --dry-run` | 单点投放预览；去掉 `--dry-run` 起飞、对准、指定口投放、回起飞点降落 |
| `sh flight/record_drop_points.sh` | 按空格记录 1～4 个投放点，X 撤销，Q 保存 |
| `sh flight/record_single_drop.sh` | 单投放录点：站在靶标上方按空格记 1 个点（再按覆盖），Q 保存 |
| `sh flight/single_drop.sh --slot 1 --dry-run` | 单投放预览；去掉 `--dry-run` 起飞、飞向录好的点、对准投放、回升后原地悬停 |
| `sh flight/drop_points.sh --count 1 --takeoff-height 1.0 --drop-height 0.7 --dry-run` | 测试前 1 个录点；count 可选 1～4，省略则全部，完成后回起飞点降落 |
| `sh flight/hover_drop.sh --height 1.0 --dry-run` | 悬停手动投放预览；去掉 `--dry-run` 起飞后输入 1～4 投放，q 当前位置降落 |
| `sh flight/red_point.sh` | 独立 DF200 红点观察，自动打印 XY；q 退出 |
| `sh flight/record_mount.sh --help` | MID360 / PX4 安装标定采集工具 |

统一调参：[flight.yaml](flight.yaml)。基础模式使用 `common`，比赛入口自动叠加 `mission_profiles.delivery-route`。高度 m、速度 m/s、加速度 m/s²；高度相对本次起飞基准。文件修改在对应程序或下次任务启动时读取，运行中不热更新。

临时覆盖示例：

```sh
sh flight/takeoff.sh --height 1.0 --takeoff-speed 0.5 --dry-run
sh flight/fly_points.sh --speed 0.4 --acceleration 0.2 --show-config
sh flight/fly_points.sh --set planning.map.inflation_m=0.25 --dry-run
```

普通航点和一至四点投放共用 `data/routes/waypoints.yaml`，比赛航线保存到 `data/routes/delivery_route.yaml`。录点和飞行应在同一次定位会话内完成，定位重启后坐标原点会改变。

执行时显示实际读取路径、文件 SHA256 和坐标。旧 `drop_points.yaml` 只通过显式 `--route` 使用，操作见 [录点投放测试](DROP_POINTS.md)。

四次投放依次为 MAIN6/1190、MAIN7/1190、MAIN6/1350、MAIN7/1350 μs。两台舵机各投两个，默认关闭 900 μs，上限 2000 μs。兼容入口 `magnet.sh` 的编号 1～4 对应这四次投放；地面测试和关闭复位见 [tools/payload](../tools/payload/)。基础试飞与红点观察不调用投放接口。

比赛任务自动启动独立红点进程，全程保持运行；前 N-1 圈满载，最后一圈投放；每个投放点先下降至识别高度，一次红点开环对准后再下降投放。不需要同时打开预览。识别或相机异常走已配置的 15 秒识别兜底，不停止 EGO。取图、运动兜底和偏移参数见 [比赛航线](DELIVERY_ROUTE.md)。

详细说明：[连续固定航线与回退](FIXED_ROUTE.md)、[基础试飞](BASIC_FLIGHT.md)、[比赛航线](DELIVERY_ROUTE.md)、[单点投放测试](DROP_TEST.md)、[悬停手动投放](HOVER_DROP.md)、[红点观察](RED_POINT.md)。

### 普通航线混合巡航点与过门点（2026-10-30）

普通航线现在默认 `navigation.backend: fixed_min_snap`、连续两圈；下文的地图膨胀、EGO 切点和围墙约束说明只适用于显式选择 `navigation.backend: ego`。固定模式仅使用各点类型的高度，边界作预览对照，不实时绕障；行为与命令见 [固定航线](FIXED_ROUTE.md)。

在 `flight/flight.yaml` 的 `mission_profiles.waypoint-route.waypoint_profiles` 设置两套参数：

```yaml
waypoint_profiles:
  cruise:
    height_m: 1.0
    inflation_m: 0.25
  gate:
    height_m: 1.0
    inflation_m: 0.25
```

两套初值沿用现场普通航线的 1.0 m / 0.25 m，不代表已按门洞尺寸调好。
高度均相对本次实际起飞点，起飞使用 cruise；EGO 模式的规划上下界继续用 `common.planning.map`。
修改配置在下次任务加载，已录航点保存的是类型，不会冻结当时配置数值。

```sh
sh flight/record_points.sh --output data/routes/field_loop.yaml
```

`1` 选择巡航，`2` 选择过门，空格记录 XY 和当前类型；选择保持有效直到再次切换。
状态行显示当前类型的高度和膨胀。`x` 撤销末点，`q` 保存；`--resume` 保留已有点类型。
可以按巡航点、门前点（2）、门中心点（2）、门后点（2）、巡航点（1）依次录制。
录点到执行保持同一次定位，不用手持飞机到飞行高度。旧版纯 XY 文件全部按 cruise 执行。
使用选择键后文件为 version 2，`point_profiles` 与 `points` 一一对应；不能交给未更新的旧程序执行。

```sh
sh flight/fly_points.sh --route data/routes/field_loop.yaml --dry-run
sh flight/fly_points.sh --route data/routes/field_loop.yaml
```

以下为 EGO 回退模式的执行语义：预览逐点显示类型、高度和膨胀。每段从开始前往该点时就使用目标点的类型；在门前留出改变高度的路线，不能把第一次 gate 点仅录在门洞里面。
到门后、切换前往 cruise 点时恢复 cruise 膨胀及高度；高度由 EGO 连续规划，交接保持上一段的参考高度。
切换沿用同一个 EGO 实例，使用已缓存原始障碍点重新膨胀，不重启地图，也不等新点云。
保留现有到点容差和停留判据，不新增准入、停稳、限龄或超时。最后一点保持它自己的高度，不自动降落。

膨胀沿用现有栅格离散化：XY 格数是 `ceil(inflation_m / resolution_m)`；两个数值若落在同一档，实际 XY 膨胀可能相同。
雷达点云路径的 Z 膨胀仍是原有一格；地图上下界、分辨率和速度没有按类型切换。
过门点仍是 EGO 目标，不是强制直线穿门指令；障碍地图和路径规划仍然生效。
`--height` / `--takeoff-height` / `--gate-height` 沿用各自覆盖范围；`--set planning.map.inflation_m=...` 同时覆盖两类膨胀。
投放入口保持原有 XY 与投放高度逻辑，不按此普通航线的 profile 执行投放。

### 场地四角虚拟围墙（2026-10-30）

此约束由 EGO 地图实现；固定轨迹模式只显示边界，不执行围墙避让。

```sh
sh flight/record_boundary.sh
```

拿着未解锁的无人机，沿场地边缘按顺时针或逆时针依次到四个角，各按一次空格，最后按 q 保存。
不要对角交叉录。x 撤销末角。只记录 XY，不要求手持到飞行高度；保存到独立的 `data/routes/field_boundary.yaml`，不覆盖航线文件。
四角不足、重合、共线或交叉会提示无法保存，并保留原有文件。录点不会发飞行或舵机指令。

在 `flight/flight.yaml` 的 common.planning 下配置：

```yaml
virtual_boundary:
  enabled: true
  file: data/routes/field_boundary.yaml
  bottom_m: null
  top_m: null
```

安装默认 enabled=false；录好场地四角后启用。null 表示向下/向上覆盖整个规划地图；数值表示相对本次起飞点的高度。
这四点按实测位置连成凸四边形，允许实测的轻微不方正，不把它们强制拟合成轴对齐矩形。
墙外及靠墙的膨胀带按障碍处理。围墙使用当前航段的膨胀距离（沿用按地图分辨率向上取整），因此巡航/过门切换也会改变墙内侧的避让距离。
例如当前巡航 0.45m 膨胀时，沿墙内侧至少约 0.45m 的区域属于占用区；录航线要留在场内可通行区域。
它约束 EGO 的规划地图，不裁剪原有航点、不增加等待或重试，也不控制手动飞行/原生起降；不能视为飞控级电子围栏。

录四角、录航线、执行应保持同一次定位。重启定位改变原点后应重新录四角和航线。
任务启动时读取文件并冻结坐标和来源 SHA256；修改文件或配置对下一次任务生效。

```sh
sh flight/fly_points.sh --route data/routes/field_loop.yaml --dry-run
sh flight/fly_points.sh --route data/routes/field_loop.yaml
```

预览显示围墙开关、文件、四角和上下界。关闭用 enabled=false。
EGO 启动后 RViz 可添加 Marker 话题 `/drone_0_ego_planner_node/grid_map/virtual_boundary` 查看橙色四边围墙框线；框线为录制边界，避让距离在其内侧。
