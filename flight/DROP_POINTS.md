# 录点后测试一至四次投放

两条命令：`sh flight/record_drop_points.sh` 录点，`sh flight/drop_points.sh` 执行。也可用 `record_points.sh` 录点，两种录点入口和两种执行入口共用同一份 XY 文件。

每点流程：前往录点 → 下降至定位 Z=0.5 米 → 红点开环对准 → 下降 → 单次释放并等匹配成功回执 → 原有 1 秒动作等待 → 回升。依次完成所有选定点后，返回本次实际起飞 XY 并降落，等待落地上锁。只起飞一次，不执行比赛满载绕圈，也不回投放点 1 闭环。

## 录点

机载目录 `/home/drone24/aquacore-nanjing`，先运行日常传感器与定位终端，保持同一次定位直到飞行结束。在目标位置读取无人机当前 XY：

```sh
sh flight/record_drop_points.sh
```

- 走到第一个投放位置，按空格录点 1；继续走到下一位置按空格。
- 录 1、2、3 或 4 点均可；X 撤销最后一点，Q 保存退出。
- 四个点对应四个现有机构档位，录满后继续按空格不新增第五个；仍可 X 撤销。
- 不录 HOME，不采用手持时高度；飞行自动记录实际起飞位置用于返航。
- 默认保存 `data/routes/waypoints.yaml`，与普通航点共用；比赛九步航线仍单独保存。
- 再次运行默认重新录制；`--resume` 继续录点，`--output 路径` 另存文件。

录点只订阅 `/airseafly/native/planning_odometry`，不启动控制器、相机、任务或舵机。不增加录点停稳、样本限龄或飞行准入条件。

## 执行

将飞机放回起飞位置，机构装载并处于待释放状态。前三基础终端保持运行；任务不自动复位舵机。

| 点 / 投放编号 | 物理输出 | 档位 | PWM |
|---|---|---|---|
| 1 | MAIN6 | 第一档 | 1190 μs |
| 2 | MAIN7 | 第一档 | 1190 μs |
| 3 | MAIN6 | 第二档 | 1350 μs |
| 4 | MAIN7 | 第二档 | 1350 μs |

以下示例临时使用起飞／巡航／回升 1.0 m，投放 0.7 m；高度相对本次起飞基准。先预览：

```sh
sh flight/drop_points.sh --count 1 --takeoff-height 1.0 --drop-height 0.7 --dry-run
```

去掉 `--dry-run` 执行：

```sh
sh flight/drop_points.sh --count 1 --takeoff-height 1.0 --drop-height 0.7
```

`--count 2` 投前两个点，`--count 3` 投前三个，`--count 4` 投四个；省略 `--count` 投全部已录点（最多四个现有档位）。普通录点超过四个时可用 `--count` 选择前 1～4 点。不能选比文件实际记录更多的点。选择前 N 点不改变录点文件，也不改变其余任务的参数。

预览及执行都会显示本次使用的绝对文件路径、SHA256 和实际 XY，任务配置日志保存相同来源。默认不再读取旧 `drop_points.yaml`；需要旧文件时明确指定 `--route data/routes/drop_points.yaml`，旧格式仍兼容。点间默认用固定轨迹航段（与比赛共用 `mission_profiles.delivery-route.navigation.backend`）：起飞后从起飞点正上方出发，每段到投放点或返航点停下，参考到终点后按 `arrival_radius_m` / `arrival_dwell_s` 判断到点再对准；投放后的下一段从回升终点出发。不运行 EGO、不实时绕障。设 `backend: ego` 恢复原 EGO 导航。

默认仍读取 `flight/flight.yaml` 的比赛参数，没有另存一套飞行参数。2026-10-26 已确认比赛地图上限改为 2.4 m，覆盖巡航 1.4 m 和门段 1.6 m；上述示例显式指定的 1.0 m 会覆盖巡航高度。机载当前投放高度为 0.25 m，以该机 `--dry-run` 输出为准。`--route 路径` 可使用另一份投放录点文件；原 `drop_test.sh --x ... --y ... --slot ...` 仍可指定单口测试。

不需要另运行起飞或红点识别命令。执行任务自动启动独立红点进程，固定航段模式不启动 EGO；`backend: ego` 时保持同一个 EGO 进程贯穿所有点、投放及返航。

沿用已确认的 [比赛投放规则](DELIVERY_ROUTE.md)：每次取图 15 秒，参考运动结束后宽限 15 秒，每点最多额外重拍一次，重拍后 XY 容差各放宽 8 cm、Z 不变；识别超时用最后视觉目标，没有则用录点；下降超时按实际位置和高度释放。每点单独记录匹配成功回执，同类别不跳过；舵机回执不代表已核实物件落地。

`Ctrl+C` 或 `sh flight/hold.sh` 停止本次任务并保持，保留成功记录；新任务清空记录。需要当前位置降落时运行 `sh flight/safe_land.sh`。录点的 `--resume` 仅继续编辑文件，不是恢复飞行任务。

## 离线验证

`sh flight/record_drop_points.sh --demo --stdin --output /tmp/drop_points.yaml` 可用 `xy 1 2`、`space`、`x`、`q` 输入演示录点，不连接 ROS。`drop_points.sh --dry-run` 只预览。

回放验证覆盖 1～4 点顺序、真实起飞基准返航、同类别逐点计数、旧回执隔离、无识别兜底、hold 记录保留，以及管理器与独立视觉、持续 EGO 的接线；固定航段模式另有逐段回放：投放顺序、返回修正后的起飞 XY、全程不启动 EGO。离线结果不替代现场飞行和实际落物验证。

中间红点识别高度由 `visual_alignment.intermediate_height_m` 配置，默认定位 Z=0.5 米，与原有相对起飞基准的投放高度分别配置；中间下降超时保持实测 XYZ 继续识别。
