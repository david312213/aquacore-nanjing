# 基础起飞与普通录点试飞

使用 [flight.yaml](flight.yaml) 的公共参数：起飞／普通航点相对本次起飞基准 1.0 m，起飞参考及航线速度 0.5 m/s、加速度 0.25 m/s²，规划上限 1.4 m。相机、识别、录像和投放关闭；轨迹日志保留。

先启动 [三个基础终端](README.md)。单独起飞：

```sh
sh flight/takeoff.sh --dry-run
sh flight/takeoff.sh
```

原生起飞参考执行结束后持续保持目标高度。这里不增加停稳等待。继续沿用原控制器的保持、降落与人工接管机制。

普通录点只读取定位，记录任意数量 XY，不记录手持时的高度：

`record_points.sh` 和 `record_drop_points.sh` 默认共用 `data/routes/waypoints.yaml`；`fly_points.sh` 和 `drop_points.sh` 都从该文件读取。预览和执行显示实际来源路径、SHA256 与坐标，显式 `--route` 可选择其他文件。

```sh
sh flight/record_points.sh
# 空格记点，x 撤销末点，q 保存退出
sh flight/fly_points.sh --dry-run
sh flight/fly_points.sh
```

`record_points.sh`（2026-11-19 起）先录**进场点**：从起飞点进入闭环途经的点，引导录 3 个，录满自动进入闭环航点；Enter 可提前进入，不强制个数。进场点保存为 `version: 3` 的 `entry_points`，飞行时只经过一次，之后按 `navigation.laps` 绕闭环；EGO 模式先依次到达进场点，再逐个到达闭环点。旧文件没有进场点，飞法不变。`drop_points.sh` 只使用闭环点，文件中有进场点时打印提示，不投放进场点；投放测试建议用 `record_drop_points.sh` 录点。

`record_points.sh` 保存后（2026-11-18 起）自动按 `flight.yaml` 的 waypoint-route 参数生成固定轨迹预览，放在航点文件旁边：默认 `data/routes/waypoints_preview.html`，终端打印路径，用浏览器打开；内容与 `fly_points.sh --preview-html` 相同（预览假定起飞 XY 为首点、Z=0）。生成失败只打印提示，不影响保存。`record_drop_points.sh` 和边界录制不生成。

普通航线默认连续固定轨迹：起飞 → 首点 → 全路线两圈 → 首点减速保持。普通点与两圈交界不停留，参考按时间连续推进；全部高度使用本次起飞基准，同高段恒高、不同高度段平滑过渡。不会实时绕障，EGO 地图膨胀和虚拟围墙不生效。完整行为与预览见 [固定航线说明](FIXED_ROUTE.md)。

```sh
sh flight/fly_points.sh --preview-html data/previews/fixed-route.html
# 以下仅预览原 EGO 单遍逐点路线：
sh flight/fly_points.sh --set navigation.backend=ego --dry-run
```

EGO 回退模式保持原 XYZ 0.35 m / 0.5 s 到点判据、单个 EGO 工作进程和末点保持。固定模式保留原起飞、控制权、保持、降落及人工接管机制，不增加停稳、准入、超时或重试。

```sh
sh flight/hold.sh
sh flight/safe_land.sh
```

保持会停止后续航点推进。`safe_land.sh` 通过既有 `/px4ctrl/force_land` 请求 PX4 `AUTO.LAND`，以实际模式、落地和上锁状态显示结果，不是空中停桨。人工切换模式后任务不会自动恢复争抢控制。

实飞依次检查：单独起飞悬停、一个航点、多点直线和转弯。用日志分别核对定位、规划参考和 PX4 实际跟踪，不以离线通过代替飞行验收。

日志目录在终端显示，轨迹文件为 `data/mission/<本次目录>/native_trajectory.jsonl`。离线分析：

```sh
python3 src/airseafly_real400_mission/scripts/native_flight_report.py data/mission/<本次目录>
```

报告包含高度超调、悬停偏差、位置／速度跟踪误差和时间统计，不据此自动中断任务。`--json 报告.json` 可另存。录点、试飞、保持和降落支持 `--dry-run`；`--flight-config` 与单次命令参数仍可覆盖默认配置。
