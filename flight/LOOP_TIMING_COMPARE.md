普通航点单圈配时对比（2026-11-17）

两个入口共用当前普通航线参数和同一份普通航点文件。都使用固定轨迹，只飞一圈并回首点悬停，不开启视觉、不投放、不自动降落。原入口 fly_points.sh 默认行为保持不变；比赛投放及降落代码不变。

在 /home/drone24/aquacore-nanjing 中运行。前三终端按原方式保持打开，第 4 终端录点：

```sh
sh flight/record_points.sh
```

按 1 选普通巡航点，到每个位置按空格，x 撤销，q 保存。只记 XY；不用 h 或 b/d。默认保存 data/routes/waypoints.yaml，会覆盖该普通航点文件，不覆盖比赛的 delivery_route.yaml。录点和两次测试保持同一次定位。程序自动闭环回首点，不需要重复记录首点。固定轨迹不实时绕障。

先预览两个配时，核对文件 SHA256、点数、高度、速度和加速度一致：

```sh
sh flight/test_loop_original.sh --dry-run
sh flight/test_loop_optimized.sh --dry-run
```

下面两个是分别执行的实飞命令；每次任务都会起飞，绕一圈后悬停。完成一趟并落地后，再由操作者执行另一趟：

```sh
sh flight/test_loop_original.sh
```

```sh
sh flight/test_loop_optimized.sh
```

沿用原来的当前位置降落入口：sh flight/safe_land.sh。两个测试都不会自动启动下一趟。自定义航线时，两边都加相同的 --route 文件名；不自动寻找“最新文件”，也不转换或删选现有航点。

original 使用原来的普通航线配时。optimized 对同一条空间曲线调用 2026-11-15 已部署的局部配时算法，保留相同速度、加速度和已有过门点限速。规划在起飞准备阶段完成，飞行中沿用已部署的时间映射采样器。若没有收益或优化器不可用，沿用原参考并显示“局部配时未采用”；不能把这次结果算作采用优化后的成绩。

两条脚本都只在本次命令里指定 fixed_min_snap、navigation.laps=1 和配时选择，不修改 flight.yaml。普通路线 navigation.laps 与比赛 delivery_route_options.laps 是两项不同设置，比赛圈数不变。

预览的导航参考时间包含起飞后的接入段和一圈闭环，不含起飞、后续悬停或降落。默认预览假设起飞 XY 位于首点，实际任务会使用实测起飞原点重新生成；对比时采用相同路线、参数和起飞位置。实际通过耗时和跟踪误差要结合实飞日志评估，参考变快不代表飞机一定按同等比例变快。

日志保留本次导航配时选择、是否采用优化、原/新参考时间及实际位置误差。原配时与优化配时的轨迹事件均按各自时间映射推进，首点保持和人工 hold 仍走原流程。

## 配时版本选择（2026-11-18）

`flight/flight.yaml` 的 `mission_profiles.waypoint-route.navigation.timing` 决定 `fly_points.sh` 用哪个版本：

| 值 | 含义 |
|---|---|
| `original` | 原配时 |
| `optimized` | 局部配时，只限速度/加速度（2026-11-15 版） |
| `thrust` | 局部配时，推力方向约束（2026-11-18 版） |

`thrust` 的参数在同一 profile 的 `planning.retiming`：前馈倾角上限、倾转角速度上限、风阻系数、总推力上限。它把前馈倾角限制在 30° 以内，给 PX4 的 45° 上限留出反馈余量，并把倾转角速度限制在 120 °/s 以内。

- 临时切换：`sh flight/fly_points.sh --timing original|optimized|thrust`，不写则按 yaml。
- `test_loop_original.sh` 和 `test_loop_optimized.sh` 仍固定使用各自的版本。
- 预览第一行显示“配时版本”；选 `thrust` 时会多一行“推力方向约束”，列出前馈倾角和倾转角速度的峰值。
