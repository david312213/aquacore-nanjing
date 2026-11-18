# 录点单投放（投放后悬停）

先录一个投放点，再起飞直接前往，对准投放后回升并**原地悬停**，不返航、不降落。投放流程与[单点投放测试](DROP_TEST.md)、比赛完全相同，只有结束方式不同。

保持 `1_sensors.sh`、`2_localization.sh`、`3_controller.sh` 三个终端运行；录点和飞行使用同一次定位，中途不要重启定位。

## 1. 录点

```sh
sh flight/record_single_drop.sh
```

拿着飞机走到投放靶标正上方，按**空格**记录 X/Y；位置不满意，走到新位置再按空格即**覆盖**旧点。`x` 撤销，`q` 保存退出。保存到 `data/routes/single_drop_point.yaml`，单独一份，不覆盖普通航点 `waypoints.yaml` 和比赛航线。只读定位，不控制飞机或舵机。

## 2. 预览与执行

```sh
# 预览：显示投放点、投放口、MAIN 输出与 PWM，不连接飞机
sh flight/single_drop.sh --slot 1 --dry-run

# 执行
sh flight/single_drop.sh --slot 1
```

流程：**起飞 → 飞向录好的点 → 下降至识别高度 → 红点对准 → 下降投放 → 等真实成功回执及原有 1 秒动作等待 → 回升到巡航高度 → 在该位置持续悬停**。

| `--slot` | 舵机与档位 |
|---|---|
| 1（默认） | MAIN6 第一档 |
| 2 | MAIN7 第一档 |
| 3 | MAIN6 第二档 |
| 4 | MAIN7 第二档 |

实际 PWM 以 `flight.yaml` 的 `delivery-route.payload` 为准，预览会打印。只发一次释放指令，不自动复位、不重发。

- 悬停点：回升终点，即投放时的实际 XY、本次起飞基准上方巡航高度。任务显示“投放完成，在回升位置持续悬停”，终端随即退出，飞机继续悬停。
- 结束悬停：`sh flight/safe_land.sh` 在当前位置降落；`sh flight/hold.sh` 停止任务、原地保持。
- 其他投放口、高度、对准、超时兜底参数全部沿用 `delivery-route`，与 `drop_test.sh` 相同；`--point 文件` 可指定别的投放点文件，`--takeoff-height`、`--drop-height`、`--set` 等用法同 `drop_test.sh`。
- 投放点文件只能有 1 个点；用别的录点脚本录了多个点时会提示重新录。

## 投放运动速度与加速度（2026-11-19）

均在 `flight.yaml` → `mission_profiles.delivery-route`，比赛、`single_drop.sh`、`drop_test.sh`、`drop_points.sh` 共用，下次任务生效：

| 参数 | 管什么 | 临时覆盖 |
|---|---|---|
| `flight.drop_speed_mps` | 投放下降速度（含降到识别高度） | `--drop-speed` |
| `flight.recover_speed_mps` | 投放后回升速度 | `--recover-speed` |
| `motion.native_acceleration_mps2` | 对准移动、投放口补偿、下降的加速度 | `--align-acceleration` |
| `motion.recover_acceleration_mps2` | **只管投放后回升**的加速度；`null` 沿用上一行 | `--recover-acceleration` |
| `common.flight.native_takeoff_speed_mps` / `native_takeoff_acceleration_mps2` | 起飞速度 / 加速度 | `--takeoff-speed` / `--takeoff-acceleration` |

参考运动先加速后减速：距离短而加速度小时到不了设定速度，最高约 √(加速度×距离)。例如回升约 1.2 m、加速度 0.5 时最高约 0.77 m/s；要真正以 2 m/s 回升，回升加速度需约 3.4 m/s² 以上。预览（`--dry-run`）会打印“投放运动”一行，显示实际生效的速度与加速度。
