# 地面压力测试：delivery-route 负载下的 CPU 与 FAST-LIO 延迟

目的：在不起飞的情况下复现 10-27 22:55 炸机前的状态，确认 FAST-LIO 是否因 CPU 不够而落后。

当时记录（`data/mission/20261027_225452_9781`）：下视红点识别进程已在运行，EGO 启动后约 2 s，`/Odometry` 时间戳开始落后于真实时间，14 s 内从 0.5 s 涨到 6 s；`/Odom_high_freq` 在 22:55:53.8 停止输出，定位 unhealthy，PX4 退出 OFFBOARD 进入 ALTCTL。EGO 从请求到启动用了 4.86 s（平时 0.1～0.3 s）。只跑 EGO 的 `waypoint-route` 没出现过这种落后。机载为 Intel N250（4 核），飞行时还开着 NoMachine 远程桌面。

本目录工具针对机载工作空间 `/home/drone24/aquacore-nanjing`，路径按 `tools/stress/` 相对定位。

## 安全

- **拆掉全部桨叶**。全程不运行任何会解锁、起飞或投放的入口：`takeoff.sh`、`fly_points.sh`、`drop_points.sh`、`drop_test.sh`、`4_delivery_route.sh`、`hover_drop.sh`、`magnet.sh`。
- 遥控器保持上锁，急停开关打到停止位。
- `stress_monitor.py` 只订阅，不发布话题、不调用服务。`load_red_point.sh` 只做识别。`load_ego.sh` 不发目标；它的 `position_cmd` 发在会话话题上，平时由任务管理器转发，px4ctrl 只听 `/position_cmd`，所以不会传到 PX4。
- 监视器若显示 `!!! 飞控已解锁`，立即用遥控器上锁并停止测试。

## 准备

1. **确认下视相机**。`flight/red_point.yaml` 的 `camera` 段写着设备路径、分辨率、格式和内参文件，`load_red_point.sh` 按它启动相机；设备不存在时直接退出，B、D、E 阶段测不到识别负载。10-28 起换装 `32e6:9005`（YUYV 640×480），配置已改好，识别进程按比赛流程自己启动相机，不再需要 `load_camera.sh`。`load_camera.sh` 只在配置与相机不符、需要临时手动出图时使用。
2. 用飞行电池供电，接法和飞行时相同；N250 的频率和温度受供电影响。
3. 把飞机放在比赛场地或特征相近的地方，雷达朝向有结构的场景，下视相机能看到地面。
4. 把本目录复制到机载（在笔记本上执行）：

```bash
scp -r tools/stress drone24@172.18.9.122:/home/drone24/aquacore-nanjing/tools/
```

## 终端

全部通过 SSH 打开（不用 NoMachine，E 阶段再开）。都在 `cd /home/drone24/aquacore-nanjing` 下执行。

| 终端 | 命令 | 说明 |
|---|---|---|
| T1 | `sh flight/1_sensors.sh` | MAVROS、MID360、FAST-LIO |
| T2 | `sh flight/2_localization.sh` | 局部坐标与定位接口 |
| T3 | `sh flight/3_controller.sh` | 原生 PX4 指令接口；不自动起飞 |
| T4 | `sh tools/stress/monitor.sh` | 监视器；在这里输入阶段名 |
| T5 | 按阶段运行 `load_red_point.sh` | 红点识别负载 |
| T6 | 按阶段运行 `load_ego.sh` | EGO 负载 |

T1～T3 启动后等定位稳定（`sh flight/position.sh --once` 有输出），再启动 T4。T4 默认阶段名为 `A_baseline`。

## 阶段

每阶段约 3 分钟：前 90 s 静止，后 90 s 一人端着飞机沿航线慢走（桨叶已拆），让 FAST-LIO 处理移动时的点云。切换阶段时在 T4 输入阶段名回车，再启停负载。

| 阶段 | T4 输入 | 负载 | 对应 |
|---|---|---|---|
| A | （启动即是）`A_baseline` | 只有 T1～T3 | 基线 |
| B | `B_red_point` | T5 `sh tools/stress/load_red_point.sh` | 只开识别 |
| C | `C_ego` | T5 Ctrl-C；T6 `sh tools/stress/load_ego.sh` | 只开 EGO，同 waypoint-route |
| D | `D_red_then_ego` | T6 Ctrl-C；T5 启动识别，30 s 后 T6 启动 EGO | **复现炸机组合** |
| D2 | `D2_ego_restart` | 识别保持运行；T6 每 20 s Ctrl-C 后重启，共 3 次 | EGO 启动瞬间的冲击；10-25 17:23 那次在第 4 次重启后落后 2.3 s |
| E | `E_plus_desktop` | 保持 D 的负载；连上 NoMachine，在机载打开 QGC | 现场操作习惯 |

**E 阶段会打断链路**（2026-10-28 实测）：机载 QGC 默认自动连接 USB，打开约 8 s 后占用 PX4 串口 `/dev/ttyACM0`。MAVROS 异常退出（exit -6），它是 required 节点，roslaunch 随即关闭整个 T1，雷达驱动和 FAST-LIO 一起停止。之后监视器显示 `lidHz 0`、`unhlth` 持续增长。E 阶段做完必须重启 T1～T3。不要在机载打开 QGC，地面站请在笔记本上通过 UDP 连接。

结束：T4 输入 `q` 回车（或 Ctrl-C），然后依次停止 T6、T5、T3、T2、T1。

可选 F 阶段（会改系统设置，需要你决定是否做）：保持 D 的负载，把 CPU 调速从 `powersave` 改为 `performance`，看延迟是否消失。重启后自动恢复，也可手动改回：

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

```bash
echo powersave | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## 读结果

监视器每 2 s 打印一行，退出时打印每阶段汇总，并写入 `data/stress/<时间>/`：

| 文件 | 内容 |
|---|---|
| `samples.csv` | 每 0.5 s：CPU 总量与最忙核、频率、温度、节流计数、各话题频率与时间戳年龄、预测校正延迟、定位健康、飞控模式 |
| `groups.csv` | 每 0.5 s 各进程组 CPU（单核百分比，可超过 100） |
| `events.csv` | 阶段切换、定位健康变化、延迟越线、预测被拒、解锁与模式变化 |
| `summary.json` | 每阶段汇总，与终端表格相同 |

汇总列：

| 列 | 含义 | 正常 | 10-27 炸机时 |
|---|---|---|---|
| `odomAge` | `/Odometry` 时间戳落后真实时间的最大值，即 FAST-LIO 校正延迟 | < 0.2 s | 0.5 → 6 s |
| `corrLag` | `/prediction/status` 最后一次校正离现在多久 | < 1.0 s（预测上限） | 6 s |
| `wthld` | 本阶段预测被拒次数 | 0 | 2684 |
| `unhlth` | `localization_healthy` 为 false 的秒数；px4ctrl 依据它 | 0 | 持续到断电 |
| `lidHz` | `/livox/lidar` 最低频率 | 稳定约 10 | — |
| `cpu%`、`cpu95` | 4 核总 CPU 平均与 95 分位 | | 当时未记录 |

标记只用于报告，不会停止任何进程：`LIO_LAG`（odomAge > 0.5 s）、`PRED_HORIZON`（corrLag > 1 s）、`WITHHELD`、`LOC_UNHEALTHY`、`LIDAR_DROP`（雷达频率低于本阶段中位数一半）、`THROTTLE`（CPU 热节流计数增加）、`ARMED!`。

区分原因：

- `odomAge` 上升而 `lidHz` 稳定：雷达数据正常到达，是 FAST-LIO 算不过来，即 CPU 不够。
- `odomAge` 上升且 `lidHz` 掉到 0：雷达或网线断了，查硬件。
- `THROTTLE` 或频率明显下降：散热或供电问题。
- 汇总最后一列列出该阶段最耗 CPU 的进程组，看是谁占用了核。

结论判断：

- A、B、C 无标记，D 或 D2 出现 `LIO_LAG`/`WITHHELD`：确认识别和 EGO 叠加超出 N250 能力。之后如何减负（关识别、换取图方式、调整 EGO 地图参数、换机载）需要另行讨论，本测试不改任何飞行配置。
- 只有 E 出现：远程桌面和 QGC 是主要负担，飞行时不要开。
- 全部阶段都正常：地面负载低于飞行（无振动、点云变化小、EGO 不规划轨迹），不能排除 CPU 原因。监视器只订阅，下次试飞可以在 T4 同时运行，直接记录飞行中的数据。

## 无终端运行

只有一个 SSH 会话、没有 tmux 时，用 `run_bg.sh` 在后台启动各终端。它启动的每个进程都带 `STRESS_RUN=<目录>` 环境变量，`stop` 只停这些进程（发 SIGINT，与 Ctrl-C 相同）。监视器从 `<目录>/phases.txt` 读取阶段名：

```bash
bash tools/stress/run_bg.sh start data/stress/run1 t1_sensors sh flight/1_sensors.sh
```

```bash
bash tools/stress/run_bg.sh start data/stress/run1 monitor sh -c "tail -n0 -F data/stress/run1/phases.txt | exec sh tools/stress/monitor.sh --out data/stress/run1/monitor"
```

```bash
bash tools/stress/run_bg.sh phase data/stress/run1 B_red_point
```

```bash
bash tools/stress/run_bg.sh stop data/stress/run1
```

每个进程的输出在 `<目录>/<名字>.log`。阶段名写 `q` 时监视器结束并打印汇总。

## 其他用法

```bash
sh tools/stress/monitor.sh --summarize data/stress/20261029_150000
```

```bash
sh tools/stress/monitor.sh --no-ros --duration 30 --phase cpu_only
```

结果拷回笔记本：

```bash
scp -r drone24@172.18.9.122:/home/drone24/aquacore-nanjing/data/stress ./stress_results
```

离线单元测试：

```bash
cd tools/stress && python3 -m unittest test_stress_core
```

## 局限

- 静止地面上点云变化小，FAST-LIO 负载低于飞行。端着走一段可以部分弥补。
- EGO 不发目标，不运行轨迹优化，只有建图负载，测得的是下限。
- 监视器用 Python 订阅 `/livox/lidar` 原始字节，自身占一点 CPU，在 `groups.csv` 中单列为 `stress_monitor`。
- 雷达时间戳若与主机时钟有固定偏差，`odomAge` 会整体偏移；看各阶段之间的变化，而不只看绝对值。10-27 的记录中偏差约 0.02 s。
