# 普通航线连续固定轨迹

普通 `fly_points.sh` 默认使用 GCOPTER MINCO 的七次多项式，整段生成后连续采样；比赛、投放、单独起飞入口保持原实现。

## 构建与预览

本地 macOS 或 Ubuntu 可独立构建数学库，不启动 ROS 或硬件：

```sh
sh scripts/build_fixed_trajectory.sh
sh scripts/test.sh
sh flight/fly_points.sh --route data/routes/field_loop.yaml --dry-run
sh flight/fly_points.sh --route data/routes/field_loop.yaml --preview-html data/previews/fixed-route.html
```

`--preview-html` 只生成预览，不启动管理器或飞行；页面包含 XY 两圈曲线、相对高度、速度和时间滑条。不提供 `--preview-origin X Y Z` 时假设起飞 XY 在首点、起飞 Z=0，并在页面显式说明。实际任务以同次定位的真实起飞原点重新生成，不能将假设起点的预览视为真实接入段。

机载完整编译使用 `sh scripts/build.sh`，数学库随 catkin 一起构建；不存在飞行中在线下载或自动编译。

## 飞行行为

录点仍使用 `record_points.sh`：1/2 选择 cruise/gate、空格记录 XY、x 撤销、q 保存。录点和飞行保持同一次定位会话；本次未加入重定位。

路线：起飞参考终点 → 首点 → 其余点 → 首点（第一圈）→ 其余点 → 首点（第二圈、减速保持）。首次进入首点不算一圈。普通点及两圈交界无到点半径、驻留或停稳等待，按同一条参考的时间轴推进。最终“参考完成”不代表真机零误差到点，误差继续显示。

XY 轨迹经过每个录制点，点间平滑曲线可能偏离录点折线，两圈也可能因起止加减速略有不同。Z 使用同一起飞基准；同高点间恒高、不同高度之间单调平滑变化。全程保持起飞朝向。采用现有航线速度和加速度上限统一缩放时间，不在运行中剪裁指令。

单个目的点退化为平滑到点并保持，完成圈数为零。相邻完全相同的 XYZ 不产生零长度运动；原航点索引保留。首末重复点不会再插入额外零长度闭合段。

**固定模式不运行 EGO，不实时绕障。障碍膨胀、地图上下界及 EGO 虚拟围墙不约束此轨迹。** 已录边界只在预览中作为对照和诊断，不新增拒飞、等待或重试规则。查看完整曲线，不只查看航点位置。

```sh
# 在机载完成部署、构建后，由操作者明确启动；本次本地开发没有执行以下飞行命令。
sh flight/fly_points.sh --route data/routes/field_loop.yaml
sh flight/hold.sh
sh flight/safe_land.sh
```

hold 使用现有机制停止任务，无继续命令。人工接管、真实控制故障、降落及新任务撤销旧轨迹，不会重新开始计时或恢复旧任务。

## 设置和回退

仅普通航线配置：

```yaml
mission_profiles:
  waypoint-route:
    navigation:
      backend: fixed_min_snap
      laps: 2
```

速度、加速度继续使用 `planning.max_velocity_mps`、`planning.max_acceleration_mps2`，CLI 的 `--speed`、`--acceleration` 保持有效。

过门速度 `planning.gate_velocity_mps`（默认 `null`）：设为正数后，经过过门点（录点按 2）以及相邻过门点之间的速度不超过该值；其余路段仍按航线速度。曲线形状不变，只改变走这条曲线的快慢：在第一个过门点之前用平滑的速度过渡减速，最后一个过门点之后再加速回航线速度，加速度不超过航线上限。只在起飞前计算一次，不增加到点、等待或重试。门前点离门框约 1 m，飞机到门框时已经降到过门速度。过门处本来就不超过该值时，轨迹不变；`null` 时与原实现完全相同。

```sh
sh flight/fly_points.sh --set planning.gate_velocity_mps=0.4 --dry-run
```

```sh
# 临时预览原 EGO 单遍逐点航线；去掉 --dry-run 才会启动飞行。
sh flight/fly_points.sh --route data/routes/field_loop.yaml --set navigation.backend=ego --dry-run
# 更改固定轨迹圈数预览
sh flight/fly_points.sh --route data/routes/field_loop.yaml --set navigation.laps=1 --dry-run
```

设 `backend: ego` 永久回退，保留原到点判据和末点悬停；不会将固定模式的两圈设置套到 EGO 上。比赛和投放入口不受此设置影响。

## 本次保全与后续机载部署

修改前完整本地快照：`/Users/david/Documents/ChatGPT/aquacore-nanjing-backups/20261030-041544-before-fixed-route/`，包含 `workspace.tar`、`manifest.json`、Git 状态与补丁、独立解包及 `VERIFIED.json`。不要以旧 Git 提交覆盖现有未提交工作。

按用户最新要求，本次只在本地实现与验证。后续部署先独立备份机载源码、构建产物、配置和航点，再按变更清单同步；保留机载独立修改，`flight.yaml` 只合入普通航线 navigation 字段，不覆盖现场高度、速度、标定和投放参数。完整文件回滚使用此次变更清单和快照，避免覆盖后来录制的航点及日志。
