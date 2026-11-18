# 普通航线双高度、双膨胀交付（2026-10-30）

已部署到 drone24@192.168.90.236 的 /home/drone24/aquacore-nanjing。

## 配置及录点

flight/flight.yaml → mission_profiles.waypoint-route.waypoint_profiles：
- cruise.height_m / cruise.inflation_m
- gate.height_m / gate.inflation_m

两套初值均保持现场普通航线的高度 1.0m、膨胀 0.25m；尚未根据门洞尺寸调整。
所有高度相对实际起飞点。普通航线起飞使用 cruise；地图上限仍为 common.planning.map.max_height_m（现场 1.4m），分辨率 0.15m。
录点按 1 选择巡航、2 选择过门，空格记录；状态显示该类型高度、膨胀，x 撤销，q 保存。
选择保持到下一次切换；新录制/恢复录制初始选择为巡航。旧版纯 XY 文件按巡航点执行。
选择类型后写入 version 2 和与 points 同长度的 point_profiles。文件存类型，任务启动时读取配置值。

```
cd /home/drone24/aquacore-nanjing
sh flight/record_points.sh --output data/routes/field_loop.yaml
sh flight/fly_points.sh --route data/routes/field_loop.yaml --dry-run
sh flight/fly_points.sh --route data/routes/field_loop.yaml
```

## 执行语义

前往目标点时应用目标点类型。门前、门中心、门后可连续录为 gate，之后录 cruise 恢复巡航参数。
EGO 在该航段规划高度过渡；等待新轨迹时保持上一段参考高度，不直接跳到新目标高度。
同一 EGO 实例接收目标、PVA 起点和膨胀参数的绑定消息，收到新目标时原子应用膨胀；历史目标重试不覆盖当前地图。
点云路径保留未膨胀障碍样本，切换时立即重建膨胀，不等待新点云、不重启 EGO。
沿用原来 XY ceil(inflation / resolution) 格、Z 一格的点云膨胀。不同数值可能对应相同栅格半径。
原有到点判据、速度、定位异常处理、投放流程保持；无新增等待、停稳、限龄或重试限制。
最后一点保持其对应高度，不自动降落；可用 hold.sh 停止，safe_land.sh 当前位置降落。
仍为障碍约束下的 EGO 目标点，不保证直线穿门；需要门前留出高度过渡的路线。

## 验证与部署

- 本地与机载航点测试通过（64 个 unittest 用例运行，包含继承/导入导致的重复回归项）；混合高度、上一段保持、膨胀绑定、录点/撤销/恢复均有覆盖。
- 投放文件兼容 11 项、原比赛录点 13 项、本地任务接口真实 C++ 回调 4 项、坐标启动接口 4 项通过。
- 机载全量 catkin_make -j2 编译成功；机载真实 C++ 回调测试 4 项通过。
- 实际 ego_worker.launch 在独立 ROS master 下运行，只有合成 odometry/cloud，没有 MAVROS/PX4/舵机连接。
- 停止新点云后验证 0.35 → 0.10 → 0.35 → 0.10m，查询真实规划地图占用依次变化；每段取得 40 个目标绑定指令样本，起始参考 Z 与给定 PVA 一致，VZ 未超过测试上限。
- 重发旧 0.35m 目标，当前 0.10m 地图保持不变。
- 初次测试收据订阅连接尚未完成导致测试超时；补齐测试端连接同步后通过，未向实际任务添加任何等待。
- 机载 dry-run 验证不同高度/膨胀按点预览；17 个部署文件 SHA256 一致。
- 现场 prediction_mode、盲区、比赛分辨率/膨胀、0.25m 投放高度及取图进程生命周期保留。现场 flight_core.py 的独立改动未覆盖。
- 已退出空闲且落地状态的旧管理器；下次入口自动启动新版本。测试进程已退出。没有执行实飞。

证据及部署清单：data/fixes/20261030-route-profiles/，机载同路径。
修改前关键源码在 before.tgz，运行产物在机载 before-runtime.tgz；恢复源码后需重新编译，不能混用新旧 ROS 消息产物。
