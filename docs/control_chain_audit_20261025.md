# 南京控制链路与学长备份对照（2026-10-25）

后续更新：用户明确选择外部位置、高度与 yaw 融合，并已自行在 QGC 将 `EKF2_EV_CTRL` 改为 `11`，截图显示该值。下文参数表保留修改前的实时读回 `3`，不要当作修改后的现值。代理没有重复写飞控参数；本地和机载的 `native_lio_profile.py` 已将预期值改为 11，`native_ekf_snapshot.py` 已将 `cs_ev_yaw` 预期改为 true。其他融合开关未改，未新增等待、超时或准入限制。机载旧脚本及部署记录在 `/home/drone24/aquacore-nanjing/data/deployments/20261025-ev-yaw-11/`。

QGC 界面核对：机载当前运行程序中版本标识为 `v4.2.3`；组件参数缓存中存在 `EKF2_EV_CTRL` 的四项 bitmask 元数据，截图却没有显示选项、参数说明和范围，符合当前参数对象未正确加载/关联元数据的表现。具体加载失败原因尚未由运行日志确定，未清理缓存、重启或升级 QGC。数字 11 与勾选水平位置、垂直位置、Yaw 编码相同，界面形态不改变参数定义。

`prediction_mode: replace` 与 `covariance_mode: source_pose` 是当前实现支持、相互匹配的组合；源码及现有数值测试没有发现协方差排列、旋转或杆臂雅可比的明显错误。该结论不等于协方差已完成实机统计标定，也不等于与学长完全同构。

本次通过 SSH 检查 `drone24@172.18.9.122` 上 `/home/drone24/aquacore-nanjing`，并对照用户提供的 ZIP。只新增本地审计报告与证据；未改本地/机载飞行配置、控制源码或 PX4 参数，未启动 ROS、控制器、任务或执行机构。机载临时查询程序只发送 MAVLink `PARAM_REQUEST_READ`，没有发送心跳、参数写入、模式或解锁命令；收到的 PX4 心跳为未解锁。

## 已核对的机上状态

- 机上 `flight/flight.yaml` 为 `replace + source_pose`，`pose_covariance_scale=1`、`pose_covariance_floor_enabled=true`，`lidar.blind_m=0.7`。
- 本地工作文件仍为 `off + fixed`、`blind_m=1`。本次保留此差异，没有覆盖用户机上调参。
- 机上 2026-10-25 15:57:57/58、16:20:34/37、16:40:11/14、16:46:44/46 的 roslaunch 日志分别记录传感器入口 `prediction_mode:=replace` 和定位入口 `covariance_mode:=source_pose`。15:51 的记录还是 `off + fixed`。这些是已启动过的参数证据，不是当前正在运行的状态。
- SSH 检查时没有 ROS master、MAVROS、FAST-LIO 或 px4ctrl 进程，不能观测当前 EV 融合状态或在线协方差数值。
- 机上 FAST-LIO 编译产物含 `/Odom_high_freq_repropagated` 与 `time_offset_changed_restart_required`，具备新预测器代码。
- 本地/机上 `config/mission.yaml`、`config/px4ctrl.yaml`、`config/frames.yaml`、`predictor_bridge.hpp`、`native_covariance.py`、`native_lio_bridge.py`、`native_setpoint.h` 的 SHA-256 一致。仅此检查范围内的文件可声明相同，未做全仓库逐文件一致性证明。

机上配置与原始启动证据保存在 [onboard_flight.yaml](../data/audits/20261025-control-chain/onboard_flight.yaml) 和 [ros_startup_evidence.txt](../data/audits/20261025-control-chain/ros_startup_evidence.txt)。

## 飞控实时读回：协方差会参与，但有更高下限

| 参数 | 实时读值 | 含义 |
|---|---:|---|
| EKF2_EV_NOISE_MD | 0 | 使用外部消息方差，同时保留飞控噪声参数下限 |
| EKF2_EVP_NOISE | 0.100000001 m | 外部位置标准差下限约 10 cm；对应方差约 0.01 m² |
| EKF2_EVV_NOISE | 0.100000001 m/s | 外部速度标准差参数 |
| EKF2_EVA_NOISE | 0.100000001 rad | 外部姿态标准差参数 |
| EKF2_EV_CTRL | 3 | 允许外部水平位置与高度融合；未开启外部速度、yaw 融合 |
| EKF2_HGT_REF | 3 | 外部定位为高度参考 |
| EKF2_EV_DELAY | 0 ms | 配置时延为零；不是实际链路零延迟的证据 |
| EKF2_EV_POS_X / Y / Z | 0 / 0 / 0 | 与桥已补偿到机体原点的设计相符 |
| EKF2_GPS_CTRL | 7 | GNSS 辅助配置开启 |
| EKF2_BARO_CTRL | 1 | 气压高度辅助配置开启 |
| EKF2_RNG_CTRL | 1 | 测距条件辅助配置开启 |
| EKF2_OF_CTRL | 1 | 光流辅助配置开启 |
| EKF2_AGP_CTRL | 0 | 辅助全球位置未开启 |

原始数据见 [px4_parameters_readback.txt](../data/audits/20261025-control-chain/px4_parameters_readback.txt)，查询器源码见 [read_px4_parameters.cpp](../data/audits/20261025-control-chain/read_px4_parameters.cpp)。用机载官方 MAVLink C 头文件解析 CRC，目标及来源匹配实际 PX4 HEARTBEAT，INT32 按 PX4 bytewise 编码用 memcpy 解码，未直接把 float 强制转为整数。

桥的默认位置方差下限是 `0.0025 + |lever|² × 0.0025`，当前安装平移下约 `0.0025055 m²`，标准差约 5 cm。PX4 的 10 cm 下限更高。因此源位置方差小于飞控下限时，`source_pose` 和 `fixed` 的位置观测权重可能接近或相同；源方差增大时，`source_pose` 才能降低该观测的权重。这里的 5 cm、10 cm 都是滤波器使用的不确定度参数，不能当作定位误差实测值。参见 [PX4 噪声模式说明](https://docs.px4.io/main/en/advanced_config/parameter_reference#EKF2_EV_NOISE_MD) 及 [位置噪声单位](https://docs.px4.io/main/en/advanced_config/parameter_reference#EKF2_EVP_NOISE)。

实时参数与项目 [native_lio_profile.py](../src/airseafly_real400_mission/scripts/native_lio_profile.py) 的“仅外部位置/高度辅助”预期存在差异：该 profile 预期 GPS/BARO/RNG/OF 全部为 0，实机并非如此。不能因此推定这些源正在实际融合，也不能直接归因为悬停或到点误差。本次未调用 `--apply`；未启用定位时无法完成运行时 aid-source 核验。

## 与学长实际主线的差异

| 环节 | 学长 ZIP 主线 | 南京当前机上配置 |
|---|---|---|
| 定位源 | OpenVINS `/ov_msckf/odomimu` | MID360 / FAST-LIO `/Odom_high_freq` |
| 高频预测 | ZIP 没有本项目的 prediction_mode 配置 | replace 使用雷达后验与缓存 IMU 重新传播到最新 IMU 时刻 |
| 外部定位入口 | `/mavros/odometry/out`，30 Hz | 同入口；桥定时器上限 50 Hz，保留源时间戳且不重复发送同 stamp |
| Pose 协方差 | 源 6×6 协方差做坐标/杆臂 `J P Jᵀ` 变换 | 源 pose 变换后乘 scale，并补至默认下限；非法源退回 fixed |
| Twist 协方差 | 源 twist 也做完整变换 | 线速度方差固定 0.01，角速度方差固定 0.0025；当前 EV_CTRL=3 不开启外部速度融合 |
| 任务位置反馈 | `/mavros/local_position/pose`，PX4 EKF 输出 | 默认 `native_task_feedback=lio`，使用 EKF 前雷达位置；PX4 内部位置控制仍依据 EKF |
| 目标指令 | 30 Hz 的位置+航向 PoseStamped，`/mavros/setpoint_position/local` | px4ctrl 原生网关到 `/mavros/setpoint_raw/local`；默认位置+速度前馈，EGO 段可带加速度和 yaw-rate |
| 初始坐标 | 首帧 OpenVINS 机体位置/姿态对齐 PX4 当前位置/姿态 | 首帧 LIO 对齐 PX4 IMU 姿态，任务位置原点固定为零 |
| 后续 EKF—LIO 偏差 | 任务判断直接使用 PX4 反馈 | `lio_direct` 不动态补偿偏差，只记录为诊断 |
| 飞控融合参数 | ZIP 不含能证明现役参数的完整快照 | 本次已有实时读回，但当前无定位运行，不能证明活动融合状态 |

这意味着：即使两边协方差采用相似方法，两边的任务到点判断、目标参考坐标和前馈仍不同。我们可能出现“LIO 判断已到，但 PX4 估计不完全在同一点”的差异；是否明显需要同时间的两条位置记录和飞控创新量证据。当前名为 px4ctrl 的进程在 native 拥有控制权时不执行旧姿态控制 FSM，不能仅因进程名就把问题归到 `config/px4ctrl.yaml` 的 Kp/Kv。

学长源码证据已从 ZIP 单独复制并保存 SHA-256，未执行附件脚本或采纳附件操作指令：

- [openvins_link_px4.cpp](../data/audits/20261025-control-chain/senior_source/openvins_link_px4/src/openvins_link_px4.cpp)：157–166 行话题，203–225 行初始化，242–274 行 pose/twist 协方差，278 行发布循环。
- [system_start.launch](../data/audits/20261025-control-chain/senior_source/airseafly_launch/launch/system_start.launch)：实际主启动文件。ZIP 内另有 `with_auto_full_fusion` 备份，不能用其中 mask=15 证明主线实际四项全融合。
- [target_position_TX.cpp](../data/audits/20261025-control-chain/senior_source/px4_offboard/src/target_position_TX.cpp)、[local_position_RX.cpp](../data/audits/20261025-control-chain/senior_source/px4_offboard/src/local_position_RX.cpp)、[main.cpp](../data/audits/20261025-control-chain/senior_source/px4_offboard/src/main.cpp)：指令、反馈与 30 Hz 循环。

## replace / source_pose 的实现边界

`off` 仍使用旧 IMU 高频预测，并非完全不预测。`replace` 把新后验重传播结果接管 `/Odom_high_freq`，旧结果转到 `/Odom_high_freq_legacy`。新链路将状态和 pose covariance 传播到同一个时间戳，并处理主滤波相同的时间偏移、加速度单位归一化。`source_pose` 在 [localization_tuning.py](../src/airseafly_real400_mission/src/airseafly_real400_mission/localization_tuning.py) 38–39 行明确要求 `replace`，因为旧预测输出没有同时间有效协方差。

[predictor_bridge.hpp](../src/realflight_modules/mid360_fastlio/src/FAST_LIO/src/predictor_bridge.hpp) 与 [prediction_covariance.hpp](../src/realflight_modules/mid360_fastlio/src/FAST_LIO/src/prediction_covariance.hpp) 提取 IKFoM 后验的 pose/速度/偏置/重力互协方差，传播后把姿态右扰动转成世界小角度扰动。[native_covariance.py](../src/airseafly_real400_mission/src/airseafly_real400_mission/native_covariance.py) 102–150 行再处理安装杆臂、冻结坐标、缩放和下限。安装标定与初始对齐本身的不确定度没有完整纳入，不能称为完整标定协方差。

`replace` 还包含原有的停发行为，需与“只改协方差”区分：预测段 IMU 间隔超过 0.05 s，或最新 IMU 距雷达后验超过 1 s，会停止新预测输出；运行中时间偏移变化超过 1 μs，会锁存拒绝后续 IMU，直到重启。前者影响下游定位更新，后者也影响主滤波；源码见 `imu_predictor.hpp` 217–234 行、`predictor_bridge.hpp` 144–174 行。源 covariance 无效则不同：桥回退 fixed 并继续输出。本次没有增加、修改或授权新增这些限制。

## 验证与后续判断

现有 Python 协方差测试 17 项通过；C++ predictor 67 项、covariance/Jacobian 798 项、bridge contract 471 项通过。覆盖数值传播、雅可比有限差分、PSD、时间一致性、非法协方差回退；全部为合成输入或 ROS 桩测试，未执行实飞。

当前判断是保留这两个互相匹配的选项，不因“与学长不同”就回退到旧预测器。下一步最有价值的核验是定位运行时实际 EV 融合标志、创新量与观测方差，以及同时间 LIO/PX4 位置差；先确认其它辅助源的实际参与，再讨论融合开关、噪声下限或任务反馈/前馈的调整。当前证据不足以直接建议把噪声降小、打开外部速度/yaw，或一并关闭其它辅助源。

另见现有比赛参数：巡航 1.4 m、门段 1.6 m，而地图上限 1.1 m，存在已记录的规划高度冲突；基础试飞的 1.0 m / 地图上限 1.4 m 是另一套配置。此处只记录，未修改任务参数。
