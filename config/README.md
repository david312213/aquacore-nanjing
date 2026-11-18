# 配置分工

日常飞行高度、速度、加速度、地图尺寸和既有到点判据只修改 [flight/flight.yaml](../flight/flight.yaml)。公共参数 → 对应入口的任务预设 → `--set`／专用单次参数，后者优先。参数在下次程序或任务启动读取，不热更新。

[mission.yaml](mission.yaml) 是内部运行默认值，不是第二个日常调参入口。红点参数单独保存在 [flight/red_point.yaml](../flight/red_point.yaml)。

遥控器 9 通道参数位于 `flight/flight.yaml` 的 `common.rc_payload`：`closed_pwm_us=900`、`open_pwm_us=1350`，控制 MAIN6/7；接收 PWM ≤`low_max_us=1300` 为低位全开，≥`high_min_us=1700` 为高位不动作，中间为中位全闭合。第 1 终端启动后首个有效档位及后续换挡各发一次，修改配置需重启该终端。

比赛红点开环对准及兜底参数位于 `mission_profiles.delivery-route.delivery_route_options.visual_alignment`：取图等待 15 秒、运动参考结束后宽限 15 秒、每点一次额外重拍、重拍后 XY 各放宽 0.08 m。`red_point_config` 指向共享红点配置，任务启动时固定副本。`mount_offsets_body_m` 保留四个口的 `[前, 左]` 米制偏移接口，当前均为零。

比赛投放参数位于 `flight/flight.yaml` 的 `mission_profiles.delivery-route.payload`：`sequence` 是四次投放编号，`main_outputs` 将编号 1/3 映射到物理 MAIN6、2/4 映射到 MAIN7，`actuator_indices` 对应功能 301/302 的命令参数。`released_pwm_us_by_slot` 依次为 1190、1190、1350、1350 μs；关闭及手动复位均为 900 μs，PWM 上限 2000 μs。配置中的 MIN/MAX 供离线换算，在线使用 MAVROS 缓存的对应物理输出 MIN/MAX 和反向参数；不会写入飞控参数。

机器标定与底层配置独立保留：

- [frames.yaml](frames.yaml)：MID360 与 PX4 安装旋转和平移。
- [fastlio.yaml](fastlio.yaml)：FAST-LIO / MID360 内部参数。
- [px4ctrl.yaml](px4ctrl.yaml)：现有原生指令接口与控制器配置。
- [MID360 网络配置](aircraft/MID360_config.json)。
- [下视 HD USB Camera YUYV 内参](aircraft/down_hd_1280x720_yuyv_calib.yaml) 和 [下视相机启动文件](cameras/down.launch)。启动文件默认是 HD USB Camera 的 1280×720 YUYV 9 fps；红点识别从 `flight/red_point.yaml` 的 `camera` 段传入实际分辨率、格式和内参文件。换相机后用 [下视相机标定](../tools/calibration/down_camera/calibrate.py) 生成新的内参文件。

这次清理不改变安装标定或写入 PX4 参数。包内 config / launch 的相对链接仍指向本工作空间配置，便于整体迁移。
