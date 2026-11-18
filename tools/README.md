# 工具目录

- `camera/`：独立相机可视化与 30 fps 录像，见[使用说明](camera/README.md)；不启动 ROS 或飞行。

- `calibration/mid360_px4/`：安装标定采集与提取；原始数据保留在 `data/calibration/`。
- `calibration/down_camera/`：下视相机棋盘格内参标定，无界面、不启动 ROS；输出 camera_info，见 [红点说明](../flight/RED_POINT.md)。
- `stress/`：地面压力测试，拆桨后叠加红点识别与 EGO，只读记录 CPU 与 FAST-LIO 延迟，见[使用说明](stress/README.md)。
- `px4/`：已有只读参数、融合和 MAVLink 控制台检查工具。
- `mapping/`：保留雷达地图观察工具。
- `remote_rviz/`：在 Mac 上用 RViz 实时查看 EGO 膨胀地图、规划定位和轨迹（只看，不含 2D Nav Goal）。机载照常启动即可，1 号终端会打印 IP；Mac 上在本项目副本里运行 `bash tools/remote_rviz/mac_rviz.sh <飞机IP>`，IP 变化或首次需输入 Mac 密码写 `/etc/hosts`。机载各入口按主机名公布 ROS 地址（`scripts/ros_network.sh`），机内通信走回环；只想本机可见时启动前设 `AIRSEAFLY_ROS_IP=127.0.0.1`。ROS1 无鉴权，同网段可读写话题。
- `payload/`：地面执行器测试工具；MAIN6/7 与 MAIN1 自定义 PWM 测试见 [使用说明](payload/README.md)。开发验证只做模拟和 dry-run，不连接机构。

日常起飞、录点、保持、降落见 [flight](../flight/README.md)。旧双目、神经网络过门与旧自动 PID 向导已移出工作工程，清理前源码备份可恢复。
