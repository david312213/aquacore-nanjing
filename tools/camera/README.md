# 独立相机录像

在机载电脑桌面终端运行，不需要启动前三个飞行终端：

```sh
cd /home/drone24/aquacore-nanjing
sh flight/record_video.sh
```

直接打开下视相机（HD USB Camera），以 1280×720、MJPEG、30 fps 请求采集；显示原始画面，同时保存 30 fps MJPEG AVI。没有识别框、坐标或画面叠字，不启动 ROS、定位、识别、控制器或投放。若相机正在被红点识别等程序占用，先退出该程序。

按 q、Esc、Ctrl-C 或关闭窗口结束并保存。视频位于 `data/videos/df200_年月日-时分秒-微秒.avi`，每次生成新文件。图像中断时保存已经录制的部分；采集实际帧率受相机和电脑影响，终端结束时显示实测平均值，文件帧率固定为 30 fps。

可用 `--device /dev/video0` 指定相机、`--output-dir /路径` 指定保存目录。`--dry-run` 仅查看参数，不打开设备或窗口。

## 绿色圆心离线验证

`python3 tools/camera/evaluate_green_centers.py <JPG目录> --sweep` 只读取图片，输出逐图候选、参数扫描和标注预览到 `data/checks/green_circle/`；不会启动相机、ROS 或飞行。详见 [视觉说明](../../flight/RED_POINT.md)。
