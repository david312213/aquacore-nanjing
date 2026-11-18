# 绿色圆心粗对准与红点精对准

只在终端持续打印无人机 XYZ、红点中心 XY 和相对飞机 ΔXY：

```sh
sh flight/vision_coords.sh
```

该脚本复用下述识别与投影，每秒默认打印 5 次，不开图像窗口。只有绿环、白圆、红点通过现有共心确认后才打印，每个确认靶标一行：飞机 XYZ、红点中心 XY、ΔXY。飞机坐标使用同一图像时刻的定位，单位米；未确认或无红点时不打印，也不沿用旧靶心坐标。按 q 或 Ctrl-C 退出。传感器和定位需已运行；本脚本不启动飞行或投放。可加 `--dry-run` 查看配置；详细诊断仍使用 `red_point.sh --no-window`。

```sh
sh flight/red_point.sh
```

先开传感器和定位两个终端。红点入口独立启动或复用 DF200，不启动控制器、任务管理器、EGO 或投放。退出 q 只清理它自己启动的相机。

上述命令用于独立观察。比赛航线只在投放点取图阶段自动启动无窗口红点进程、取到这一帧即关闭，复用同一识别、投影和配置，将结果交给红点开环对准流程；无需额外打开本预览。比赛启动时固定配置副本，修改后在下次任务生效，详见 [比赛航线](DELIVERY_ROUTE.md)。

使用原始 1280×720 彩色图，转换到 HSV 分别提取红色、绿色、白色。红点采用全分辨率连通域质心，绿环外边缘和中央白色圆盘使用椭圆拟合，此红点确认分支保持椭圆拟合。另有独立绿色圆环霍夫检测供第一阶段使用。按轮廓层级、包含关系和半径比例区分中央白圆与外侧白底，也不把绿环内缘当成外缘。

红点分支只有红点、中央白圆、绿环外缘三者共心确认后，才换算有效红点 XY。三个中心之间的最大距离需小于等于 `max(6 px, 白圆等效半径 × 15%)`；用于投影的像素始终是红点质心，不平均三个中心。这个确认表示靶标识别成立，不表示飞机已经对准，本身不表示飞机到位。

圆环部分出画时先排除图像裁切边，再使用可见边缘拟合；不足以独立拟合两圈时输出“未确认”。未确认候选仍保留像素和原因，XY 与相对偏移为空；未检测到红点时输出空候选，不沿用旧结果。多个候选都保留，确认候选排在前面，C1/C2 仅表示本帧顺序，不分配投放编号。每帧独立确认，没有连续多帧等待。

## 参数

所有识别参数在 [red_point.yaml](red_point.yaml) 的 `detector` 中：

| 参数 | 默认值与含义 |
| --- | --- |
| `red_ranges` / `green_range` | 沿用现有红色、绿色 HSV 范围 |
| `white_range` | H=0～179，S=0～70，V=80～255；去掉已分为红、绿的像素 |
| `min_contour_points` | 排除图像裁切边后至少 12 点 |
| `min_arc_degrees` | 拟合椭圆上可见轮廓覆盖至少 120° |
| `max_radial_residual` | 按拟合半轴归一化的平均径向误差不超过 0.16 |
| `white_green_radius_ratio` | 白圆 / 绿环外缘等效半径为 0.20～0.85 |
| `concentric_tolerance_px` | 共心容许偏差的像素下限，默认 6 px |
| `concentric_tolerance_ratio` | 共心容许偏差相对白圆半径的比例，默认 0.15 |

等效半径为两条椭圆半轴的几何平均。以上均为图像识别参数，不是飞行高度、停稳或任务准入条件；现场需结合真实光照、目标尺寸与画面检查。

当前再放宽一档：共心偏差从 `max(4 px, 半径 × 10%)` 改为 `max(6 px, 半径 × 15%)`，可见弧度从 150° 改为 120°，拟合误差从 0.12 改为 0.16，半径比例从 0.25～0.8 改为 0.20～0.85。仍要求三种颜色和内外圈配对成立；颜色阈值、红点质心和坐标投影保持原样。放宽后允许更大的形状与中心偏差，也可能增加误确认，需用现场画面复核。修改 YAML 后按 q 退出预览，再运行 `sh flight/red_point.sh` 才会加载新值；只打印时运行 `sh flight/red_point.sh --no-window`。

如果原图只有红点和白底，绿环完全在画外，放宽这些参数仍不能确认靶标。先拉开镜头与靶面距离，让绿环外缘和中央白圆边界进入画面，再检查识别。`nonpositive_camera_height` 是独立的高度换算问题，需要核对定位零点和靶面高度，不能通过放宽识别阈值解决。

## 画面、坐标与接口

高度自动使用 `planning_odometry.z + 0.015 m`，无需 g 登记。前提是本次定位在与靶面同一地面初始化；启动识别、抬高或移动飞机不会重置高度基准。

相机格式和内参在 `red_point.yaml` 的 `camera` 段：`video_device`、`image_width`、`image_height`、`pixel_format`、`framerate`、`calibration`。`calibration` 指向 ROS camera_info 格式的内参文件，相对路径按工作空间根目录解析，其分辨率须与相机实际输出一致；没有这些键的旧配置按当前 HD USB Camera 1280×720 YUYV 9 fps 和 `config/aircraft/down_hd_1280x720_yuyv_calib.yaml` 处理（MJPEG 视场不同，不能沿用这份内参）。安装方向 `pixel_to_body` 与镜头离地高度不随相机自动改变，换相机后需在现场核对。换相机后的内参标定：

```sh
python3 tools/calibration/down_camera/calibrate.py
```

先停止占用相机的进程。把 11×8 内角点、30 mm 方格的标定板放平，移动飞机，让标定板出现在画面四角、四边和中心，并有远近变化；每个位置把飞机前后左右各倾斜约 30°。每换一个姿态停稳约 1 秒，工具只保存静止且姿态不同的帧，每 5 秒提示还缺哪些边角区域和倾斜方向。结束后在 `data/calibration/down_camera/<时间>/camera_info.yaml` 写出内参，打印重投影误差、fx 等参数的标准差、各方向倾斜张数和未覆盖区域。这只镜头畸变很大，靠近画面边角的帧和明显倾斜的帧决定畸变和焦距能否标准。确认后复制到 `config/aircraft/`，再修改 `camera.calibration`。

保存过的帧可离线重算，结果写回同一目录：`python3 tools/calibration/down_camera/calibrate.py --images data/calibration/down_camera/<时间>`。

终端默认每秒 5 次打印图像时刻飞机 XYZ、相机高度、时间差和检测耗时；已确认候选打印像素、目标 XY、相对飞机 XY 及共心误差，未确认候选打印像素与原因。出现／消失／确认状态改变立即打印，误差数值变化仍遵循 `console.rate_hz`，不会因此逐帧刷屏。

窗口保持原图尺寸，显示 `TARGET CONFIRMED`、`UNCONFIRMED`、`NO RED POINT` 、`IMAGE ERROR` 或 `GREEN CENTER FOUND`，并分别标记 `GREEN EST CENTER`（绿色估算圆心）与 `RED CENTER`（真实红点质心）。坐标与误差不叠加到视频。保留仅处理最新帧、只在有新结果时刷新画面、无图像订阅者时不序列化标注图的行为。

图像按时间查询定位历史：前后样本插值，否则最近样本并显示时间差，不增加等待条件。坐标系为 `fast_lio_local_enu`；仍是近水平、航向旋转投影，不声称完成全姿态三维外参投影。缺少定位时仍显示像素，XY 不可计算。定位会话变化后清空定位历史。

观察话题为 `/airseafly/red_point/status`（JSON String，`airseafly.red_point.v1`）和 `/airseafly/red_point/image_annotated`（Image，实际名字见 YAML）。状态新增 `confirmed_count`；每个候选包含：

- `target_confirmed`、`confirmation_reason`：是否确认及原因。原因值为 `confirmed`、`missing_green_outer`、`missing_white_inner`、`invalid_nesting`、`not_concentric`。
- `circles.green_outer`、`circles.white_inner`：选中配对的中心、轴长、角度、等效半径、拟合误差和可见弧度；无法配对时为空。
- `center_error_px`、`center_tolerance_px`：实际三中心最大距离与容许值；无法配对时为空。
- `xy_m`、`offset_xy_m`：仅在确认且定位可投影时非空；未确认时 `projection_status=target_unconfirmed`。

根级 `projection_status` 继续表示定位与高度是否可用于投影，不能替代候选的确认状态。旧的辅助排序字段 `green_support` 和三个 `green_*` 排序参数已由轮廓配对取代。

## 离线与现场验证

```sh
sh flight/red_point.sh --dry-run
sh flight/red_point.sh --image src/airseafly_real400_mission/test/fixtures/red_point_target.jpg --no-window
```

离线图片仅验像素，附图质心约 `(457.55, 682.06)`，共心误差约 `7.84 px`，当前容许值约 `30.71 px`；不会为照片虚构米制坐标。现场约 1 m、近水平观察后，移动已知距离验证轴向、比例和固定靶心 XY 的一致性。椭圆拟合允许轻微倾斜，但没有进行透视姿态校正；像素识别、坐标误差与以后自动对准分别验收。

## 第一阶段绿色圆心

`detector.green_hough` 集中配置霍夫参数。先提取绿色掩膜，仅为霍夫投票填充闭合内部孔洞，避免内圈压制同心外圈；再进行霍夫圆检测。原始绿色掩膜检查圆环内侧/外侧颜色支持及非绿色中心，排除实心绿色块；只统计画内可见弧段。此分支不要求白色或红色，不使用红点修正圆心。输出坐标恢复为原图像素再复用既有投影。

观察消息保持 `airseafly.red_point.v1`、`candidates` 和 `confirmed_count` 为原红点含义，增加 `green_candidates` 与 `green_count`。绿色候选包含 `class_name=green_circle`、`pixel`、`radius_px`、`target_confirmed`、`xy_m`、`projection_status`；共用同一图像时刻、任务会话和图像序号。预览与无窗口任务进程共用 `detect_targets`；原 `detect` 仍为红点接口。`vision_coords.sh` 继续只打印确认红点，不打印绿色估算圆心。

比赛、单点及录点投放已改为到点直接下降至识别高度，只读取红点候选对准一次，不再使用绿色候选；绿色检测保留给手持检查与离线评估。无结果按已有 15 秒流程处理。强透视、模糊、严重裁切可能漏检或使霍夫圆心偏移；特别是绿色环完全出画时，第二阶段原有三色确认仍可能失败，本次不改变其确认标准。

离线回放：

```sh
python3 tools/camera/evaluate_green_centers.py data/frame_exports/df200_20261027-000122/images --sweep
```

输出到 `data/checks/green_circle/`：逐图候选、参数比较、检测耗时和全量标注缩略图。该视频帧集用于调参，不是独立标注准确率或实飞验收。

## 两次手持对准观察

已有传感器与定位时保持运行，不要举起后重新初始化定位。依次运行：

```sh
# 手持到约 1 米：仅看绿色圆环估算圆心
sh flight/test_align_green.sh
# 按 q 退出第一个，再手持到约 0.5 米：仅看原三色确认小红点
sh flight/test_align_red.sh
```

两者自动显示相机画面，确认目标时默认每秒打印一次飞机 XYZ、靶心 XY、ΔXY 和像素坐标；未识别不重复刷空结果。绿色脚本不要求小红点，红点脚本仍要求绿环、白圆与小红点确认。高度只是手持参考，不是脚本触发条件，两者都不控制飞机、不投放，也不启动定位或 EGO。复用已有相机，无发布者时仅启动 DF200；不要同时运行这两个脚本。

需现有 ROS 环境；没有定位时仍可看标记和像素，米制坐标显示不可计算。没有桌面显示时自动只打印，亦可加 `--no-window`。按 q、Esc 或 Ctrl-C 退出；`--dry-run` 只检查配置。
