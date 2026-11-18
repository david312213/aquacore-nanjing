#!/usr/bin/env python3
"""Direct down-camera capture and raw preview, with 30-fps MJPEG AVI recording."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
DEVICE = '/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0'
FPS = 30.0
WINDOW = 'DOWN REC 30 FPS - q: save and quit'


def record(device, output):
    import cv2
    capture = writer = None
    window = False
    frames = 0
    started = None
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        window = True
        cv2.resizeWindow(WINDOW, 1280, 720)
        source = int(device) if device.isdigit() else device
        capture = cv2.VideoCapture(source, cv2.CAP_V4L2) if sys.platform.startswith('linux') else cv2.VideoCapture(source)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        capture.set(cv2.CAP_PROP_FPS, FPS)
        if not capture.isOpened():
            raise RuntimeError('无法打开相机：'+device+'；如果红点识别正在使用相机，请先退出识别。')
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError('相机没有返回图像；请检查设备连接或是否被其他程序占用。')
        height, width = frame.shape[:2]
        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'MJPG'), FPS, (width, height))
        if not writer.isOpened():
            raise RuntimeError('无法创建录像：'+str(output))
        print('正在录制：'+str(output), flush=True)
        print('原始画面 {}×{}；请求采集 30 fps，相机报告 {:g} fps；视频编码 30 fps。'.format(
            width, height, capture.get(cv2.CAP_PROP_FPS)), flush=True)
        print('按 q / Esc 或关闭窗口结束并保存。', flush=True)
        started = time.monotonic()
        while ok and frame is not None:
            # Record the original frame; display never adds overlays to the file.
            writer.write(frame)
            frames += 1
            cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0:
                    break
            except cv2.error:
                break
            ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError('相机图像中断，已保存读取到的视频。')
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if writer is not None:
            writer.release()
        if capture is not None:
            capture.release()
        if window:
            try:
                cv2.destroyWindow(WINDOW)
            except cv2.error:
                pass
        if frames:
            elapsed = max(time.monotonic() - started, .001)
            print('已保存 {} 帧，采集平均 {:.1f} fps：{}'.format(frames, frames / elapsed, output), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description='仅打开下视原始画面并录制 30 fps AVI；不启动 ROS、识别或飞行。')
    parser.add_argument('--device', default=DEVICE, help='相机设备路径或编号，默认当前下视 HD USB Camera')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'data/videos', help='视频保存目录')
    parser.add_argument('--dry-run', action='store_true', help='仅预览参数，不打开相机或窗口')
    args = parser.parse_args(argv)
    output = args.output_dir.expanduser().resolve()/('df200_'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.avi')
    if args.dry_run:
        print(json.dumps(dict(device=args.device, width=1280, height=720, capture_fps=FPS,
                             video_fps=FPS, codec='MJPG', output=str(output), preview=True,
                             ros=False, detection=False, flight=False), ensure_ascii=False, indent=2))
        return 0
    if sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('请在机载电脑的桌面终端运行，以显示相机窗口。', file=sys.stderr)
        return 1
    return record(args.device, output)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('录像结束：'+str(exc), file=sys.stderr)
        sys.exit(1)
