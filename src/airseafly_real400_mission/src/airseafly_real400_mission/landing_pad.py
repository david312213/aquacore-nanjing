"""Down-camera YOLO11n landing-pad (H) boxes for the final landing.

Observation only: no projection, target selection or flight interface. The
ONNX network runs in a child interpreter because the onboard system OpenCV 4.2
cannot load these YOLO11 exports. Letterbox, decode and box mapping live here
so the worker and the child share one implementation. This file has no package
imports; the worker runs it directly as the child script.
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time

import cv2
import numpy as np


PROJECT = Path(__file__).resolve().parents[4]
DEFAULTS = dict(enabled=False, model='models/yolo11n_20261016/down/down_yolo11n_best.onnx',
                python='runtime/yolo_fast/bin/python',
                class_names=['tent', 'bunker', 'bridge', 'armored_vehicle', 'red_cross', 'landing_pad'],
                target_class='landing_pad', input_size=640, confidence=.45, nms_iou=.70,
                opencv_threads=1, reply_timeout_s=2.)


def validate_settings(settings):
    """Configuration computability only; none of these values gate the flight."""
    value = dict(DEFAULTS, **(settings or {}))
    if not isinstance(value['enabled'], bool):
        raise ValueError('landing_pad.enabled 必须是 true/false')
    names = value['class_names']
    if (not isinstance(names, (list, tuple)) or not names or len(set(names)) != len(names)
            or any(not isinstance(name, str) or not name for name in names)):
        raise ValueError('landing_pad.class_names 必须是不重复的类别名列表')
    if value['target_class'] not in names:
        raise ValueError('landing_pad.target_class 必须在 class_names 中')
    for key in ('input_size', 'opencv_threads'):
        if isinstance(value[key], bool) or not isinstance(value[key], int) or value[key] < 1:
            raise ValueError('landing_pad.%s 必须是正整数' % key)
    if value['input_size'] % 32:
        raise ValueError('landing_pad.input_size 必须是 32 的倍数')
    for key in ('confidence', 'nms_iou'):
        if isinstance(value[key], bool) or not 0 < float(value[key]) <= 1:
            raise ValueError('landing_pad.%s 必须在 (0, 1] 内' % key)
    timeout = value['reply_timeout_s']
    if isinstance(timeout, bool) or not math.isfinite(float(timeout)) or timeout <= 0:
        raise ValueError('landing_pad.reply_timeout_s 必须是正数')
    if not isinstance(value['model'], str) or value['python'] is not None and not isinstance(value['python'], str):
        raise ValueError('landing_pad.model / python 必须是路径字符串（python 可为 null）')
    value['class_names'] = list(names)
    return value


def resolve(path):
    path = Path(path).expanduser()
    return path if path.is_absolute() else PROJECT/path


def letterbox(image, size):
    """BGR uint8 to the model's 114-padded square, plus the inverse mapping."""
    height, width = image.shape[:2]
    ratio = min(size/float(width), size/float(height))
    resized_width = max(1, min(size, int(round(width*ratio))))
    resized_height = max(1, min(size, int(round(height*ratio))))
    left, top = (size-resized_width)//2, (size-resized_height)//2
    canvas = np.full((size, size, 3), 114, np.uint8)
    canvas[top:top+resized_height, left:left+resized_width] = cv2.resize(
        image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    return canvas, dict(scale_x=resized_width/float(width), scale_y=resized_height/float(height),
                        left=left, top=top, width=width, height=height)


def decode(output, class_count, target_id, confidence, nms_iou):
    """YOLO11 raw head (1 x 4+classes x anchors) to target-class boxes in model pixels."""
    values = np.asarray(output, dtype=np.float32)
    if values.ndim != 3 or values.shape[0] != 1 or values.shape[1] != 4+class_count:
        raise ValueError('YOLO 输出应为 1×%d×N，实际 %s' % (4+class_count, values.shape))
    rows = values[0].T
    scores, classes = rows[:, 4:].max(axis=1), rows[:, 4:].argmax(axis=1)
    keep = (np.isfinite(rows).all(axis=1) & (classes == target_id) & (scores >= confidence)
            & (rows[:, 2] > 0) & (rows[:, 3] > 0))
    rows, scores = rows[keep], scores[keep]
    if not len(rows):
        return []
    boxes = np.column_stack((rows[:, 0]-rows[:, 2]/2, rows[:, 1]-rows[:, 3]/2, rows[:, 2], rows[:, 3]))
    picked = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), float(confidence), float(nms_iou))
    result = []
    for index in np.asarray(picked).reshape(-1):
        x, y, w, h = (float(v) for v in boxes[int(index)])
        result.append([x, y, x+w, y+h, float(scores[int(index)]), int(target_id)])
    return sorted(result, key=lambda row: -row[4])


def to_candidates(rows, transform, class_name):
    """Model-pixel boxes back to clipped source pixels, as worker candidates."""
    limits = (transform['width'], transform['height'])*2
    output = []
    for x1, y1, x2, y2, score, _ in rows:
        box = [(x1-transform['left'])/transform['scale_x'], (y1-transform['top'])/transform['scale_y'],
               (x2-transform['left'])/transform['scale_x'], (y2-transform['top'])/transform['scale_y']]
        box = [min(max(v, 0.), limit) for v, limit in zip(box, limits)]
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        output.append(dict(class_name=class_name, pixel=[(box[0]+box[2])/2., (box[1]+box[3])/2.],
            bbox_xywh=[int(round(box[0])), int(round(box[1])),
                       int(round(box[2]-box[0])), int(round(box[3]-box[1]))],
            score=float(score), target_confirmed=True, confirmation_reason='yolo'))
    return output


def serve(model, size, class_count, target_id, confidence, nms_iou, threads):
    """Child loop: one raw letterboxed BGR frame in, one JSON line out."""
    reply = sys.stdout
    try:
        cv2.setNumThreads(threads)
        net = cv2.dnn.readNetFromONNX(model)
        net.setInput(np.zeros((1, 3, size, size), np.float32))
        net.forward()
    except Exception as exc:
        reply.write(json.dumps(dict(error='模型加载失败：%s' % exc), ensure_ascii=False)+'\n')
        reply.flush()
        return 1
    reply.write(json.dumps(dict(ready=True, opencv=cv2.__version__))+'\n')
    reply.flush()
    frame_bytes = size*size*3
    source = sys.stdin.buffer
    while True:
        data = source.read(frame_bytes)
        if len(data) < frame_bytes:
            return 0
        start = time.perf_counter()
        try:
            image = np.frombuffer(data, np.uint8).reshape(size, size, 3)
            net.setInput(cv2.dnn.blobFromImage(image, 1/255., (size, size), swapRB=True, crop=False))
            rows = decode(net.forward(), class_count, target_id, confidence, nms_iou)
            message = dict(rows=rows, inference_ms=(time.perf_counter()-start)*1000)
        except Exception as exc:
            message = dict(error=str(exc))
        reply.write(json.dumps(message)+'\n')
        reply.flush()


class LandingPadDetector:
    """Owns one warm child process for this mission worker; never restarts it.

    A failed or silent child leaves the detector empty, which the mission treats
    exactly like an image without an H pad.
    """
    def __init__(self, settings, popen=subprocess.Popen):
        self.settings, self.popen = validate_settings(settings), popen
        self.process, self.state, self.error, self.opencv = None, 'off', '', None
        self.inference_ms = None
        self._buffer = b''
        self.target_id = self.settings['class_names'].index(self.settings['target_class'])

    def start(self):
        s = self.settings
        if not s['enabled']:
            return self.status()
        python = sys.executable if s['python'] is None else str(resolve(s['python']))
        model = resolve(s['model'])
        if not Path(python).exists():
            return self._fail('YOLO 解释器不存在：%s' % python)
        if not model.is_file():
            return self._fail('H 标模型不存在：%s' % model)
        command = [python, str(Path(__file__).resolve()), '--serve', str(model),
                   '--size', str(s['input_size']), '--classes', str(len(s['class_names'])),
                   '--target', str(self.target_id), '--confidence', str(s['confidence']),
                   '--nms', str(s['nms_iou']), '--threads', str(s['opencv_threads'])]
        env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
        try:
            self.process = self.popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        except OSError as exc:
            return self._fail('无法启动 H 标识别进程：%s' % exc)
        self.state = 'loading'
        return self.status()

    def _line(self, timeout):
        # Own buffer over the raw descriptor: a buffered readline() could hold
        # a second reply that select() would then never report.
        descriptor = self.process.stdout.fileno()
        deadline = time.monotonic()+timeout
        while b'\n' not in self._buffer:
            if not select.select([descriptor], [], [], max(0., deadline-time.monotonic()))[0]:
                return None
            chunk = os.read(descriptor, 65536)
            if not chunk:
                raise EOFError('H 标识别进程已退出')
            self._buffer += chunk
        line, self._buffer = self._buffer.split(b'\n', 1)
        return json.loads(line.decode('utf-8'))

    def _fail(self, message):
        self.state, self.error = 'error', str(message)
        self.close()
        return self.status()

    def detect(self, image):
        """Source-pixel landing-pad candidates; empty while loading or failed."""
        try:
            if self.state == 'loading':
                reply = self._line(0.)
                if reply is None:
                    return []
                if not reply.get('ready'):
                    self._fail(reply.get('error', 'H 标识别进程启动失败'))
                    return []
                self.state, self.opencv = 'ready', reply.get('opencv')
            if self.state != 'ready':
                return []
            size = self.settings['input_size']
            canvas, transform = letterbox(image, size)
            self.process.stdin.write(canvas.tobytes())
            self.process.stdin.flush()
            reply = self._line(float(self.settings['reply_timeout_s']))
        except (OSError, ValueError, EOFError) as exc:
            self._fail(exc)
            return []
        if reply is None:
            self._fail('H 标识别 %.1fs 内无回复' % self.settings['reply_timeout_s'])
            return []
        if reply.get('error'):
            self.error = reply['error']
            return []
        self.error, self.inference_ms = '', reply.get('inference_ms')
        return to_candidates(reply['rows'], transform, self.settings['target_class'])

    def close(self):
        process, self.process = self.process, None
        if process is None:
            return
        try:
            process.stdin.close()
            process.wait(timeout=1.)
        except Exception:
            process.kill()
        finally:
            process.stdout.close()

    def status(self):
        return dict(enabled=self.settings['enabled'], state=self.state, error=self.error,
                    opencv=self.opencv, inference_ms=self.inference_ms)


def main(argv=None):
    parser = argparse.ArgumentParser(description='H 标 YOLO 子进程；由红点任务观察进程启动。')
    parser.add_argument('--serve', required=True, help='ONNX 模型路径')
    parser.add_argument('--size', type=int, required=True)
    parser.add_argument('--classes', type=int, required=True)
    parser.add_argument('--target', type=int, required=True)
    parser.add_argument('--confidence', type=float, required=True)
    parser.add_argument('--nms', type=float, required=True)
    parser.add_argument('--threads', type=int, default=1)
    args = parser.parse_args(argv)
    return serve(args.serve, args.size, args.classes, args.target, args.confidence, args.nms, args.threads)


if __name__ == '__main__':
    sys.exit(main())
