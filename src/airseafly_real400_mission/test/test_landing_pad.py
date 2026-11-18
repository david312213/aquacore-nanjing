"""H-pad YOLO decode, child protocol and worker switching. No ROS or camera."""
import io
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission import landing_pad as lp
from airseafly_real400_mission import red_point_runtime as runtime
from airseafly_real400_mission import red_point_worker as worker
from airseafly_real400_mission.red_point import annotate

SESSION = '052594d7-c3a8-4a42-80f9-2f759dbab62c'


def raw_output(boxes, classes=6):
    """boxes: (cx, cy, w, h, class_id, score) in model pixels."""
    output = np.zeros((1, 4+classes, 8400), np.float32)
    for index, (cx, cy, w, h, class_id, score) in enumerate(boxes):
        output[0, :4, index] = cx, cy, w, h
        output[0, 4+class_id, index] = score
    return output


class FakeProcess:
    """Real pipe for stdout so select() behaves like the child."""
    def __init__(self, replies=()):
        read, self.write_fd = os.pipe()
        self.stdout = os.fdopen(read, 'rb')
        self.stdin = io.BytesIO()
        self.stdin.close = lambda: None
        self.killed = False
        for reply in replies:
            self.reply(reply)

    def reply(self, value):
        os.write(self.write_fd, (json.dumps(value)+'\n').encode())

    def eof(self):
        os.close(self.write_fd)

    def wait(self, timeout=None):
        return 0

    def kill(self):
        self.killed = True


def detector(process, **settings):
    calls = []

    def popen(command, **kwargs):
        calls.append(command)
        return process
    # Any existing file stands in for the model: the fake child never reads it.
    settings = dict(dict(enabled=True, python=sys.executable, model=__file__, reply_timeout_s=.2), **settings)
    result = lp.LandingPadDetector(settings, popen=popen)
    result.calls = calls
    return result


class DecodeTests(unittest.TestCase):
    def test_decode_keeps_target_class_above_threshold_and_applies_nms(self):
        output = raw_output([(320, 320, 100, 80, 5, .9), (322, 321, 100, 80, 5, .8),
                             (100, 100, 40, 40, 5, .44), (500, 500, 60, 60, 4, .95),
                             (200, 400, 50, 50, 5, .6)])
        rows = lp.decode(output, 6, 5, .45, .70)
        self.assertEqual([round(r[4], 2) for r in rows], [.9, .6])
        self.assertEqual([round(v) for v in rows[0][:4]], [270, 280, 370, 360])
        self.assertTrue(all(r[5] == 5 for r in rows))
        self.assertEqual(lp.decode(raw_output([]), 6, 5, .45, .7), [])
        with self.assertRaises(ValueError):
            lp.decode(np.zeros((1, 9, 10)), 6, 5, .45, .7)

    def test_letterbox_and_box_mapping_round_trip_to_source_pixels(self):
        image = np.zeros((720, 1280, 3), np.uint8)
        canvas, transform = lp.letterbox(image, 640)
        self.assertEqual(canvas.shape, (640, 640, 3))
        self.assertEqual((transform['left'], transform['top']), (0, 140))
        self.assertTrue((canvas[:140] == 114).all())
        # Source box (600..700, 300..400) -> model pixels at scale 0.5, top pad 140.
        rows = [[300., 290., 350., 340., .8, 5]]
        candidate = lp.to_candidates(rows, transform, 'landing_pad')[0]
        self.assertEqual(candidate['pixel'], [650., 350.])
        self.assertEqual(candidate['bbox_xywh'], [600, 300, 100, 100])
        self.assertEqual((candidate['class_name'], candidate['target_confirmed']), ('landing_pad', True))
        clipped = lp.to_candidates([[-20., 100., 40., 160., .8, 5], [0., 10., 50., 130., .8, 5]],
                                   transform, 'landing_pad')
        self.assertEqual(clipped[0]['bbox_xywh'][0], 0)
        self.assertEqual(len(clipped), 1)  # The second box lies entirely in the padding.

    def test_settings_validation(self):
        self.assertFalse(lp.validate_settings(None)['enabled'])
        for bad in (dict(enabled=1), dict(target_class='helipad'), dict(input_size=630),
                    dict(confidence=0), dict(nms_iou=1.5), dict(opencv_threads=0),
                    dict(reply_timeout_s=-1), dict(class_names=['a', 'a']), dict(model=3)):
            with self.assertRaises(ValueError):
                lp.validate_settings(bad)


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((720, 1280, 3), np.uint8)

    def test_loading_then_ready_then_reply_maps_back_to_source(self):
        process = FakeProcess()
        d = detector(process)
        self.assertEqual(d.start()['state'], 'loading')
        self.assertIn('--serve', d.calls[0])
        self.assertEqual(d.detect(self.image), [])  # Still loading: no frame sent.
        self.assertEqual(process.stdin.getvalue(), b'')
        process.reply(dict(ready=True, opencv='4.13.0'))
        process.reply(dict(rows=[[300., 290., 350., 340., .8, 5]], inference_ms=12.))
        candidates = d.detect(self.image)
        self.assertEqual(len(process.stdin.getvalue()), 640*640*3)
        self.assertEqual(candidates[0]['pixel'], [650., 350.])
        self.assertEqual(d.status(), dict(enabled=True, state='ready', error='', opencv='4.13.0', inference_ms=12.))

    def test_frame_error_keeps_child_but_timeout_eof_and_load_failure_stop_it(self):
        process = FakeProcess([dict(ready=True), dict(error='bad frame')])
        d = detector(process); d.start()
        self.assertEqual(d.detect(self.image), [])
        self.assertEqual((d.state, d.error), ('ready', 'bad frame'))
        self.assertEqual(d.detect(self.image), [])  # No reply within 0.2 s.
        self.assertEqual(d.state, 'error')
        self.assertIn('无回复', d.error)
        self.assertIsNone(d.process)
        self.assertEqual(d.detect(self.image), [])  # Never restarted.
        self.assertEqual(len(d.calls), 1)
        process = FakeProcess([dict(ready=True)])
        process.eof()
        d = detector(process); d.start(); d.detect(self.image)
        self.assertEqual(d.state, 'error')
        d = detector(FakeProcess([dict(error='模型加载失败：x')])); d.start(); d.detect(self.image)
        self.assertEqual((d.state, d.error), ('error', '模型加载失败：x'))

    def test_missing_interpreter_model_or_disabled_never_spawns(self):
        for settings, state in ((dict(python='/missing/python'), 'error'),
                                (dict(model='/missing/model.onnx'), 'error'),
                                (dict(enabled=False), 'off')):
            d = detector(FakeProcess(), **settings)
            self.assertEqual(d.start()['state'], state)
            self.assertEqual(d.calls, [])
            self.assertEqual(d.detect(self.image), [])

    def test_real_child_loads_the_1016_model(self):
        if not lp.resolve(lp.DEFAULTS['model']).is_file():
            self.skipTest('1016 model not present in this checkout')
        python = lp.resolve(lp.DEFAULTS['python'])
        python = str(python) if python.exists() else sys.executable
        version = subprocess.run([python, '-c', 'import cv2;print(cv2.__version__)'],
                                 capture_output=True, text=True).stdout.strip()
        if not version or tuple(int(v) for v in version.split('.')[:2]) < (4, 7):
            self.skipTest('no OpenCV >= 4.7 interpreter for YOLO11 (found %r)' % version)
        d = lp.LandingPadDetector(dict(enabled=True, python=python, reply_timeout_s=10.))
        self.addCleanup(d.close)
        d.start()
        for _ in range(200):
            result = d.detect(self.image)
            if d.state != 'loading':
                break
            __import__('time').sleep(.05)
        self.assertEqual((d.state, d.error, result), ('ready', '', []))
        self.assertEqual(d.opencv, version)
        self.assertIsNotNone(d.inference_ms)


class WorkerSwitchTests(unittest.TestCase):
    def test_worker_runs_red_detector_or_h_detector_by_mission_request(self):
        config = runtime.load_config(runtime.DEFAULT_CONFIG)
        self.assertTrue(config['landing_pad']['enabled'])

        class Landing:
            def detect(self, image):
                return [dict(class_name='landing_pad', pixel=[640., 360.], bbox_xywh=[540, 260, 200, 200],
                             score=.8, target_confirmed=True, confirmation_reason='yolo')]

            def status(self):
                return dict(enabled=True, state='ready', error='')
        observations = worker.MissionObservations(config, SESSION, Landing())
        observations.inputs.on_odometry(NS(header=NS(stamp=NS(to_sec=lambda: 1.), seq=1, frame_id='fast_lio_local_enu'),
            pose=NS(pose=NS(position=NS(x=0., y=0., z=1.), orientation=NS(x=0., y=0., z=0., w=1.)))))
        bridge = NS(imgmsg_to_cv2=lambda message, **kw: message.pixels)

        def frame(seq):
            return NS(header=NS(stamp=NS(to_sec=lambda: 1.), seq=seq, frame_id='down'), width=1280, height=720,
                      pixels=np.full((720, 1280, 3), 230, np.uint8))
        image, status = observations.process(frame(1), bridge, 'reused', 10.)
        self.assertEqual(status['landing_count'], 1)
        self.assertEqual(status['candidates'], [])
        h = status['landing_candidates'][0]
        self.assertEqual(h['projection_status'], 'ok_near_level')
        self.assertEqual(len(h['xy_m']), 2)
        self.assertEqual(status['landing_detector']['state'], 'ready')
        self.assertEqual(annotate(image, status).shape, image.shape)
        # A red-point window has no H detector: original status, no landing_detector key.
        red = worker.MissionObservations(config, SESSION)
        _, status = red.process(frame(2), bridge, 'reused', 10.)
        self.assertEqual(status['landing_count'], 0)
        self.assertNotIn('landing_detector', status)

    def test_target_argument_reaches_live_and_defaults_to_red_point(self):
        config = runtime.DEFAULT_CONFIG
        calls = []
        with mock.patch.object(worker, 'live', side_effect=lambda c, s, t: calls.append(t) or 0):
            self.assertEqual(worker.main(['--config', str(config), '--session', SESSION]), 0)
            self.assertEqual(worker.main(['--config', str(config), '--session', SESSION,
                                          '--target', 'landing_pad']), 0)
        self.assertEqual(calls, ['red_point', 'landing_pad'])
        with self.assertRaises(SystemExit):
            worker.main(['--config', str(config), '--session', SESSION, '--target', 'helipad'])


if __name__ == '__main__':
    unittest.main()
