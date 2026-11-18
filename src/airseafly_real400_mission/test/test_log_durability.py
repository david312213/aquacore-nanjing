"""Offline durability checks; no ROS endpoints or flight commands are started."""
import json
import os
from pathlib import Path
import queue
import stat
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from airseafly_real400_mission.log_durability import DurableLogSync, LOG_FILES
from airseafly_real400_mission.alignment_trace import AlignmentTrace, TraceBuffer


def wait_until(predicate, timeout=2.):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        if predicate():
            return True
        threading.Event().wait(.005)
    return bool(predicate())


class LogDurabilityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name) / 'mission'
        self.directory.mkdir()

    def start(self, **kwargs):
        worker = DurableLogSync(self.directory, **kwargs)
        self.addCleanup(worker.close)
        return worker

    def test_shutdown_syncs_all_existing_log_files_without_changing_contents(self):
        contents = {}
        for name in LOG_FILES:
            contents[name] = '{"sample": 1}\n'
            (self.directory / name).write_text(contents[name])
        worker = self.start(interval_s=60.)
        self.assertTrue(worker.close())
        self.assertEqual(worker.status()['synced_files'], len(LOG_FILES))
        self.assertEqual(worker.status()['error_count'], 0)
        self.assertIsNotNone(worker.status()['last_sync_wall_time'])
        self.assertFalse(worker.status()['running'])
        for name, expected in contents.items():
            self.assertEqual((self.directory / name).read_text(), expected)

    def test_blocked_background_sync_does_not_block_manager_event_write(self):
        from ros_fixtures import MANAGER, make_manager
        manager = make_manager()
        path = self.directory / 'events.jsonl'
        manager.events = path.open('a', buffering=1)
        self.addCleanup(manager.events.close)
        manager.events.write('{"initial":true}\n')
        entered, release = threading.Event(), threading.Event()
        sync_threads = []

        def blocked_sync(fd):
            sync_threads.append(threading.current_thread().name)
            if stat.S_ISREG(os.fstat(fd).st_mode):
                entered.set()
                release.wait(2.)

        with patch('airseafly_real400_mission.log_durability.os.fsync', blocked_sync):
            worker = self.start(interval_s=.01)
            try:
                self.assertTrue(entered.wait(2.))
                # The real manager event path keeps writing while fsync is
                # deliberately blocked on the same file in a separate thread.
                MANAGER.Manager.event(manager, 'offline_event', value=42)
                self.assertFalse(release.is_set())
                record = json.loads(path.read_text().splitlines()[-1])
                self.assertEqual((record['event'], record['value']), ('offline_event', 42))
                self.assertTrue(all(name == 'mission-log-sync' for name in sync_threads))
            finally:
                release.set()
                worker.close()

    def test_periodic_sync_skips_unchanged_files_and_detects_new_lines(self):
        path = self.directory / 'events.jsonl'
        path.write_text('first\n')
        calls = []

        def record_sync(fd):
            info = os.fstat(fd)
            if stat.S_ISREG(info.st_mode):
                calls.append(info.st_size)

        with patch('airseafly_real400_mission.log_durability.os.fsync', record_sync):
            worker = self.start(interval_s=.01)
            self.assertTrue(wait_until(lambda: len(calls) == 1))
            threading.Event().wait(.05)
            self.assertEqual(calls, [6])
            with path.open('a', buffering=1) as stream:
                stream.write('second\n')
            self.assertTrue(wait_until(lambda: len(calls) == 2))
            worker.close()
        self.assertEqual(calls, [6, 13])

    def test_atomic_status_replacement_is_synced_even_at_same_size(self):
        path = self.directory / 'status.json'
        path.write_text('{"active":0}')
        inodes = []

        def record_sync(fd):
            info = os.fstat(fd)
            if stat.S_ISREG(info.st_mode):
                inodes.append(info.st_ino)

        with patch('airseafly_real400_mission.log_durability.os.fsync', record_sync):
            worker = self.start(interval_s=.01)
            self.assertTrue(wait_until(lambda: len(inodes) == 1))
            temporary = path.with_suffix('.json.tmp')
            temporary.write_text('{"active":1}')
            temporary.replace(path)
            self.assertTrue(wait_until(lambda: len(inodes) == 2))
            worker.close()
        self.assertNotEqual(inodes[0], inodes[1])

    def test_sync_error_is_reported_and_later_retry_recovers(self):
        path = self.directory / 'events.jsonl'
        path.write_text('first\n')
        fail = threading.Event()
        fail.set()

        def unreliable_sync(fd):
            if stat.S_ISREG(os.fstat(fd).st_mode) and fail.is_set():
                raise OSError('simulated storage failure')

        with patch('airseafly_real400_mission.log_durability.os.fsync', unreliable_sync):
            worker = self.start(interval_s=.01)
            self.assertTrue(wait_until(lambda: worker.status()['error_count'] > 0))
            self.assertIn('simulated storage failure', worker.status()['last_error'])
            with path.open('a', buffering=1) as stream:
                stream.write('task continues\n')
            fail.clear()
            self.assertTrue(wait_until(lambda: worker.status()['synced_files'] == 1))
            worker.close()
        self.assertIsNone(worker.status()['last_error'])
        self.assertGreater(worker.status()['error_count'], 0)
        self.assertEqual(path.read_text(), 'first\ntask continues\n')

    def test_close_requests_a_final_sync_without_waiting_for_period(self):
        worker = self.start(interval_s=60.)
        (self.directory / 'native_trajectory.jsonl').write_text('{"final":true}\n')
        self.assertTrue(worker.close(timeout=2.))
        self.assertEqual(worker.status()['synced_files'], 1)

    def test_missing_optional_traces_are_not_storage_errors(self):
        worker = self.start(interval_s=60.)
        self.assertTrue(worker.close())
        self.assertEqual(worker.status()['synced_files'], 0)
        self.assertEqual(worker.status()['error_count'], 0)


class LocalizationTraceTests(unittest.TestCase):
    def setUp(self):
        self.trace = AlignmentTrace.__new__(AlignmentTrace)
        self.now = 100.
        self.trace.rospy = NS(get_time=lambda: self.now)
        self.trace.lock, self.trace.buffer = threading.Lock(), TraceBuffer()
        self.trace.diagnostic_period_s, self.trace.diagnostic_last_sample = .5, {}
        self.trace.queue, self.trace.closed = queue.Queue(maxsize=256), False
        self.trace.dropped, self.trace.error = 0, None
        self.trace.pd_stream, self.trace.pd_samples = None, 0

    def command(self):
        return NS(position=NS(x=1., y=2., z=3.), velocity=NS(x=0., y=0., z=0.),
                  yaw=0., type_mask=2496, header=NS(stamp=NS(to_sec=lambda: self.now)))

    def test_localization_statuses_join_existing_trace_with_source_and_receipt_times(self):
        predictor = dict(last_correction_stamp_s=98.5, latest_imu_stamp_s=99.9,
                         last_output_stamp_s=99.2, last_withheld='correction_expired')
        localization = dict(stamp=99.95, healthy=False, sample_age_s=.75,
                            input_count=100, accepted_input_count=98, output_count=90)
        self.trace.prediction_status(NS(data=json.dumps(predictor)))
        self.trace.localization_status(NS(data=json.dumps(localization)))
        diagnostics = [self.trace.queue.get_nowait(), self.trace.queue.get_nowait()]
        self.assertEqual([row['event'] for row in diagnostics],
                         ['lio_prediction', 'native_localization'])
        self.assertTrue(self.trace.queue.empty())
        self.trace.native_command(self.command())
        packet = self.trace.queue.get_nowait()
        self.assertEqual(packet['feedback']['lio_prediction'],
                         dict(values=predictor, stamp_s=None, received_s=100.))
        self.assertEqual(packet['feedback']['native_localization'],
                         dict(values=localization, stamp_s=99.95, received_s=100.))
        # A second status does not bypass the unchanged 20 Hz trace sampling.
        self.now += .01
        self.trace.localization_status(NS(data=json.dumps(dict(localization, stamp=100.01))))
        self.trace.native_command(self.command())
        self.assertTrue(self.trace.queue.empty())

    def test_bad_status_does_not_replace_last_parseable_diagnostic(self):
        self.trace.prediction_status(NS(data='{"published": 10}'))
        for bad in ('not json', '[]', 'null'):
            self.trace.prediction_status(NS(data=bad))
        self.assertEqual(self.trace.buffer.latest['lio_prediction']['values'], dict(published=10))
        self.assertEqual(self.trace.queue.get_nowait()['event'], 'lio_prediction')
        self.assertTrue(self.trace.queue.empty())

    def test_passive_diagnostics_are_written_in_existing_alignment_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'alignment_control.jsonl'
            self.trace.stream = path.open('a', buffering=1)
            self.trace.prediction_status(NS(data='{"last_withheld":"correction_expired"}'))
            self.trace.localization_status(NS(data='{"stamp":100,"healthy":false}'))
            self.trace.native_command(self.command())
            self.trace.closed = True
            self.trace._write()
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(records), 3)
            record = records[-1]
            self.assertNotIn('record_type', record)  # Existing command shape is unchanged.
            self.assertEqual(record['feedback']['lio_prediction']['values']['last_withheld'],
                             'correction_expired')
            self.assertFalse(record['feedback']['native_localization']['values']['healthy'])

    def test_independent_diagnostics_continue_without_any_command(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'alignment_control.jsonl'
            self.trace.stream = path.open('a', buffering=1)
            for offset in (0., .1, .2, .49, .5):
                self.now = 100.+offset
                self.trace.prediction_status(NS(data=json.dumps(dict(
                    last_withheld='correction_expired', latest_imu_stamp_s=self.now))))
                self.trace.localization_status(NS(data=json.dumps(dict(
                    stamp=self.now, healthy=offset == .5))))
            self.trace.closed = True
            self.trace._write()
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(records), 4)  # Each diagnostic source stays at <=2 Hz.
            for name in ('lio_prediction', 'native_localization'):
                matching = [row for row in records if row['event'] == name]
                self.assertEqual([row['sampled_ros_time'] for row in matching], [100., 100.5])
                self.assertTrue(all(row['record_type'] == 'diagnostic' for row in matching))
                self.assertTrue(all('command' not in row for row in matching))
            recovery = records[-1]['diagnostic']
            self.assertTrue(recovery['values']['healthy'])
            self.assertEqual((recovery['stamp_s'], recovery['received_s']), (100.5, 100.5))


if __name__ == '__main__':
    unittest.main()
