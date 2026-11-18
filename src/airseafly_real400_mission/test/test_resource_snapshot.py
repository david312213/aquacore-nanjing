"""Portable concurrency checks: diagnostics must not delay control or cancel."""
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from airseafly_real400_mission import resources


def bare_runner():
    runner = resources.ComputeRunner.__new__(resources.ComputeRunner)
    runner.condition = threading.Condition()
    runner.jobs = {}
    runner.wanted, runner.error = 'EGO', None
    runner.generations = dict(EGO=1, FRONT=0, DOWN=0)
    return runner


def job(pid=123, session='first', generation=1):
    return dict(process=SimpleNamespace(pid=pid), session=session, peak_rss=0,
                generation=generation)


class ResourceSnapshotTests(unittest.TestCase):
    def test_status_and_cancel_complete_while_process_scan_is_blocked(self):
        runner = bare_runner()
        runner.jobs['EGO'] = job()
        scanning, release, completed = threading.Event(), threading.Event(), threading.Event()

        def scan():
            scanning.set()
            if not release.wait(2):
                raise AssertionError('test did not release process scan')
            return [dict(pid=123, rss_bytes=1024, cpu_ticks=10)]

        with mock.patch.object(resources, 'track_process', return_value=SimpleNamespace(snapshot=scan)):
            sampler = threading.Thread(target=runner._sample_jobs)
            sampler.start()
            self.assertTrue(scanning.wait(1))
            observed = []
            def read_and_cancel():
                observed.append(runner.snapshot())
                runner.request('NONE')
                completed.set()
            reader = threading.Thread(target=read_and_cancel)
            reader.start()
            try:
                self.assertTrue(completed.wait(.2), 'diagnostics/cancel waited for /proc')
                self.assertEqual(observed[0]['session_id'], 'first')
                self.assertEqual(runner.snapshot()['wanted'], 'NONE')
            finally:
                release.set()
                sampler.join(2)
                reader.join(2)
            self.assertFalse(sampler.is_alive())
            self.assertEqual(runner.snapshot()['rss_bytes'], 1024)

    def test_snapshot_is_a_detached_copy_and_does_not_scan(self):
        runner = bare_runner()
        current = job()
        current.update(metrics=dict(pids=[123, 124], rss_bytes=1024, cpu_seconds=.1), peak_rss=2048)
        runner.jobs['EGO'] = current
        with mock.patch.object(resources, 'track_process', side_effect=AssertionError('unexpected scan')):
            snapshot = runner.snapshot()
            snapshot['pids'].append(987)
            snapshot['workers']['EGO']['pids'].append(456)
            snapshot['workers']['EGO']['rss_bytes'] = 0
            latest = runner.snapshot()
        self.assertEqual(latest['pids'], [123, 124])
        self.assertEqual(latest['workers']['EGO']['pids'], [123, 124])
        self.assertEqual(latest['rss_bytes'], 1024)
        self.assertEqual(latest['peak_rss_bytes'], 2048)

    def test_old_scan_cannot_update_replacement_job(self):
        runner = bare_runner()
        previous = runner.jobs['EGO'] = job()
        replacement = job(456, 'second', 2)
        def scan():
            with runner.condition:
                runner.jobs['EGO'] = replacement
            return [dict(pid=123, rss_bytes=9999, cpu_ticks=20)]
        with mock.patch.object(resources, 'track_process', return_value=SimpleNamespace(snapshot=scan)):
            runner._sample_jobs()
        result = runner.snapshot()
        self.assertEqual(result['session_id'], 'second')
        self.assertEqual(result['pids'], [456])
        self.assertEqual(result['rss_bytes'], 0)
        self.assertNotIn('metrics', previous)
        self.assertNotIn('metrics', replacement)

    def test_worker_exit_still_stops_tree_and_removes_cached_metrics(self):
        runner = bare_runner()
        current = job()
        current.update(started_at=time.monotonic(), metrics=dict(pids=[123,124], rss_bytes=1024, cpu_seconds=.1))
        runner.jobs['EGO'] = current
        events = []
        runner.event = lambda name, **data: events.append(name)
        with mock.patch.object(resources, 'stop_process_group') as stop:
            runner._stop('EGO')
        stop.assert_called_once_with(current['process'])
        self.assertEqual(events, ['compute_stopping', 'compute_exited'])
        self.assertEqual(runner.snapshot()['group'], 'NONE')
        self.assertEqual(runner.snapshot()['pids'], [])

    def test_lifecycle_samples_and_cleans_up_without_holding_condition(self):
        class Process:
            pid = 123
            returncode = None
            def poll(self):
                return self.returncode

        process = Process()
        started, sampled = threading.Event(), threading.Event()
        errors, scan_threads = [], []
        runner_box = []
        def scan():
            runner = runner_box[0]
            result = []
            def acquire():
                with runner.condition:
                    result.append(True)
            competitor = threading.Thread(target=acquire)
            competitor.start()
            competitor.join(.2)
            if not result:
                raise AssertionError('lifecycle sampled while holding condition')
            scan_threads.append(threading.current_thread().name)
            sampled.set()
            return [dict(pid=123, rss_bytes=2048, cpu_ticks=10)]

        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(resources.subprocess, 'Popen', return_value=process), \
                mock.patch.object(resources, 'track_process', return_value=SimpleNamespace(snapshot=scan)), \
                mock.patch.object(resources, 'stop_process_group') as stop:
            runner = resources.ComputeRunner(lambda group, session: ['fake'], Path(directory),
                lambda group, session: started.set(), errors.append, lambda *args, **kwargs: None)
            runner_box.append(runner)
            try:
                runner.request('EGO')
                self.assertTrue(started.wait(1))
                self.assertTrue(sampled.wait(1))
                deadline = time.monotonic()+1
                while runner.snapshot()['rss_bytes'] != 2048 and time.monotonic() < deadline:
                    time.sleep(.005)
                self.assertEqual(runner.snapshot()['rss_bytes'], 2048)
            finally:
                runner.close()
            stop.assert_called_once_with(process)
            self.assertEqual(runner.snapshot()['group'], 'NONE')
            self.assertEqual(set(scan_threads), {'compute-lifecycle'})
            self.assertFalse(runner.thread.is_alive())
            self.assertEqual(errors, [])


if __name__ == '__main__':
    unittest.main()
