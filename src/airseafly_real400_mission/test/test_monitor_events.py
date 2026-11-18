"""Terminal-only tests: temporary logs and fake RPC, no ROS or actuator calls."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from airseafly_real400_mission.monitor_events import EventMonitor
from airseafly_real400_mission.telemetry import EventBuffer, event_line

spec = importlib.util.spec_from_file_location('monitor_test_cli', PACKAGE/'scripts/mission_cli.py')
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


def point(seq, boot='boot', source='loaded'):
    return dict(seq=seq, boot_id=boot, event='effect',
                effect=dict(type='delivery_route_point', lap=2, index=seq-1, route_source=source))


class MonitorEventsTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)
        self.path = self.directory/'events.jsonl'
        self.path.touch()
        self.emitted, self.warnings = [], []
        self.monitor = EventMonitor(emit=self.emitted.append, warn=self.warnings.append)
        self.monitor.update_status(dict(boot_id='boot', log_dir=str(self.directory)))

    def journal(self, rows):
        with self.path.open('a') as stream:
            for row in rows:
                stream.write(json.dumps(row)+'\n')

    def test_large_batch_and_overlapping_rpc_are_displayed_once(self):
        for count in (161, 1001):
            with self.subTest(count=count):
                received = []
                monitor = EventMonitor(emit=received.append)
                rows = [point(seq) for seq in range(1, count+1)]
                packet = dict(rows[-1], recent_events=rows)
                monitor.consume(packet)
                monitor.consume(packet)
                monitor.consume(dict(boot_id='boot', last_seq=count, events=rows[-256:]))
                self.assertEqual([r['seq'] for r in received], list(range(1, count+1)))
                self.assertEqual(monitor.after(), count)
                self.assertEqual(received, rows)  # No embedded history on the last item.

    def test_journal_preserves_full_record_when_rpc_state_is_compact(self):
        full = dict(event='state_changed', seq=1, boot_id='boot',
                    output=dict(state='DROP', phase='ALIGN', extra='full log detail'))
        self.journal([full])
        compact = dict(full, output=dict(state='DROP'))
        self.monitor.consume(dict(boot_id='boot', events=[compact], last_seq=1))
        self.assertEqual(self.emitted, [full])

    def test_late_join_recovers_cache_overflow_and_receipts_from_read_only_log(self):
        buffer = EventBuffer()
        rows = [buffer.append(dict(event='effect', effect=point(n)['effect']))
                for n in range(1, 1002)]
        rows += [buffer.append(dict(event='payload_result', slot=slot, request_id=slot,
                                    success=True, outcome='accepted')) for slot in range(1, 5)]
        self.journal([dict(event='runtime_sample')]+rows)
        digest = hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.monitor.update_status(dict(boot_id=buffer.boot_id, log_dir=str(self.directory),
                                        last_event_seq=buffer.seq, payload_records=rows[-4:]))
        response = buffer.read()
        self.assertFalse(response['gap'])  # after=0 does not flag overflow in the producer.
        self.assertEqual(len(response['events']), 256)
        self.monitor.consume(response)
        self.monitor.consume(response)
        self.assertEqual(self.emitted, rows)
        self.assertEqual(self.monitor.after(), 1005)
        self.assertEqual(self.warnings, [])
        self.assertEqual(digest, hashlib.sha256(self.path.read_bytes()).hexdigest())

    def test_resume_after_task_start_does_not_reprint_older_events(self):
        self.journal([point(seq) for seq in range(1, 401)])
        monitor = EventMonitor(after=200, emit=self.emitted.append)
        monitor.update_status(dict(boot_id='boot', log_dir=str(self.directory), last_event_seq=400))
        monitor.consume()
        self.assertEqual([r['seq'] for r in self.emitted], list(range(201, 401)))

    def test_partial_line_and_gap_are_recovered_without_skipping_or_duplicates(self):
        self.journal([point(1)])
        with self.path.open('a') as stream:
            stream.write(json.dumps(point(2)))
        response = dict(boot_id='boot', last_seq=3, events=[point(3)])
        self.monitor.consume(response)
        self.assertEqual(self.monitor.after(), 1)
        self.assertEqual([r['seq'] for r in self.emitted], [1, 3])
        self.monitor.consume(response)
        self.assertEqual(len(self.warnings), 1)
        with self.path.open('a') as stream:
            stream.write('\n')
        self.monitor.consume(response)
        self.assertEqual([r['seq'] for r in self.emitted], [1, 3, 2])
        self.assertEqual(self.monitor.after(), 3)
        self.assertIsNone(self.monitor.gap_start)

    def test_missing_log_keeps_receipts_and_reports_missing_points(self):
        self.path.unlink()
        receipt = dict(seq=5, boot_id='boot', event='payload_result', success=True, slot=4)
        self.monitor.update_status(dict(boot_id='boot', log_dir=str(self.directory),
                                        last_event_seq=5, payload_records=[receipt]))
        self.monitor.consume()
        self.monitor.consume()
        self.assertEqual(self.emitted, [receipt])
        self.assertEqual(self.monitor.after(), 0)
        self.assertEqual(len(self.warnings), 2)
        self.journal([point(seq) for seq in range(1, 5)]+[receipt])
        self.monitor.consume()
        self.assertEqual(len(self.emitted), 5)
        self.assertEqual(self.monitor.after(), 5)

    def test_log_replacement_and_boot_restart_allow_reused_sequences(self):
        self.journal([point(1)])
        self.monitor.consume()
        self.path.rename(self.directory/'old.jsonl')
        self.journal([point(1), point(2)])
        self.monitor.consume()
        self.assertEqual([r['seq'] for r in self.emitted], [1, 2])
        self.path.write_text('')
        self.journal([point(1, 'next')])
        self.monitor.update_status(dict(boot_id='next', log_dir=str(self.directory), last_event_seq=1))
        self.monitor.consume(dict(boot_id='boot', last_seq=3, events=[point(3)]))
        self.assertEqual([(r['boot_id'], r['seq']) for r in self.emitted],
                         [('boot', 1), ('boot', 2), ('next', 1)])
        self.assertEqual(self.monitor.after(), 1)
        self.assertEqual(self.monitor.poll_after(), 0)  # Old-boot RPC must not move the cursor.

    def test_cache_gap_does_not_turn_existing_long_poll_into_busy_poll(self):
        buffer = EventBuffer(limit=1)
        rows = [buffer.append(dict(event='effect', effect=point(n)['effect'])) for n in range(1, 6)]
        self.path.unlink()
        self.monitor.update_status(dict(boot_id=buffer.boot_id, log_dir=str(self.directory), last_event_seq=5))
        self.monitor.consume(buffer.read())
        self.assertEqual(self.monitor.after(), 0)  # Still recover the missing four records.
        self.assertEqual(self.monitor.poll_after(), 5)  # Wait for new RPC events, not the same tail.
        self.assertEqual(buffer.read(after=self.monitor.poll_after())['events'], [])
        self.journal(rows)
        self.monitor.consume(buffer.read(after=self.monitor.poll_after()))
        self.assertEqual(self.monitor.after(), 5)
        self.assertEqual(sorted(self.emitted, key=lambda item: item['seq']), rows)

    def test_bad_log_line_is_reported_and_rpc_fills_it(self):
        self.journal([point(1)])
        with self.path.open('a') as stream:
            stream.write('{bad line}\n')
        self.journal([point(3)])
        self.monitor.consume(dict(boot_id='boot', last_seq=3, events=[point(2)]))
        self.assertEqual([r['seq'] for r in self.emitted], [1, 2, 3])
        self.assertIn('无法解析', self.warnings[0])

    def test_concurrent_final_and_background_drains_do_not_duplicate(self):
        rows = [point(seq) for seq in range(1, 1002)]
        self.journal(rows)
        response = dict(boot_id='boot', last_seq=1001, events=rows[-256:])
        threads = [threading.Thread(target=self.monitor.consume, args=(response,)) for _ in range(4)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        self.assertEqual(self.emitted, rows)


class TerminalFormatTests(unittest.TestCase):
    def test_route_sources_and_planning_do_not_claim_arrival(self):
        for source, label in [('loaded', '绕圈路线'), ('delivery', '投放路线'), ('shared', '共用路线')]:
            line = event_line(point(3, source=source))
            self.assertIn('第2圈 '+label+'第3号点', line)
            self.assertIn('航点登记', line)
            self.assertNotIn('到位', line)
        event = point(3)
        event['effect'].update(type='delivery_route_reference_passed', closing=True)
        self.assertIn('参考通过', event_line(event))
        self.assertIn('闭环', event_line(event))
        event['effect'].update(type='delivery_route_point_arrived', stage='home')
        self.assertIn('实际到位：第2圈 HOME', event_line(event))

    def test_all_four_receipts_keep_request_and_route_point_identity(self):
        for slot, index in enumerate((0, 8, 12, 16), 1):
            result = dict(event='payload_result', slot=slot, request_id=100+slot,
                          success=True, outcome='accepted')
            self.assertIn('编号%d，请求%d，飞控接受' % (slot, 100+slot), event_line(result))
            ack = dict(event='effect', effect=dict(type='delivery_route_release_ack',
                       slot=slot, index=index, lap=9, route_source='delivery'))
            self.assertIn('第9圈 投放路线第%d号点，投放编号%d' % (index+1, slot), event_line(ack))
        result.update(success=False, outcome='rejected')
        self.assertIn('飞控拒绝', event_line(result))
        self.assertNotIn('成功', event_line(result))
        result.update(event='payload_late_result', success=True)
        self.assertIn('迟到回执（是否计入以任务记录为准）', event_line(result))


class WatchCompletionTests(unittest.TestCase):
    def run_completed(self, state, failed_rpc=False, json_output=True):
        with tempfile.TemporaryDirectory() as temp:
            rows = [point(seq) for seq in range(1, 402)]
            rows += [dict(seq=seq, boot_id='boot', event='payload_result', success=True,
                          slot=seq-401, request_id=seq) for seq in range(402, 406)]
            Path(temp, 'events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            status = dict(boot_id='boot', log_dir=temp, last_event_seq=405,
                          output=dict(state=state, done=state=='COMPLETE', drops_completed=4),
                          task_id='task', compute=dict(group='NONE'), payload_records=rows[-4:])
            calls = []
            def request(path, body, *args):
                calls.append(body)
                if body['command'] == 'status':return status
                self.assertEqual(body['command'], 'events')
                if failed_rpc:raise OSError('test transport unavailable')
                return dict(boot_id='boot', last_seq=405, events=rows[-256:])
            stream = io.StringIO()
            with mock.patch.object(cli, 'request', side_effect=request), contextlib.redirect_stdout(stream):
                code = cli.watch(dict(socket='fake'), SimpleNamespace(verbose=False, json=json_output),
                                 {}, finish_task=True, expected_task_id='task')
            self.assertEqual(code, 0 if state=='COMPLETE' else 1)
            self.assertEqual({call['command'] for call in calls}, {'status', 'events'})
            self.assertTrue(any(call.get('wait_s') == 0 for call in calls))
            if json_output:
                output = [json.loads(line) for line in stream.getvalue().splitlines()]
                self.assertEqual([r['data'] for r in output if r['type']=='event'], rows)
                self.assertEqual(output[-1]['type'], 'status')
            else:
                value = stream.getvalue()
                self.assertEqual(value.count('航点登记：'), 401)
                self.assertEqual(value.count('投放回执：'), 4)
                self.assertLess(value.rindex('投放回执：'), value.index('任务状态：'))

    def test_complete_drains_full_log_before_final_status(self):
        self.run_completed('COMPLETE')

    def test_hold_drains_receipts_without_sending_any_command(self):
        self.run_completed('HOLD')

    def test_final_rpc_failure_still_drains_journal(self):
        self.run_completed('COMPLETE', failed_rpc=True)

    def test_human_terminal_shows_points_and_receipts_before_summary(self):
        self.run_completed('COMPLETE', json_output=False)

    def test_keyboard_interrupt_keeps_existing_task_scoped_hold(self):
        calls = []
        def request(path, body, *args):
            calls.append(body)
            if body['command']=='status':raise KeyboardInterrupt()
            return dict(ok=True)
        with mock.patch.object(cli, 'request', side_effect=request), contextlib.redirect_stdout(io.StringIO()):
            result = cli.watch(dict(socket='fake'), SimpleNamespace(verbose=False, json=True), {},
                               finish_task=True, expected_task_id='task', expected_height_id='height')
        self.assertEqual(result, 0)
        self.assertEqual(calls, [dict(command='status'),
            dict(command='hold', task_id='task', manual_height_id='height')])


if __name__ == '__main__':
    unittest.main()
