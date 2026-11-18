"""Lossless route announcements with real manager logging and inert ROS."""
import io
import json
import threading
import unittest

from ros_fixtures import MANAGER, make_manager


class RouteEventBatchTests(unittest.TestCase):
    def logger(self):
        manager = make_manager()
        manager.events = io.StringIO()
        manager.log_lock = threading.Lock()
        manager.event = MANAGER.Manager.event.__get__(manager)
        return manager

    def test_every_point_is_logged_and_published_beyond_history_capacity(self):
        for count in (1, 161, 1001):
            with self.subTest(count=count):
                manager = self.logger()
                batch = []
                for index in range(count):
                    effect = dict(type='delivery_route_point', index=index, route_source='loaded')
                    manager.event('effect', effect=effect, _batch=batch)
                    effect['index'] = -1
                # Records reach the log immediately, without a worker/flush wait.
                records = [json.loads(line) for line in manager.events.getvalue().splitlines()]
                self.assertEqual([r['effect']['index'] for r in records], list(range(count)))
                self.assertEqual([r['seq'] for r in records], list(range(1, count+1)))
                self.assertEqual(manager.event_pub.messages, [])
                manager.publish_point_events(batch)
                self.assertEqual(len(manager.event_pub.messages), 1)
                packet = json.loads(manager.event_pub.messages[0].data)
                self.assertEqual(packet['seq'], count)
                self.assertEqual([r['effect']['index'] for r in packet['recent_events']], list(range(count)))
                self.assertEqual([r['seq'] for r in packet['recent_events']], list(range(1, count+1)))
                self.assertFalse(any('_batch' in r for r in records))

    def test_payload_result_is_immediate_and_separate_from_point_batch(self):
        manager = self.logger()
        batch = []
        manager.event('effect', effect=dict(type='delivery_route_point', index=0), _batch=batch)
        manager.event('payload_result', task_id=manager.task_id, for_mission=True,
                      request_id='real-receipt', success=True, outcome='accepted', slot=4)
        self.assertEqual(len(manager.payload_records), 1)
        self.assertEqual(manager.payload_records[0]['request_id'], 'real-receipt')
        self.assertEqual(len(manager.event_pub.messages), 1)
        self.assertEqual(json.loads(manager.event_pub.messages[0].data)['event'], 'payload_result')
        manager.event('effect', effect=dict(type='delivery_route_point', index=1), _batch=batch)
        manager.publish_point_events(batch)
        records = [json.loads(line) for line in manager.events.getvalue().splitlines()]
        self.assertEqual([r['seq'] for r in records], [1, 2, 3])
        self.assertEqual([r['seq'] for r in json.loads(manager.event_pub.messages[-1].data)['recent_events']], [1, 2, 3])
        self.assertEqual(len(manager.payload_records), 1)
        self.assertEqual(records[1]['request_id'], 'real-receipt')

    def test_batch_keeps_recent_history_and_empty_batch_does_nothing(self):
        manager = self.logger()
        manager.event('previous_task_event', value=5)
        batch = []
        manager.event('effect', effect=dict(type='delivery_route_point', index=0), _batch=batch)
        manager.publish_point_events(batch)
        packet = json.loads(manager.event_pub.messages[-1].data)
        self.assertEqual([r['seq'] for r in packet['recent_events']], [1, 2])
        before = len(manager.event_pub.messages)
        manager.publish_point_events([])
        self.assertEqual(len(manager.event_pub.messages), before)


if __name__ == '__main__':
    unittest.main()
