"""Read-only flight status and bounded RPC event history."""
from collections import deque
from copy import deepcopy
import math
import threading
import uuid


def settings(config):
    values = dict(terminal_hz=1., status_hz=5.)
    values.update(config.get('visualization', {}))
    return values


def present_engine(engine):
    now, start, end = engine._now, engine.started_at, engine.ended_at
    origin, goal = engine.origin_xyz, engine.command_position
    error = [b-a for a, b in zip(engine.position, goal)]
    elapsed = lambda value: max(0., now-value) if value is not None else 0.
    return dict(state_name=engine.state, phase_name=engine.phase, reason_text=engine.reason,
        origin_xyz=list(origin) if origin is not None else None,
        relative_height_m=engine.position[2]-origin[2] if origin is not None else None,
        navigation_goal_xyz=list(goal) if engine.state == 'APPROACH' else None,
        navigation_error_xyz=error if engine.state == 'APPROACH' else None,
        horizontal_distance_m=math.hypot(*error[:2]), distance_3d_m=math.sqrt(sum(v*v for v in error)),
        height_target_z=goal[2], height_error_m=error[2], actual_yaw_rad=engine.yaw,
        target_yaw_rad=engine.command_yaw, yaw_error_rad=math.atan2(math.sin(engine.command_yaw-engine.yaw), math.cos(engine.command_yaw-engine.yaw)),
        arrival=dict(elapsed_s=elapsed(engine._arrival_since), required_s=engine.navigation_arrival_dwell_s,
                     radius_m=engine.navigation_arrival_radius_m, metric=engine.navigation_arrival_metric),
        arrival_radius_m=engine.navigation_arrival_radius_m, arrival_metric=engine.navigation_arrival_metric,
        height=dict(elapsed_s=elapsed(engine._height_since), required_s=engine.config['flight']['height_dwell_s']),
        task_elapsed_s=max(0., (end if end is not None else now)-start) if start is not None else 0.)

class EventBuffer:
    def __init__(self, limit=256):
        self.boot_id = str(uuid.uuid4())
        self.seq = 0
        self.items = deque(maxlen=limit)
        self.condition = threading.Condition()

    def append(self, event):
        with self.condition:
            self.seq += 1
            item = dict(event, seq=self.seq, boot_id=self.boot_id)
            self.items.append(item)
            self.condition.notify_all()
            return deepcopy(item)

    def read(self, after=0, wait_s=0.):
        with self.condition:
            if self.seq <= after and wait_s:
                self.condition.wait(min(1., max(0., float(wait_s))))
            return dict(boot_id=self.boot_id, last_seq=self.seq,
                        gap=bool(self.items and after and after < self.items[0]['seq']-1),
                        events=deepcopy([e for e in self.items if e['seq'] > after]))

def numbers(values, digits=2):
    return '('+', '.join('%.*f' % (digits, float(v)) for v in values)+')' if values is not None else '—'

def event_line(event):
    item = event.get('effect', {}) if event.get('event') == 'effect' else event
    kind = item.get('type', item.get('event', 'event'))
    reason = item.get('reason', item.get('message', ''))
    prefix = '[#%s] ' % event['seq'] if 'seq' in event else ''
    labels = {'loaded': '绕圈路线', 'delivery': '投放路线', 'shared': '共用路线',
              'entry': '进场段', 'cruise_loop': '巡航闭环', 'delivery_loop': '投放环', 'return': '返航段'}
    source = labels.get(item.get('route_source'), '航线')
    index, lap = item.get('index'), item.get('lap')
    point = ('HOME' if item.get('stage') == 'home' else
             '%s第%s号点' % (source, index+1 if isinstance(index, int) else '—'))
    where = ('第%s圈 ' % lap if lap is not None else '')+point
    if kind == 'delivery_route_point':
        text = '航点登记：%s（已列入航段）' % where
    elif kind == 'delivery_route_reference_passed':
        text = '参考通过：%s%s' % (where, '（闭环）' if item.get('closing') else '')
    elif kind == 'delivery_route_point_arrived':
        text = '实际到位：'+where
    elif kind == 'delivery_route_release_ack':
        text = '投放成功回执已登记：%s，投放编号%s' % (where, item.get('slot', '—'))
    elif kind == 'delivery_route_lap_complete':
        text = '第%s圈完成' % item.get('completed_laps', '—')
    elif kind in ('payload_result', 'payload_late_result'):
        outcome = ('飞控接受' if item.get('success') else
                   '飞控拒绝' if item.get('outcome') == 'rejected' else '结果未确认')
        label = '投放回执' if kind == 'payload_result' else '迟到回执（是否计入以任务记录为准）'
        text = '%s：编号%s，请求%s，%s%s' % (
            label, item.get('slot', '—'), item.get('request_id', '—'), outcome,
            '；'+str(reason) if reason else '')
    elif kind == 'payload_request':
        command = item.get('command') or {}
        text = '舵机请求：编号%s，请求%s，动作%s（等待回执）' % (
            command.get('slot', item.get('slot', '—')), item.get('request_id', '—'),
            command.get('action', item.get('action', '—')))
    elif kind == 'state_changed':
        output = item.get('output') or {}
        text = '状态：%s / %s；%s' % (output.get('state', ''), output.get('phase', ''), output.get('reason', ''))
    else:
        text = '%s %s' % (kind, reason)
    return prefix+text

class TerminalReporter:
    def __init__(self, verbose=False, emit=print):
        self.verbose, self.emit = verbose, emit

    def status(self, status):
        out = status.get('output') or {}
        position = status.get('pose')
        phase = out.get('telemetry', {}).get('phase_name', out.get('phase', ''))
        self.emit('%s / %s | XYZ %s | 目标 %s | %s' % (
            out.get('state', 'IDLE'), phase, numbers(position),
            numbers(out.get('position_xyz')), out.get('reason', '')))
        route = out.get('drop_points') or out.get('drop_test') or out.get('delivery_route') or {}
        alignment = route.get('visual_alignment') or {}
        if out.get('state') == 'DROP' and alignment:
            fallback = alignment.get('fallback') or {}
            fixes = alignment.get('fixes') or []
            target = fixes[-1]['target_xyz'][:2] if fixes else alignment.get('recorded_xy')
            remaining = fallback.get('remaining_s')
            height = fallback.get('release_actual_height_m')
            self.emit('投放点 %s | 对准 %s/2 (%s) | 锁定XY %s | 剩余 %s | 重拍 %s/%s | %s%s' % (
                (route.get('current_point') or {}).get('drop_slot', '—'), alignment.get('stage', 0),
                '绿色圆心' if alignment.get('target_kind') == 'green_circle' else '红点',
                numbers(target), '—' if remaining is None else '%.1fs' % remaining,
                fallback.get('retakes', 0), fallback.get('retake_limit', 1),
                ('兜底：'+fallback.get('reason', '')) if fallback.get('active')
                else fallback.get('waiting_reason', ''),
                '' if height is None else ' | 实际释放高度 %.3fm' % height))

    def summary(self, status):
        out = status.get('output') or {}
        self.emit('任务状态：%s / %s；已确认投放 %s 次；%s' % (
            out.get('state', 'IDLE'), out.get('phase', ''), out.get('drops_completed', 0), out.get('reason', '')))
        if status.get('log_dir'):
            self.emit('日志：'+status['log_dir'])
