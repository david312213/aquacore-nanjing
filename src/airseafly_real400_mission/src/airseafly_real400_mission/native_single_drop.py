"""One supplied delivery target, then return to this takeoff's actual origin
(finish 'land') or keep hovering at the post-drop recovery point ('hover')."""
from copy import deepcopy
import math

from .native_delivery_route import DeliveryRouteEngine


def _xy(value, name):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
        raise ValueError(name+' 必须是两个有限数值 [X, Y]，单位米')
    return [float(v) for v in value]


def validate_drop_test(value):
    """Shared CLI/engine input contract; no recorded-route document is needed."""
    if not isinstance(value, dict):
        raise ValueError('drop_test 必须配置 target_xy 和 slot')
    target = _xy(value.get('target_xy'), 'drop_test.target_xy')
    slot = value.get('slot')
    if isinstance(slot, bool) or not isinstance(slot, int) or slot not in (1, 2, 3, 4):
        raise ValueError('drop_test.slot 必须是 1～4 的整数投放编号')
    finish = value.get('finish', 'land')
    if finish not in ('land', 'hover'):
        raise ValueError('drop_test.finish 必须为 land（返航降落）或 hover（投放后悬停）')
    return dict(target_xy=target, slot=slot, finish=finish)


def build_single_drop_itinerary(target_xy, slot, home=None, finish='land'):
    """Preview/execution share the steps; home is captured on the aircraft.

    finish 'hover' ends after the drop's recovery with a hover step instead of
    returning to the takeoff XY and landing.
    """
    settings = validate_drop_test(dict(target_xy=target_xy, slot=slot, finish=finish))
    target = settings['target_xy']
    home = None if home is None else _xy(home, 'home')
    steps = [
        dict(kind='navigation', lap=None, xy=list(target), height_role='cruise',
             route_index=0, stage='drop_test_target', closing=False, completes_lap=None),
        dict(kind='drop', lap=None, xy=list(target), height_role='drop', recover_height_role='cruise',
             route_index=0, stage='drop_test_target', drop_slot=slot),
    ]
    if settings['finish'] == 'hover':
        return steps+[dict(kind='hover', lap=None, xy=list(target), height_role='cruise',
                           route_index=None, stage='hover')]
    return steps+[
        dict(kind='navigation', lap=None, xy=deepcopy(home), height_role='cruise',
             route_index=None, stage='home', closing=False, completes_lap=None),
        dict(kind='land', lap=None, xy=deepcopy(home), height_role='land',
             route_index=None, stage='home'),
    ]


class SingleDropEngine(DeliveryRouteEngine):
    # Returns to the actual takeoff XY, which need not carry an H pad.
    landing_alignment_supported = False
    mission_mode = 'drop_test'

    def _load_delivery_plan(self):
        self.single_drop = validate_drop_test(self.config.get('drop_test'))
        self.route_document = None
        self.route_points = [dict(kind='drop', stage='drop_test_target',
            xy=list(self.single_drop['target_xy']), drop_slot=self.single_drop['slot'])]
        self.route_home = None
        self._single_home_registered = False
        self.itinerary = build_single_drop_itinerary(self.single_drop['target_xy'], self.single_drop['slot'],
                                                     finish=self.single_drop['finish'])

    def _preflight_itinerary(self, position):
        # HOME is this takeoff's XY; flight rebuilds it from the corrected origin.
        return build_single_drop_itinerary(self.single_drop['target_xy'], self.single_drop['slot'],
                                           tuple(position)[:2], self.single_drop['finish'])

    def _capture_single_drop_home(self):
        self.route_home = tuple(self.origin_xyz[:2])
        self.landing_xy = self.route_home
        self.itinerary = build_single_drop_itinerary(self.single_drop['target_xy'],
                                                     self.single_drop['slot'], self.route_home,
                                                     self.single_drop['finish'])

    def start(self, *args, **kwargs):
        super().start(*args, **kwargs)
        # The initial task pose is useful for previews; native preparation then
        # supplies the actual corrected radar ground origin before activation.
        self._capture_single_drop_home()
        return self.snapshot()

    def begin_native_flight(self):
        super().begin_native_flight()
        if self.state == 'TAKEOFF' and not self._single_home_registered:
            self._capture_single_drop_home()
            self._single_home_registered = True

    def on_landed(self):
        super().on_landed()
        if self.done and self.phase == 'LANDED':
            self.reason = '单次投放完成，已返回本次起飞点并落地上锁'

    def snapshot(self):
        output = super().snapshot()
        route = output.pop('delivery_route')
        stage = ('takeoff' if self.state == 'TAKEOFF' else
                 'return_home' if self.state == 'APPROACH' and self.route_returning_home else
                 'approach_target' if self.state == 'APPROACH' else
                 'recover' if self.phase == 'NATIVE_RECOVER' else
                 'delivery' if self.state == 'DROP' else
                 'landing' if self.state == 'LAND' else
                 'hover' if self.phase == 'DROP_HOVER' else
                 'complete' if self.done else 'hold' if self.state == 'HOLD' else 'idle')
        stage_names = dict(takeoff='起飞', approach_target='前往指定投放点', delivery='单次投放',
            recover='投放后回升', return_home='返回本次起飞点', landing='原起飞点降落',
            complete='已返回起飞点并落地上锁', hold='任务暂停', idle='等待开始',
            hover='投放完成，在回升位置持续悬停')
        output['implementation'] = 'drop_test_px4_native'
        output['payload_total'] = 1
        output['drop_test'] = dict(target_xy=list(self.single_drop['target_xy']),
            slot=self.single_drop['slot'], finish=self.single_drop['finish'], home_xy=route['home_xy'], stage=stage,
            stage_name=stage_names[stage], index=route['index'], current_point=route['current_point'],
            itinerary_index=self.itinerary_index, itinerary_steps=len(self.itinerary),
            returning_home=self.route_returning_home, released_slots=route['released_slots'],
            released_points=route['released_points'], visual_alignment=route['visual_alignment'])
        if self.state != 'DROP':
            output['telemetry']['phase_name'] = '单次投放测试：'+stage_names[stage]
        return output
