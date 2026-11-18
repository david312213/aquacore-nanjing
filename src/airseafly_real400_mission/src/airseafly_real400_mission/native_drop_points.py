"""Visit each recorded target once, deliver, then return to actual takeoff XY."""
from copy import deepcopy

from .drop_points import validate_points
from .native_delivery_route import DeliveryRouteEngine
from .native_single_drop import build_single_drop_itinerary


def build_drop_points_itinerary(document, home=None):
    points = validate_points(document, require_points=True)['points']
    steps = []
    for index, point in enumerate(points):
        single = build_single_drop_itinerary(point, index+1, home)
        for step in single[:2]:
            step.update(route_index=index, stage='drop_point_%d' % (index+1))
            steps.append(step)
    return steps + single[2:]


class DropPointsEngine(DeliveryRouteEngine):
    mission_mode = 'drop_points'
    # Returns to the actual takeoff XY, which need not carry an H pad.
    landing_alignment_supported = False

    def _load_delivery_plan(self):
        self.route_document = validate_points(self.config.get('drop_points'), require_points=True)
        self.route_points = [dict(kind='drop', stage='drop_point_%d' % (index+1),
            xy=list(point), drop_slot=index+1) for index, point in enumerate(self.route_document['points'])]
        self.route_home = None
        self._home_registered = False
        self.itinerary = build_drop_points_itinerary(self.route_document)

    def _preflight_itinerary(self, position):
        # HOME is this takeoff's XY; flight rebuilds it from the corrected origin.
        return build_drop_points_itinerary(self.route_document, tuple(position)[:2])

    def _capture_home(self):
        self.route_home = tuple(self.origin_xyz[:2])
        self.landing_xy = self.route_home
        self.itinerary = build_drop_points_itinerary(self.route_document, self.route_home)

    def start(self, *args, **kwargs):
        super().start(*args, **kwargs)
        self._capture_home()
        return self.snapshot()

    def begin_native_flight(self):
        super().begin_native_flight()
        if self.state == 'TAKEOFF' and not self._home_registered:
            self._capture_home()
            self._home_registered = True

    def on_landed(self):
        super().on_landed()
        if self.done and self.phase == 'LANDED':
            self.reason = '%d 个点投放完成，已返回本次起飞点并落地上锁' % self.drops_completed

    def snapshot(self):
        output = super().snapshot()
        route = output.pop('delivery_route')
        stage = ('takeoff' if self.state == 'TAKEOFF' else
                 'return_home' if self.state == 'APPROACH' and self.route_returning_home else
                 'approach_target' if self.state == 'APPROACH' else
                 'recover' if self.phase == 'NATIVE_RECOVER' else
                 'delivery' if self.state == 'DROP' else
                 'landing' if self.state == 'LAND' else
                 'complete' if self.done else 'hold' if self.state == 'HOLD' else 'idle')
        names = dict(takeoff='起飞', approach_target='前往投放点', delivery='对准下降投放',
            recover='投放后回升', return_home='返回本次起飞点', landing='原起飞点降落',
            complete='已返回起飞点并落地上锁', hold='任务停止', idle='等待开始')
        route.pop('lap')
        route.pop('completed_laps')
        route.update(stage=stage, stage_name=names[stage], points=deepcopy(self.route_document['points']))
        output['implementation'] = 'drop_points_px4_native'
        output['payload_total'] = len(self.route_points)
        output['drop_points'] = route
        if self.state != 'DROP':
            label = names[stage]
            if stage == 'approach_target':
                label += ' %d/%d' % (self.route_index+1, len(self.route_points))
            output['telemetry']['phase_name'] = '录点投放测试：'+label
        return output
