"""Ordered test targets: one recorded XY per physical payload stage."""
from .waypoint_route import FRAME, WaypointRecorder, validate_route, load_route, save_route

FORMAT = 'aquacore_drop_points'


def validate_points(document, require_points=False):
    value = validate_route(document, require_points=require_points)
    if len(value['points']) > 4:
        raise ValueError('points 应为最多四个 XY，对应现有四个投放档位')
    value['format'] = FORMAT
    return value


def load_points(path, require_points=False):
    return validate_points(load_route(path), require_points)


def save_points(path, document):
    # Both recorder entry points write the same portable XY format.
    save_route(path, validate_points(document))


def select_points(document, count=None):
    value = validate_route(document, require_points=True)
    if count is not None:
        if type(count) is not int or not 1 <= count <= 4:
            raise ValueError('--count 应为 1～4')
        if count > len(value['points']):
            raise ValueError('只录了 %d 个投放点，不能选择前 %d 个' % (len(value['points']), count))
        value['points'] = value['points'][:count]
        if 'point_profiles' in value:
            value['point_profiles'] = value['point_profiles'][:count]
    return validate_points(value, require_points=True)


class DropPointRecorder(WaypointRecorder):
    allow_profiles = False
    use_entry = False  # Entry points in a shared file are kept, never recorded here.
    def __init__(self, document=None):
        self.document = validate_points(document if document is not None else
            dict(format=FORMAT, version=1, frame=FRAME, points=[]))
        self.latest_xy = None
        self.selected_profile = 'cruise'

    def key(self, key):
        if key in (' ', 'SPACE') and len(self.document['points']) == 4:
            return dict(kind='ignored', message='已录四个投放档位；X 撤销，Q 保存退出')
        return super().key(key)


class SingleDropPointRecorder(DropPointRecorder):
    """Exactly one target: space records it, a later space replaces it."""
    def key(self, key):
        if key in (' ', 'SPACE') and self.document['points'] and self.latest_xy is not None:
            previous = self.document['points'].pop()
            if 'point_profiles' in self.document:
                self.document['point_profiles'].pop()
            event = super().key(key)
            event['replaced'] = previous
            return event
        return super().key(key)
