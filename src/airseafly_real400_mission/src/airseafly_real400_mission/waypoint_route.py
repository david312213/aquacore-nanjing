"""Ordered XY waypoints with optional cruise/gate types; no ROS or control outputs."""
from copy import deepcopy
import math
import hashlib
from pathlib import Path


FORMAT = 'aquacore_waypoints'
FRAME = 'fast_lio_local_enu'
DEFAULT_ROUTE = Path('data/routes/waypoints.yaml')
LEGACY_DROP_FORMAT = 'aquacore_drop_points'
ENTRY_GUIDE = 3  # Recorder prompt only; flying accepts any number of entry points.


def _xy(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('每个航点必须只有 X/Y 两个数，高度在飞行配置中设置')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError('X/Y 必须为有限数字')
    return [float(value[0]), float(value[1])]


def validate_route(document, require_points=False):
    """Return an independent normalized route, allowing an empty recording.

    ``require_points`` only checks that a flight has at least one destination.
    Coordinates have no age, separation, motion, or hardware readiness rules.
    """
    if not isinstance(document, dict) or document.get('format') not in (FORMAT, LEGACY_DROP_FORMAT):
        raise ValueError('普通航点文件 format 必须为 '+FORMAT+'；不能使用比赛录点文件')
    if type(document.get('version')) is not int or document['version'] not in (1, 2, 3):
        raise ValueError('普通航点文件 version 必须为 1、2 或 3')
    if document.get('frame') != FRAME:
        raise ValueError('航点坐标系必须为 '+FRAME)
    points = document.get('points')
    if not isinstance(points, list):
        raise ValueError('points 必须是按飞行顺序排列的 X/Y 列表')
    result = dict(format=FORMAT, version=1, frame=FRAME, points=[_xy(point) for point in points])
    if document['version'] in (2, 3):
        profiles = document.get('point_profiles')
        if not isinstance(profiles, list) or len(profiles) != len(points) or any(
                v not in ('cruise', 'gate') for v in profiles):
            raise ValueError('point_profiles 必须与 points 一一对应，类型为 cruise 或 gate')
        result.update(version=2, point_profiles=list(profiles))
    if document['version'] == 3:
        # Entry points: flown once from takeoff into the loop formed by points.
        entries, kinds = document.get('entry_points'), document.get('entry_profiles')
        if not isinstance(entries, list):
            raise ValueError('entry_points 必须是进场点 X/Y 列表')
        if not isinstance(kinds, list) or len(kinds) != len(entries) or any(
                v not in ('cruise', 'gate') for v in kinds):
            raise ValueError('entry_profiles 必须与 entry_points 一一对应，类型为 cruise 或 gate')
        section = document.get('recording_section', 'entry')
        if section not in ('entry', 'loop'):
            raise ValueError('recording_section 必须为 entry 或 loop')
        result.update(version=3, entry_points=[_xy(point) for point in entries], entry_profiles=list(kinds),
                      recording_section=section)
    if require_points and not result['points']:
        raise ValueError('普通航点文件没有航点，请先记录至少一个目的点')
    return result


def load_route(path, require_points=False):
    return load_route_snapshot(path, require_points)[0]


def load_route_snapshot(path, require_points=False):
    """Bind the displayed/logged source digest to the exact bytes being flown."""
    import yaml
    path = Path(path).resolve()
    raw = path.read_bytes()
    value = validate_route(yaml.safe_load(raw), require_points=require_points)
    return value, dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest(),
                      recorded_points=len(value['points']))


def save_route(path, document):
    """Atomically save XY and optional point types, including an empty recording."""
    import yaml
    value = validate_route(document)
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name+'.tmp')
    try:
        temporary.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding='utf-8')
        temporary.replace(target)
    finally:
        if temporary.exists():
            temporary.unlink()


class WaypointRecorder:
    """New recordings start with entry points (about three, Enter ends them
    early), then the loop points. Files without entry points keep the old flow."""
    use_entry = True

    def __init__(self, document=None):
        initial = document if document is not None else dict(
            format=FORMAT, version=3, frame=FRAME, points=[], point_profiles=[],
            entry_points=[], entry_profiles=[], recording_section='entry')
        self.document = validate_route(initial)
        self.latest_xy = None
        self.selected_profile = 'cruise'

    @property
    def sections(self):
        return self.use_entry and self.document['version'] == 3

    @property
    def section(self):
        return self.document['recording_section'] if self.sections else 'loop'

    def observe_xy(self, x, y):
        self.latest_xy = _xy((x, y))

    def to_dict(self):
        return deepcopy(self.document)

    def key(self, key):
        key = {'SPACE': ' ', 'X': 'x', 'Q': 'q', 'ENTER': '\n'}.get(key, key)
        if self.sections:
            event = self._section_key(key)
            if event is not None:
                return event
        if key in ('1', '2') and getattr(self, 'allow_profiles', True):
            self.selected_profile = 'cruise' if key == '1' else 'gate'
            self.document.setdefault('point_profiles', ['cruise'] * len(self.document['points']))
            self.document['version'] = max(self.document['version'], 2)
            return dict(kind='profile_selected', profile=self.selected_profile)
        if key in ('q', 'quit'):
            return dict(kind='save_exit', count=len(self.document['points']))
        if key == 'x':
            if not self.document['points']:
                return dict(kind='ignored', message='还没有可撤销的航点')
            if 'point_profiles' in self.document:
                self.document['point_profiles'].pop()
            index = len(self.document['points'])
            return dict(kind='removed', index=index, xy=self.document['points'].pop())
        if key != ' ':
            return dict(kind='ignored', message='空格记录，X 撤销最后一点，Q 保存退出')
        if self.latest_xy is None:
            return dict(kind='ignored', message='尚未收到 planning_odometry 坐标，本次没有记录点')
        point = list(self.latest_xy)
        self.document['points'].append(point)
        if 'point_profiles' in self.document:
            self.document['point_profiles'].append(self.selected_profile)
        return dict(kind='recorded', index=len(self.document['points']), xy=list(point))

    def _section_key(self, key):
        """Entry-section handling; None falls through to the loop behavior."""
        document = self.document
        if key in ('\n', '\r'):
            if self.section != 'entry':
                return dict(kind='ignored', message='已在闭环段；空格记录闭环点，X 撤销，Q 保存')
            document['recording_section'] = 'loop'
            return dict(kind='section_finished', section='loop')
        if key == 'x' and (self.section == 'entry' or not document['points']) and document['entry_points']:
            document['entry_profiles'].pop()
            document['recording_section'] = 'entry'
            return dict(kind='removed', section='entry', index=len(document['entry_points']),
                        xy=document['entry_points'].pop())
        if key == ' ' and self.section == 'entry':
            if self.latest_xy is None:
                return dict(kind='ignored', message='尚未收到 planning_odometry 坐标，本次没有记录点')
            document['entry_points'].append(list(self.latest_xy))
            document['entry_profiles'].append(self.selected_profile)
            event = dict(kind='recorded', section='entry', index=len(document['entry_points']),
                         xy=list(self.latest_xy))
            if len(document['entry_points']) == ENTRY_GUIDE:
                document['recording_section'] = 'loop'
                event['next_section'] = 'loop'
            return event
        return None
