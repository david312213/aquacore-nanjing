"""Ordered XY-only course recording; no ROS, flight or payload operations."""
from copy import deepcopy
import math
from pathlib import Path


FRAME = 'fast_lio_local_enu'
STAGES = (
    dict(id='home', label='中央起降点 HOME', mode='single', kind='home'),
    dict(id='drop1', label='投放点 1（也是绕圈起点）', mode='single', kind='drop', drop_slot=1),
    dict(id='obstacle2', label='绕障碍物 2 一圈，前往投放点 2', mode='variable', kind='transit'),
    dict(id='drop2', label='投放点 2', mode='single', kind='drop', drop_slot=2),
    dict(id='obstacle3', label='经障碍物 3 与下边界、右边界之间，前往投放点 3', mode='variable', kind='transit'),
    dict(id='drop3', label='投放点 3', mode='single', kind='drop', drop_slot=3),
    dict(id='gate', label='圆环段及门后前往投放点 4', mode='variable', kind='gate'),
    dict(id='drop4', label='投放点 4', mode='single', kind='drop', drop_slot=4),
    dict(id='return_drop1', label='经障碍物 1 与上边界、左边界之间，返回投放点 1', mode='variable', kind='transit'),
)
_STAGE_INDEX = {stage['id']: i for i, stage in enumerate(STAGES)}
# Free recording (version 2): any number of points in walking order, each one
# chosen as an ordinary, ring or drop point. Drops are numbered by record order.
FREE_KINDS = dict(transit='普通点', gate='圆环点', drop='投放点')
MAX_DROPS = 4
# Loop recording (version 4): sections in flying order.
LOOP_SECTIONS = ('entry', 'cruise_loop', 'delivery_loop', 'return')
LOOP_SECTION_NAMES = {'entry': '进场点', 'cruise_loop': '巡航闭环', 'delivery_loop': '投放环', 'return': '返航点'}
ENTRY_GUIDE = 3  # Recorder prompt only; flying accepts any number of entry points.


def _xy(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('坐标必须只有 X/Y 两个数，高度在飞行配置中单独设置')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError('X/Y 必须为有限数字')
    return [float(value[0]), float(value[1])]


def validate_route(document, require_complete=False):
    """Return a normalized copy; completion checks only recorded structure.

    Empty variable sections are allowed. No position age, motion, minimum
    point spacing, or hardware readiness is inferred from a saved route.
    """
    if isinstance(document, dict) and type(document.get('version')) is int and document['version'] == 3:
        # Legacy split files reuse their delivery points for every lap.
        # Ignore the retired loaded_route, even when it is empty or unfinished.
        if document.get('mode') != 'split':
            raise ValueError('version 3 航线 mode 必须为 split')
        return _validate_free(dict(document, version=2, mode='free'), require_complete)
    if isinstance(document, dict) and type(document.get('version')) is int and document['version'] == 4:
        return _validate_loops(document, require_complete)
    if isinstance(document, dict) and type(document.get('version')) is int and document['version'] == 2:
        return _validate_free(document, require_complete)
    if not isinstance(document, dict) or type(document.get('version')) is not int or document['version'] != 1:
        raise ValueError('航线文件 version 必须为 1、2、3 或 4')
    if document.get('frame') != FRAME:
        raise ValueError('航线坐标系必须为 '+FRAME)
    result = dict(version=1, frame=FRAME, home=None, route=[])
    if document.get('home') is not None:
        result['home'] = _xy(document['home'])
    points = document.get('route')
    if not isinstance(points, list):
        raise ValueError('route 必须是按规则顺序排列的航点列表')
    counts, previous = {}, 0
    for point in points:
        if not isinstance(point, dict) or point.get('stage') not in _STAGE_INDEX:
            raise ValueError('航点缺少有效的 stage')
        index = _STAGE_INDEX[point['stage']]
        stage = STAGES[index]
        if index == 0 or index < previous:
            raise ValueError('HOME 不进入 route，航点必须保持规则阶段顺序')
        if point.get('kind') != stage['kind']:
            raise ValueError('航点 kind 与规则阶段不一致')
        record = dict(stage=stage['id'], kind=stage['kind'], xy=_xy(point.get('xy')))
        if stage['kind'] == 'drop':
            slot = point.get('drop_slot')
            if isinstance(slot, bool) or slot != stage['drop_slot']:
                raise ValueError('投放点与 drop_slot 不一致')
            record['drop_slot'] = stage['drop_slot']
        elif 'drop_slot' in point:
            raise ValueError('普通点和圆环点不能带 drop_slot')
        counts[index] = counts.get(index, 0)+1
        if stage['mode'] == 'single' and counts[index] > 1:
            raise ValueError('每个投放点只能记录一次，闭环复用投放点 1')
        result['route'].append(record)
        previous = index
    wizard = document.get('wizard')
    if not isinstance(wizard, dict):
        raise ValueError('航线文件缺少 wizard.stage_index')
    cursor = wizard.get('stage_index')
    if isinstance(cursor, bool) or not isinstance(cursor, int) or not 0 <= cursor <= len(STAGES):
        raise ValueError('wizard.stage_index 必须在 0 到 9 之间')
    if (cursor == 0) != (result['home'] is None):
        raise ValueError('HOME 与录点进度不一致')
    for index, count in counts.items():
        if index > cursor or (index == cursor and STAGES[index]['mode'] == 'single'):
            raise ValueError('已记录航点超出当前录点进度')
    for index, stage in enumerate(STAGES[1:], 1):
        if index < cursor and stage['mode'] == 'single' and counts.get(index) != 1:
            raise ValueError('缺少 '+stage['label'])
    result['wizard'] = dict(stage_index=cursor)
    if require_complete and cursor != len(STAGES):
        raise ValueError('航线尚未录完；可以继续 --resume 录点')
    return result


def _validate_free(document, require_complete):
    """Version 2. Flying needs HOME and 1 to 4 drops; partial saves need neither."""
    if document.get('frame') != FRAME:
        raise ValueError('航线坐标系必须为 '+FRAME)
    if document.get('mode') != 'free':
        raise ValueError('version 2 航线 mode 必须为 free')
    result = dict(version=2, frame=FRAME, mode='free', home=None, route=[])
    if document.get('home') is not None:
        result['home'] = _xy(document['home'])
    points = document.get('route')
    if not isinstance(points, list):
        raise ValueError('route 必须是按录制顺序排列的航点列表')
    drops = 0
    for point in points:
        if not isinstance(point, dict) or point.get('kind') not in FREE_KINDS:
            raise ValueError('航点 kind 必须为 transit、gate 或 drop')
        record = dict(stage=point['kind'], kind=point['kind'], xy=_xy(point.get('xy')))
        if point['kind'] == 'drop':
            drops += 1
            if drops > MAX_DROPS:
                raise ValueError('投放点最多 4 个')
            slot = point.get('drop_slot')
            if isinstance(slot, bool) or slot != drops:
                raise ValueError('投放点必须按录制顺序编号为 1～4')
            record.update(stage='drop%d' % drops, drop_slot=drops)
        elif 'drop_slot' in point:
            raise ValueError('普通点和圆环点不能带 drop_slot')
        result['route'].append(record)
    if require_complete:
        if result['home'] is None:
            raise ValueError('航线缺少 HOME；录点时站在起降点按 h 记录')
        if not drops:
            raise ValueError('航线没有投放点；录点时按 3 切到投放点，在投放位置按空格记录')
    return result


def _validate_loops(document, require_complete):
    """Version 4: entry points, a closed cruise loop, a separate delivery loop
    and return points. Flying needs HOME and 1 to 4 drops, as for version 2;
    any section may be empty and is then simply skipped."""
    if document.get('frame') != FRAME:
        raise ValueError('航线坐标系必须为 '+FRAME)
    if document.get('mode') != 'loops':
        raise ValueError('version 4 航线 mode 必须为 loops')
    result = dict(version=4, frame=FRAME, mode='loops', home=None)
    if document.get('home') is not None:
        result['home'] = _xy(document['home'])
    drops = 0
    for section in LOOP_SECTIONS:
        points = document.get(section, [])
        if not isinstance(points, list):
            raise ValueError(section+' 必须是按飞行顺序排列的航点列表')
        records = []
        for point in points:
            allowed = ('transit', 'gate', 'drop') if section == 'delivery_loop' else ('transit', 'gate')
            if not isinstance(point, dict) or point.get('kind') not in allowed:
                raise ValueError('%s 航点 kind 必须为 %s' % (LOOP_SECTION_NAMES[section], '、'.join(allowed)))
            record = dict(stage=point['kind'], kind=point['kind'], xy=_xy(point.get('xy')))
            if point['kind'] == 'drop':
                drops += 1
                if drops > MAX_DROPS:
                    raise ValueError('投放点最多 4 个')
                slot = point.get('drop_slot')
                if isinstance(slot, bool) or slot != drops:
                    raise ValueError('投放点必须按录制顺序编号为 1～4')
                record.update(stage='drop%d' % drops, drop_slot=drops)
            elif 'drop_slot' in point:
                raise ValueError('普通点和圆环点不能带 drop_slot')
            records.append(record)
        result[section] = records
    recording = document.get('recording', {})
    cursor = recording.get('section', 'entry') if isinstance(recording, dict) else None
    if cursor not in LOOP_SECTIONS+('done',):
        raise ValueError('recording.section 必须为 '+'、'.join(LOOP_SECTIONS+('done',)))
    result['recording'] = dict(section=cursor)
    if require_complete:
        if result['home'] is None:
            raise ValueError('航线缺少 HOME；录点时站在起降点按 h 记录')
        if not drops:
            raise ValueError('投放环没有投放点；录到投放环时按 3 切到投放点，在投放位置按空格记录')
    return result


def route_sections(document):
    """Flight sections of any route format as (name, points) in flying order.

    Only version 4 has separate sections; older formats are one shared route.
    """
    document = validate_route(document)
    if document['version'] == 4:
        return [(section, document[section]) for section in LOOP_SECTIONS]
    return [('shared', document['route'])]


def all_route_points(document):
    return [point for _section, points in route_sections(document) for point in points]


def load_route(path, require_complete=False):
    import yaml
    with Path(path).open(encoding='utf-8') as stream:
        return validate_route(yaml.safe_load(stream), require_complete=require_complete)


def save_route(path, document):
    """Save complete or partial progress; files contain XY and no height."""
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


class GuidedRouteRecorder:
    def __init__(self, document=None):
        initial = document if document is not None else dict(
            version=1, frame=FRAME, home=None, route=[], wizard=dict(stage_index=0))
        self.document = validate_route(initial)
        self.latest_xy = None

    @property
    def stage_index(self):
        return self.document['wizard']['stage_index']

    @property
    def current_stage(self):
        return None if self.complete else deepcopy(STAGES[self.stage_index])

    @property
    def complete(self):
        return self.stage_index == len(STAGES)

    def observe_xy(self, x, y):
        self.latest_xy = _xy((x, y))

    def to_dict(self):
        return deepcopy(self.document)

    def key(self, key):
        key = {'SPACE': ' ', 'ENTER': '\n', 'X': 'x', 'Q': 'q'}.get(key, key)
        if key in ('q', 'quit'):
            return dict(kind='save_exit', complete=self.complete)
        if key == 'x':
            if self.document['route']:
                removed = self.document['route'].pop()
                self.document['wizard']['stage_index'] = _STAGE_INDEX[removed['stage']]
            elif self.document['home'] is not None:
                removed = dict(stage='home', kind='home', xy=self.document['home'])
                self.document['home'] = None
                self.document['wizard']['stage_index'] = 0
            else:
                return dict(kind='ignored', message='还没有可撤销的记录点')
            return dict(kind='removed', point=deepcopy(removed), stage_index=self.stage_index)
        if self.complete:
            return dict(kind='ignored', message='九个阶段已完成；Q 保存退出，X 撤销最后一点')
        stage = self.current_stage
        if key in ('\n', '\r'):
            if stage['mode'] != 'variable':
                return dict(kind='ignored', message='当前是固定点，请按空格记录后自动进入下一阶段')
            self.document['wizard']['stage_index'] += 1
            return dict(kind='stage_finished', stage=stage['id'], complete=self.complete)
        if key != ' ':
            return dict(kind='ignored', message='空格记录，Enter 结束可变段，X 撤销，Q 保存退出')
        if self.latest_xy is None:
            return dict(kind='ignored', message='尚未收到 planning_odometry 坐标，本次没有记录点')
        point = dict(stage=stage['id'], kind=stage['kind'], xy=list(self.latest_xy))
        if stage['id'] == 'home':
            self.document['home'] = list(self.latest_xy)
        else:
            if 'drop_slot' in stage:
                point['drop_slot'] = stage['drop_slot']
            self.document['route'].append(point)
        if stage['mode'] == 'single':
            self.document['wizard']['stage_index'] += 1
        return dict(kind='recorded', point=deepcopy(point), stage_index=self.stage_index)

    def status_text(self, position):
        stage, count = self.current_stage, len(self.document['route'])
        if stage is None:
            return '9/9 已录完；%d 个航点；%s；Q 保存，X 撤销' % (count, position)
        if stage['mode'] == 'single':
            action = '空格记录单点后自动进入下一阶段'
        else:
            stage_count = sum(point['stage'] == stage['id'] for point in self.document['route'])
            action = '本段已记录 %d 个航点；空格继续，Enter 结束本段' % stage_count
        return '%d/%d %s｜%s｜%s' % (self.stage_index+1, len(STAGES), stage['label'], action, position)


class FreeRouteRecorder:
    """Walk the course and record points in flying order, choosing each type.

    h records HOME (re-press to replace it). 1/2/3 only select ordinary, ring
    or drop; space records the latest XY as the selected type, drops numbered
    in record order. x removes the last route point; HOME is replaced with h,
    not undone.
    """
    SELECT_KEYS = {'1': 'transit', '2': 'gate', '3': 'drop'}

    def __init__(self, document=None):
        initial = document if document is not None else dict(
            version=2, frame=FRAME, mode='free', home=None, route=[])
        self.document = validate_route(initial)
        if self.document.get('mode') != 'free':
            raise ValueError('自由录点只能继续 version 2 航线；旧向导航线请加 --wizard')
        self.latest_xy = None
        self.selected_kind = 'transit'

    @property
    def drops(self):
        return sum(point['kind'] == 'drop' for point in self.document['route'])

    @property
    def points(self):
        return self.document['route']

    @property
    def complete(self):
        return self.document['home'] is not None and 1 <= self.drops <= MAX_DROPS

    def observe_xy(self, x, y):
        self.latest_xy = _xy((x, y))

    def to_dict(self):
        return deepcopy(self.document)

    def key(self, key):
        key = {'SPACE': ' ', 'X': 'x', 'Q': 'q', 'H': 'h'}.get(key, key)
        if key in ('q', 'quit'):
            return dict(kind='save_exit', complete=self.complete)
        if key in self.SELECT_KEYS:
            self.selected_kind = self.SELECT_KEYS[key]
            return dict(kind='kind_selected', point_kind=self.selected_kind)
        if key == 'x':
            if not self.points:
                return dict(kind='ignored', message='还没有可撤销的航点；HOME 用 h 重新记录')
            return dict(kind='removed', point=self.points.pop())
        if key not in (' ', 'h'):
            return dict(kind='ignored', message='h 记 HOME，1 普通 / 2 圆环 / 3 投放 切换，空格记点，X 撤销，Q 保存')
        if self.latest_xy is None:
            return dict(kind='ignored', message='尚未收到 planning_odometry 坐标，本次没有记录点')
        if key == 'h':
            replaced = self.document['home'] is not None
            self.document['home'] = list(self.latest_xy)
            return dict(kind='home_recorded', xy=list(self.latest_xy), replaced=replaced)
        if self.selected_kind == 'drop':
            if self.drops >= MAX_DROPS:
                return dict(kind='ignored', message='已有 4 个投放点，不能再记录投放点')
            point = dict(stage='drop%d' % (self.drops+1), kind='drop', xy=list(self.latest_xy),
                         drop_slot=self.drops+1)
        else:
            point = dict(stage=self.selected_kind, kind=self.selected_kind, xy=list(self.latest_xy))
        self.points.append(point)
        return dict(kind='recorded', point=deepcopy(point))

    def status_text(self, position):
        return '已记录 %d 个点（投放 %d/4）｜HOME %s｜当前：%s｜%s｜h HOME，1 普通 / 2 圆环 / 3 投放 切换，空格记点，X 撤销，Q 保存' % (
            len(self.document['route']), self.drops, '已记录' if self.document['home'] else '未记录',
            FREE_KINDS[self.selected_kind], position)


class LoopRouteRecorder:
    """Record entry points, the cruise loop, the delivery loop and the return.

    h records HOME at any time. Space records the latest XY into the current
    section as the selected type (1 ordinary / 2 ring / 3 drop, drop only in
    the delivery loop). Entry moves on after three points or on Enter; the
    other sections end on Enter. X removes the last recorded point and returns
    to its section. The cruise loop closes back to its first point when flown.
    """
    SELECT_KEYS = {'1': 'transit', '2': 'gate', '3': 'drop'}

    def __init__(self, document=None):
        initial = document if document is not None else dict(
            version=4, frame=FRAME, mode='loops', home=None, recording=dict(section='entry'))
        self.document = validate_route(initial)
        if self.document['version'] != 4:
            raise ValueError('分段录点只能继续 version 4 航线')
        self.latest_xy = None
        self.selected_kind = 'transit'

    @property
    def section(self):
        return self.document['recording']['section']

    def _set_section(self, section):
        self.document['recording']['section'] = section
        if section != 'delivery_loop' and self.selected_kind == 'drop':
            self.selected_kind = 'transit'

    @property
    def drops(self):
        return sum(point['kind'] == 'drop' for point in self.document['delivery_loop'])

    @property
    def complete(self):
        return self.document['home'] is not None and 1 <= self.drops <= MAX_DROPS

    def observe_xy(self, x, y):
        self.latest_xy = _xy((x, y))

    def to_dict(self):
        return deepcopy(self.document)

    def _advance(self):
        order = LOOP_SECTIONS+('done',)
        self._set_section(order[order.index(self.section)+1])
        return dict(kind='section_finished', section=self.section)

    def key(self, key):
        key = {'SPACE': ' ', 'ENTER': '\n', 'X': 'x', 'Q': 'q', 'H': 'h'}.get(key, key)
        if key in ('q', 'quit'):
            return dict(kind='save_exit', complete=self.complete)
        if key in self.SELECT_KEYS:
            kind = self.SELECT_KEYS[key]
            if kind == 'drop' and self.section != 'delivery_loop':
                return dict(kind='ignored', message='投放点只能录在投放环；当前为'+self.section_name)
            self.selected_kind = kind
            return dict(kind='kind_selected', point_kind=kind)
        if key == 'x':
            order = LOOP_SECTIONS if self.section == 'done' else LOOP_SECTIONS[:LOOP_SECTIONS.index(self.section)+1]
            for section in reversed(order):
                if self.document[section]:
                    removed = self.document[section].pop()
                    self._set_section(section)
                    return dict(kind='removed', point=removed, section=section)
            return dict(kind='ignored', message='还没有可撤销的航点；HOME 用 h 重新记录')
        if key in ('\n', '\r'):
            if self.section == 'done':
                return dict(kind='ignored', message='各段已录完；Q 保存，X 撤销最后一点')
            return self._advance()
        if key not in (' ', 'h'):
            return dict(kind='ignored', message='h 记 HOME，1 普通 / 2 圆环 / 3 投放，空格记点，Enter 结束本段，X 撤销，Q 保存')
        if self.latest_xy is None:
            return dict(kind='ignored', message='尚未收到 planning_odometry 坐标，本次没有记录点')
        if key == 'h':
            replaced = self.document['home'] is not None
            self.document['home'] = list(self.latest_xy)
            return dict(kind='home_recorded', xy=list(self.latest_xy), replaced=replaced)
        if self.section == 'done':
            return dict(kind='ignored', message='各段已录完；X 撤销最后一点后可继续录返航点')
        section = self.section
        if self.selected_kind == 'drop':
            if self.drops >= MAX_DROPS:
                return dict(kind='ignored', message='已有 4 个投放点，不能再记录投放点')
            point = dict(stage='drop%d' % (self.drops+1), kind='drop', xy=list(self.latest_xy),
                         drop_slot=self.drops+1)
        else:
            point = dict(stage=self.selected_kind, kind=self.selected_kind, xy=list(self.latest_xy))
        self.document[section].append(point)
        event = dict(kind='recorded', point=deepcopy(point), section=section)
        if section == 'entry' and len(self.document['entry']) == ENTRY_GUIDE:
            self._advance()
            event['next_section'] = self.section
        return event

    @property
    def section_name(self):
        return '已录完' if self.section == 'done' else LOOP_SECTION_NAMES[self.section]

    def status_text(self, position):
        counts = '进场 %d｜巡航 %d｜投放环 %d（投放 %d/4）｜返航 %d' % (
            len(self.document['entry']), len(self.document['cruise_loop']),
            len(self.document['delivery_loop']), self.drops, len(self.document['return']))
        if self.section == 'entry':
            current = '进场点 %d/%d（录满自动进入巡航闭环，Enter 提前结束）' % (
                len(self.document['entry'])+1, ENTRY_GUIDE)
        elif self.section == 'done':
            current = '各段已录完，Q 保存'
        else:
            current = '%s（Enter 结束本段）' % self.section_name
        return '当前：%s · %s｜%s｜HOME %s｜%s｜h HOME，1 普通 / 2 圆环 / 3 投放，空格记点，X 撤销，Q 保存' % (
            current, FREE_KINDS[self.selected_kind], counts,
            '已记录' if self.document['home'] else '未记录', position)


def route_recorder(document):
    """The recorder that continues an existing file, by its format."""
    document = validate_route(document)
    if document.get('mode') == 'loops':
        return LoopRouteRecorder(document)
    return FreeRouteRecorder(document) if document.get('mode') == 'free' else GuidedRouteRecorder(document)


class RecorderKeyDecoder:
    """Decode recorder keys in order, ignoring arrows and bracketed paste.

    This deliberately does not instantiate the flight keyboard controller:
    X here only removes a record and Q only saves the route.
    """
    def __init__(self):
        self.sequence = b''
        self.pasting = False
        self.paste_tail = b''
        self.previous_cr = False

    def feed(self, data):
        keys = []
        for value in data:
            char = bytes((value,))
            if self.pasting:
                self.paste_tail = (self.paste_tail+char)[-6:]
                if self.paste_tail == b'\x1b[201~':
                    self.pasting, self.paste_tail = False, b''
                continue
            if self.sequence:
                self.sequence += char
                if len(self.sequence) == 2 and char in (b'[', b'O'):
                    continue
                if len(self.sequence) >= 3 and self.sequence[1:2] in (b'[', b'O'):
                    if not 0x40 <= value <= 0x7e and len(self.sequence) < 32:
                        continue
                    if self.sequence == b'\x1b[200~':
                        self.pasting = True
                self.sequence = b''
                continue
            if value == 27:
                self.sequence = char
                continue
            if value == 10 and self.previous_cr:
                self.previous_cr = False
                continue
            self.previous_cr = value == 13
            if value in (3, 4):
                keys.append('q')
            elif value in (10, 13):
                keys.append('\n')
            elif value < 128 and chr(value).lower() in (' ', 'x', 'q', '1', '2', '3', 'h'):
                keys.append(chr(value).lower())
        return keys
