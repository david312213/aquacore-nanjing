"""Recorded perimeter, frozen once per mission; no flight commands."""
from pathlib import Path
import hashlib
import math
import yaml
from .waypoint_route import WaypointRecorder, FRAME, FORMAT

BOUNDARY_FORMAT = 'aquacore_virtual_boundary'
DEFAULT_BOUNDARY = Path('data/routes/field_boundary.yaml')


def validate_corners(points):
    if not isinstance(points, list) or len(points) != 4:
        raise ValueError('场地边界需要按顺时针或逆时针顺序记录四个角')
    for p in points:
        if not isinstance(p, (list, tuple)) or len(p) != 2 or any(isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v) for v in p):
            raise ValueError('边界角点必须是有限 XY 坐标')
    turns=[]
    for i in range(4):
        a,b,c=points[i],points[(i+1)%4],points[(i+2)%4]
        turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(v>1e-9 for v in turns) or all(v< -1e-9 for v in turns)):
        raise ValueError('四角必须沿场地边缘依次记录，不能交叉、重合或共线')
    return [[float(v) for v in p] for p in points]


def read_boundary(path):
    p=Path(path).expanduser().resolve();raw=p.read_bytes();d=yaml.safe_load(raw)
    if not isinstance(d,dict) or d.get('format')!=BOUNDARY_FORMAT or type(d.get('version')) is not int or d['version']!=1 or d.get('frame')!=FRAME:
        raise ValueError('场地边界文件格式或坐标系不正确')
    return validate_corners(d.get('corners')), dict(path=str(p),sha256=hashlib.sha256(raw).hexdigest())


def load_recording(path):
    corners,_=read_boundary(path)
    return dict(format=FORMAT,version=1,frame=FRAME,points=corners)


def save_recording(path, document):
    corners=validate_corners(document['points'])
    d=dict(format=BOUNDARY_FORMAT,version=1,frame=FRAME,corners=corners)
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True);temp=p.with_name(p.name+'.tmp')
    try:
        temp.write_text(yaml.safe_dump(d,allow_unicode=True,sort_keys=False));temp.replace(p)
    finally:
        if temp.exists():temp.unlink()


class BoundaryRecorder(WaypointRecorder):
    allow_profiles=False
    use_entry=False  # Four corners only; no entry section.
    def key(self,key):
        if key in (' ', 'SPACE') and len(self.document['points'])==4:
            return dict(kind='ignored',message='四角已录满；X 撤销，Q 保存')
        return super().key(key)


def freeze_boundary(config, project):
    settings=config.get('planning',{}).get('virtual_boundary',{})
    if not settings.get('enabled',False):
        config['_virtual_boundary']=dict(enabled=False)
        return
    path=Path(settings.get('file') or DEFAULT_BOUNDARY).expanduser()
    if not path.is_absolute():path=Path(project)/path
    corners,source=read_boundary(path)
    limits={k:settings.get(k) for k in ('bottom_m','top_m')}
    for k,v in limits.items():
        if v is not None and (isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v)):
            raise ValueError('virtual_boundary.'+k+' 必须为有限数值或 null')
    if all(v is not None for v in limits.values()) and limits['bottom_m']>=limits['top_m']:
        raise ValueError('虚拟墙底部必须低于顶部')
    config['_virtual_boundary']=dict(enabled=True,corners=corners,source=source,**limits)


def launch_arguments(config):
    b=config.get('_virtual_boundary',{})
    if not b.get('enabled'):return []
    return ['boundary_corners:='+';'.join(','.join(str(v) for v in p) for p in b['corners'])]+[
        'boundary_'+name+':='+('' if b.get(name+'_m') is None else str(b[name+'_m'])) for name in ('bottom','top')]
