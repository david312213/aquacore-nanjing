"""Offline fixed-route preview; no ROS, control commands, or admission rules."""
from bisect import bisect_left
import json
import math
from pathlib import Path

from .fixed_trajectory import build_fixed_trajectory
from .route_profiles import (resolve_profiles, point_profiles, point_velocities, gate_velocity,
                             entry_points, entry_profiles)


def route_preview(config, document, origin=None, include_samples=False):
    profiles, names = resolve_profiles(config), point_profiles(document)
    assumed = origin is None
    entries, entry_names = entry_points(document), entry_profiles(document)
    ground = tuple(origin) if origin is not None else (*(entries or document['points'])[0], 0.)
    if len(ground) != 3 or not all(math.isfinite(float(v)) for v in ground):
        raise ValueError('预览起飞原点必须是三个有限坐标')
    points = [(*xy, ground[2]+profiles[name]['height_m'])
              for xy, name in zip(document['points'], names)]
    start = (*ground[:2], ground[2]+profiles['cruise']['height_m'])
    trajectory = build_fixed_trajectory(start, points, config.get('navigation', {}).get('laps', 2),
        config['planning']['max_velocity_mps'], config['planning']['max_acceleration_mps2'],
        point_velocities(config, names),
        [(*xy, ground[2]+profiles[name]['height_m']) for xy, name in zip(entries, entry_names)],
        point_velocities(config, entry_names) if entries else None)
    from .waypoint_timing import apply_waypoint_timing
    trajectory = apply_waypoint_timing(trajectory, config, names, entry_names)
    # Display decimation only; flight samples the analytic polynomial at 50 Hz.
    count = min(5000, max(201, int(math.ceil(trajectory.duration/.1))+1))
    event_times = [event['elapsed_s'] for event in trajectory.events]
    samples = []
    for i in range(count):
        sample = trajectory.sample(trajectory.duration*i/(count-1))
        next_event = min(bisect_left(event_times, sample['elapsed_s']), len(trajectory.events)-1)
        lap = trajectory.events[next_event]['lap'] if trajectory.events else 0
        samples.append(dict(t=sample['elapsed_s'], p=list(sample['position']),
            v=math.sqrt(sum(v*v for v in sample['velocity'])),
            a=math.sqrt(sum(v*v for v in sample['acceleration'])), lap=lap))
    boundary = config.get('_virtual_boundary', {})
    corners = boundary.get('corners') or []
    def outside_xy(p):
        turns = [(b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
                 for a, b in zip(corners, corners[1:]+corners[:1])]
        return bool(turns) and not (all(v >= -1e-9 for v in turns) or all(v <= 1e-9 for v in turns))
    def distance_to_boundary(p):
        distances = []
        for a, b in zip(corners, corners[1:]+corners[:1]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length2 = dx*dx+dy*dy
            u = min(1., max(0., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2)) if length2 else 0.
            distances.append(math.hypot(p[0]-a[0]-u*dx, p[1]-a[1]-u*dy))
        return min(distances) if distances else 0.
    for sample in samples:
        sample['outside_boundary_xy'] = outside_xy(sample['p'])
        sample['outside_boundary_distance_m'] = (distance_to_boundary(sample['p'])
                                                 if sample['outside_boundary_xy'] else 0.)
    result = dict(trajectory.summary(), preview_origin_xyz=list(ground),
        preview_origin_assumed=assumed, points_xyz=[list(p) for p in points],
        start_xyz=list(start), end_xyz=list(trajectory.end),
        frame=document['frame'], height_reference='takeoff_origin_z',
        origin_note=('预览假定起飞 XY 为%s、起飞 Z 为 0；实飞以当前起飞原点重新生成。' % ('第一个进场点' if entries else '首点')
                     if assumed else '使用显式预览起飞原点；不会更改真实任务的起飞原点。'),
        realtime_avoidance=False, ego_boundary_effective=False,
        boundary=boundary, diagnostic_only=True,
        outside_boundary_xy_samples=sum(s['outside_boundary_xy'] for s in samples) if corners else None,
        sampled_max_outside_distance_m=max(s['outside_boundary_distance_m'] for s in samples) if corners else None,
        sampled_height_range_m=[min(s['p'][2]-ground[2] for s in samples),
                                max(s['p'][2]-ground[2] for s in samples)],
        preview_sample_count=count, gate_velocity_mps=gate_velocity(config),
        gate_points=names.count('gate')+entry_names.count('gate'))
    if include_samples:
        result['samples'] = samples
        result['trajectory'] = trajectory.to_dict()
    return result


def write_preview_html(path, value):
    """Self-contained plots with scrubber; no network or external JavaScript."""
    data = json.dumps(value, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    template = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>固定航线预览</title><style>
body{font:15px system-ui,sans-serif;background:#f5f7fa;color:#16263c;margin:0;padding:24px}
main{max-width:1100px;margin:auto}h1{font-size:26px;margin:0 0 12px}.note{line-height:1.7}
.plots{display:grid;grid-template-columns:1.35fr 1fr;gap:16px;margin:20px 0}
section{background:white;border:1px solid #d8e0e8;border-radius:10px;padding:16px}
svg{width:100%;display:block}#xy{aspect-ratio:1}.small{aspect-ratio:2.3}
input[type=range]{width:100%}pre{white-space:pre-wrap;font:14px ui-monospace,monospace}
.legend span{display:inline-block;margin:8px 20px 8px 0}.entry{color:#64748b}.one{color:#147d64}.two{color:#a33bb0}
@media(max-width:750px){.plots{grid-template-columns:1fr}body{padding:12px}}
</style><main><h1>连续固定航线 · 两圈参考预览</h1>
<p class="note" id="note"></p><p class="note">航点为必须经过的参考位置，点间是平滑曲线。实时绕障、障碍膨胀和 EGO 虚拟围墙不生效；图中的边界仅作对照。真实飞机仍有定位与跟踪误差。</p>
<div class="legend"><span class="entry">● 进入航线</span><span class="one">● 第一圈</span><span class="two">● 第二圈及收尾</span><span style="color:#d53932">● 边界外采样</span><span style="color:#a66a21">┄ 记录边界</span></div>
<div class="plots"><section><b>俯视 XY（m；等比例）</b><svg id="xy" viewBox="0 0 600 600"></svg></section>
<section><b>相对起飞高度（m）</b><svg id="z" class="small" viewBox="0 0 600 260"></svg><b>速度（m/s）</b><svg id="v" class="small" viewBox="0 0 600 260"></svg><pre id="stats"></pre></section></div>
<section><label for="time">拖动查看沿途参考</label><input id="time" type="range" min="0" step="1"><pre id="point"></pre></section>
</main><script>
const data=__DATA__, samples=data.samples, colors=['#64748b','#147d64','#a33bb0'];
const ns='http://www.w3.org/2000/svg';
function node(tag,attrs,parent,text){const n=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))n.setAttribute(k,v);if(text)n.textContent=text;parent.appendChild(n);return n;}
const xy=document.getElementById('xy'), corners=data.boundary.corners||[], all=samples.map(s=>s.p).concat(data.points_xyz,corners);
const low=[0,1].map(k=>Math.min(...all.map(p=>p[k]))),high=[0,1].map(k=>Math.max(...all.map(p=>p[k])));
const span=Math.max(high[0]-low[0],high[1]-low[1],1),center=low.map((v,k)=>(v+high[k])/2),scale=500/span;
const px=p=>300+(p[0]-center[0])*scale,py=p=>300-(p[1]-center[1])*scale;
node('rect',{x:25,y:25,width:550,height:550,fill:'none',stroke:'#e2e8f0'},xy);
if(corners.length)node('polygon',{points:corners.map(p=>`${px(p)},${py(p)}`).join(' '),fill:'none',stroke:'#e49437','stroke-dasharray':'7 5','stroke-width':2},xy);
let segments=[],segment=[];samples.forEach((s,i)=>{if(segment.length&&samples[i-1].lap!==s.lap){segments.push(segment);segment=[samples[i-1]];}segment.push(s);});if(segment.length)segments.push(segment);
segments.forEach(ss=>node('polyline',{points:ss.map(s=>`${px(s.p)},${py(s.p)}`).join(' '),fill:'none',stroke:colors[Math.min(ss[ss.length-1].lap,2)],'stroke-width':2.5},xy));
samples.filter(s=>s.outside_boundary_xy).forEach(s=>node('circle',{cx:px(s.p),cy:py(s.p),r:2.5,fill:'#d53932'},xy));
data.points_xyz.forEach((p,i)=>{node('circle',{cx:px(p),cy:py(p),r:4,fill:'#16263c'},xy);node('text',{x:px(p)+7,y:py(p)-7,'font-size':13},xy,String(i+1));});
node('text',{x:30,y:593,'font-size':12},xy,`X: ${low[0].toFixed(2)}…${high[0].toFixed(2)}；Y: ${low[1].toFixed(2)}…${high[1].toFixed(2)}`);
const marker=node('circle',{cx:0,cy:0,r:7,fill:'#ff5e3a',stroke:'white','stroke-width':2},xy);
function chart(id,get){const svg=document.getElementById(id),vals=samples.map(get),lo=Math.min(0,...vals),hi=Math.max(...vals,lo+.1),xp=t=>45+530*t/Math.max(data.duration_s,1e-9),yp=y=>220-180*(y-lo)/(hi-lo);
node('path',{d:'M45 20V220H575',fill:'none',stroke:'#9aaabd'},svg);node('polyline',{points:samples.map(s=>`${xp(s.t)},${yp(get(s))}`).join(' '),fill:'none',stroke:'#2865b0','stroke-width':2},svg);
node('text',{x:2,y:30,'font-size':12},svg,hi.toFixed(2));node('text',{x:2,y:220,'font-size':12},svg,lo.toFixed(2));node('text',{x:470,y:245,'font-size':12},svg,`${data.duration_s.toFixed(1)} s`);
const mark=node('circle',{cx:0,cy:0,r:5,fill:'#ff5e3a'},svg);return s=>{mark.setAttribute('cx',xp(s.t));mark.setAttribute('cy',yp(get(s)));};}
const markz=chart('z',s=>s.p[2]-data.preview_origin_xyz[2]),markv=chart('v',s=>s.v);
document.querySelector('h1').textContent=`连续固定航线 · ${data.effective_laps} 圈参考预览`;
document.getElementById('note').textContent=data.origin_note+' 整段一次求解，起止加减速会使两圈曲线略有不同。';
document.getElementById('stats').textContent=`总时长 ${data.duration_s.toFixed(2)} s\\n速度极值 ${data.max_velocity_mps.toFixed(4)} m/s\\n加速度极值 ${data.max_acceleration_mps2.toFixed(4)} m/s²\\n边界外显示采样点：${data.outside_boundary_xy_samples??'未提供边界'}\\n采样最大越界距离：${data.sampled_max_outside_distance_m==null?'未提供边界':data.sampled_max_outside_distance_m.toFixed(3)+' m'}\\n边界结果仅作诊断，采样不能证明整段均在边界内。`;
const slider=document.getElementById('time');slider.max=samples.length-1;slider.value=0;
function update(){const s=samples[Number(slider.value)];marker.setAttribute('cx',px(s.p));marker.setAttribute('cy',py(s.p));markz(s);markv(s);document.getElementById('point').textContent=`t=${s.t.toFixed(2)} s  圈段=${s.lap}\\nXYZ=(${s.p.map(v=>v.toFixed(3)).join(', ')}) m\\n速度=${s.v.toFixed(3)} m/s  加速度=${s.a.toFixed(3)} m/s²`+(s.outside_boundary_xy?`\\n位于记录边界外 ${s.outside_boundary_distance_m.toFixed(3)} m`:'');}
slider.oninput=update;update();
</script></html>'''
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(template.replace('__DATA__', data), encoding='utf-8')
    return str(target)
