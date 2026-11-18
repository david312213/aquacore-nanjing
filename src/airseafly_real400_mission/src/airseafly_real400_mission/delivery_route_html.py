"""Offline HTML view of the contest route's planned fixed legs.

Read-only: builds the same itinerary and fixed legs as --dry-run (HOME ground
at Z=0) and writes one self-contained HTML file. No ROS, no commands, and it
never decides whether a route may fly.
"""
from bisect import bisect_right
import json
import math
from pathlib import Path

from .guided_route import route_sections, validate_route
from .native_delivery_route import build_itinerary, describe_laps, plan_fixed_legs


def _category(step, laps):
    """laps are cruise laps: lap 0 is entry, laps+1 is the delivery pass."""
    if step['lap'] is None or step.get('route_source') == 'return':
        return 'home'
    if step['lap'] == 0:
        return 'entry'
    return 'loaded' if step['lap'] <= laps else 'delivery'


SECTION_TAGS = dict(shared='', entry='进', cruise_loop='巡', delivery_loop='投环', **{'return': '返'})


def delivery_preview_data(config, document):
    document = validate_route(document, require_complete=True)
    laps = config['delivery_route_options'].get('laps', 1)
    itinerary = build_itinerary(document, laps)
    home = list(document['home'])
    backend = config.get('navigation', {}).get('backend', 'ego')
    samples, legs, offset = [], [], 0.
    if backend == 'fixed_min_snap':
        for number, leg in enumerate(plan_fixed_legs(config, itinerary, (*home, 0.)), 1):
            trajectory = leg['trajectory']
            times = [event['elapsed_s'] for event in trajectory.events]
            count = max(2, int(math.ceil(trajectory.duration/.1))+1)
            for i in range(count):
                sample = trajectory.sample(trajectory.duration*i/(count-1))
                # Each sample belongs to the next waypoint it is flying toward.
                index = min(bisect_right(times, sample['elapsed_s']-1e-9), len(times)-1)
                step = itinerary[trajectory.events[index]['itinerary_index']]
                category = 'entry' if number == 1 and index == 0 else _category(step, laps)
                samples.append(dict(t=offset+sample['elapsed_s'], p=[round(v, 4) for v in sample['position']],
                                    v=round(math.sqrt(sum(v*v for v in sample['velocity'])), 4),
                                    lap=step['lap'], cat=category, leg=number))
            summary = trajectory.summary()
            legs.append(dict(number=number, first_step=leg['first']+1, last_step=leg['last']+1,
                             ends=itinerary[leg['last']+1]['kind'], start_s=offset,
                             duration_s=trajectory.duration, max_velocity_mps=summary['max_velocity_mps'],
                             retimed=bool((summary.get('local_retiming') or {}).get('applied'))))
            offset += trajectory.duration
    corners = (config.get('_virtual_boundary') or {}).get('corners') or []
    points, number = [], 0
    for section, recorded in route_sections(document):
        for i, point in enumerate(recorded):
            number += 1
            points.append(dict(number=number, label=SECTION_TAGS[section]+str(i+1), section=section,
                               xy=list(point['xy']), kind=point['kind'], slot=point.get('drop_slot')))
    if document['version'] == 4:
        note = ('进场 %d 点 → 巡航闭环 %d 圈 → 投放环（投放点停下投放）→ 返航 %d 点 → HOME 降落。' % (
            len(document['entry']), laps, len(document['return'])))
    else:
        note = ('巡航 %d 圈满载绕圈，第 %d 圈在投放点停下投放，走完全部航点回到第一个点，再回 HOME 降落。' % (laps, laps+1)
                if laps else '只飞一圈，在投放点停下投放，走完全部航点回到第一个点，再回 HOME 降落。')
    return dict(laps=laps, note=note, backend=backend, home=home, points=points, boundary=[list(c) for c in corners],
                samples=samples, legs=legs, duration_s=offset, order=describe_laps(itinerary),
                heights=dict(cruise=config['flight']['cruise_height_m'], drop=config['flight']['drop_height_m']))


TEMPLATE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>比赛航线轨迹预览</title><style>
body{font:15px system-ui,sans-serif;background:#f5f7fa;color:#16263c;margin:0;padding:20px}
main{max-width:1200px;margin:auto}h1{font-size:24px;margin:0 0 8px}.note{line-height:1.6;color:#475569;margin:4px 0}
.plots{display:grid;grid-template-columns:1.4fr 1fr;gap:16px;margin:16px 0}
section{background:white;border:1px solid #d8e0e8;border-radius:10px;padding:14px}
svg{width:100%;display:block}#xy{aspect-ratio:1}.small{aspect-ratio:2.3}
input[type=range]{width:100%}pre{white-space:pre-wrap;font:13px ui-monospace,monospace;margin:6px 0}
.legend span{display:inline-block;margin:6px 18px 6px 0}
@media(max-width:800px){.plots{grid-template-columns:1fr}body{padding:10px}}
</style><main><h1>比赛航线 · 固定轨迹预览</h1>
<p class="note" id="note"></p>
<div class="legend" id="legend"></div>
<div class="plots"><section><b>俯视 XY（m，等比例；数字=录点序号，▼=投放点，H=HOME）</b><svg id="xy" viewBox="0 0 600 600"></svg></section>
<section><b>相对起飞高度（m）</b><svg id="z" class="small" viewBox="0 0 600 260"></svg>
<b>速度（m/s）</b><svg id="v" class="small" viewBox="0 0 600 260"></svg><pre id="stats"></pre></section></div>
<section><label for="time">拖动查看沿途参考（只含航段飞行时间，不含对准、投放、降落）</label>
<input id="time" type="range" min="0" step="1"><pre id="point"></pre></section>
<section style="margin-top:16px"><b>飞行顺序</b><pre id="order"></pre></section>
</main><script>
const data=__DATA__, S=data.samples;
const C={entry:'#64748b',loaded:'#147d64',delivery:'#a33bb0',home:'#d97706'};
const N={entry:'起飞→进场',loaded:'巡航闭环',delivery:'投放环',home:'返航→HOME'};
const ns='http://www.w3.org/2000/svg';
function node(tag,attrs,parent,text){const n=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))n.setAttribute(k,v);if(text!==undefined)n.textContent=text;parent.appendChild(n);return n;}
document.getElementById('legend').innerHTML=Object.keys(C).map(k=>`<span style="color:${C[k]}">━ ${N[k]}</span>`).join('')+'<span style="color:#e49437">┄ 记录边界</span>';
const xy=document.getElementById('xy'),pts=data.points.map(p=>p.xy),all=S.map(s=>s.p).concat(pts,[data.home],data.boundary);
const lo=[0,1].map(k=>Math.min(...all.map(p=>p[k]))),hi=[0,1].map(k=>Math.max(...all.map(p=>p[k])));
const span=Math.max(hi[0]-lo[0],hi[1]-lo[1],1),ctr=lo.map((v,k)=>(v+hi[k])/2),sc=520/span;
const px=p=>300+(p[0]-ctr[0])*sc,py=p=>300-(p[1]-ctr[1])*sc;
// 1 m grid
for(let x=Math.floor(ctr[0]-span/2-1);x<=ctr[0]+span/2+1;x++)node('line',{x1:px([x,0]),x2:px([x,0]),y1:0,y2:600,stroke:'#eef2f6'},xy);
for(let y=Math.floor(ctr[1]-span/2-1);y<=ctr[1]+span/2+1;y++)node('line',{y1:py([0,y]),y2:py([0,y]),x1:0,x2:600,stroke:'#eef2f6'},xy);
if(data.boundary.length)node('polygon',{points:data.boundary.map(p=>`${px(p)},${py(p)}`).join(' '),fill:'none',stroke:'#e49437','stroke-dasharray':'7 5','stroke-width':2},xy);
if(S.length){let seg=[S[0]];const draw=ss=>node('polyline',{points:ss.map(s=>`${px(s.p)},${py(s.p)}`).join(' '),fill:'none',stroke:C[ss[ss.length-1].cat],'stroke-width':2.5,'stroke-opacity':.85},xy);
for(let i=1;i<S.length;i++){if(S[i].cat!==S[i-1].cat||S[i].leg!==S[i-1].leg){draw(seg);seg=S[i].leg===S[i-1].leg?[S[i-1]]:[];}seg.push(S[i]);}draw(seg);}
else node('polyline',{points:[data.home].concat(pts,[pts[0],data.home]).map(p=>`${px(p)},${py(p)}`).join(' '),fill:'none',stroke:'#94a3b8','stroke-dasharray':'4 4','stroke-width':2},xy);
data.points.forEach(p=>{const c=[px(p.xy),py(p.xy)];
 if(p.kind==='drop'){node('path',{d:`M${c[0]-8} ${c[1]-7}H${c[0]+8}L${c[0]} ${c[1]+8}Z`,fill:'#dc2626'},xy);node('text',{x:c[0]+9,y:c[1]-9,'font-size':13,'font-weight':'bold',fill:'#dc2626'},xy,`${p.label} 投${p.slot}`);}
 else{node('circle',{cx:c[0],cy:c[1],r:p.kind==='gate'?5:4,fill:p.kind==='gate'?'#2563eb':'#16263c'},xy);node('text',{x:c[0]+7,y:c[1]-7,'font-size':12,fill:p.kind==='gate'?'#2563eb':'#16263c'},xy,p.kind==='gate'?`${p.label} 环`:String(p.label));}});
node('circle',{cx:px(data.home),cy:py(data.home),r:10,fill:'none',stroke:'#d97706','stroke-width':2.5},xy);node('text',{x:px(data.home)-5,y:py(data.home)+5,'font-size':13,'font-weight':'bold',fill:'#d97706'},xy,'H');
node('text',{x:8,y:594,'font-size':12,fill:'#64748b'},xy,`X ${lo[0].toFixed(2)}…${hi[0].toFixed(2)}  Y ${lo[1].toFixed(2)}…${hi[1].toFixed(2)}  网格 1 m`);
const marker=node('circle',{cx:-20,cy:-20,r:7,fill:'#ff5e3a',stroke:'white','stroke-width':2},xy);
function chart(id,get){const svg=document.getElementById(id);if(!S.length)return()=>{};const vals=S.map(get),l=Math.min(0,...vals),h=Math.max(...vals,l+.1),xp=t=>45+530*t/Math.max(data.duration_s,1e-9),yp=y=>220-180*(y-l)/(h-l);
node('path',{d:'M45 20V220H575',fill:'none',stroke:'#9aaabd'},svg);
data.legs.forEach(g=>node('line',{x1:xp(g.start_s),x2:xp(g.start_s),y1:20,y2:220,stroke:'#e2e8f0'},svg));
let seg=[S[0]];const draw=ss=>node('polyline',{points:ss.map(s=>`${xp(s.t)},${yp(get(s))}`).join(' '),fill:'none',stroke:C[ss[ss.length-1].cat],'stroke-width':2},svg);
for(let i=1;i<S.length;i++){if(S[i].cat!==S[i-1].cat){draw(seg);seg=[S[i-1]];}seg.push(S[i]);}draw(seg);
node('text',{x:2,y:30,'font-size':12},svg,h.toFixed(2));node('text',{x:2,y:220,'font-size':12},svg,l.toFixed(2));node('text',{x:500,y:245,'font-size':12},svg,`${data.duration_s.toFixed(1)} s`);
const m=node('circle',{cx:-20,cy:-20,r:5,fill:'#ff5e3a'},svg);return s=>{m.setAttribute('cx',xp(s.t));m.setAttribute('cy',yp(get(s)));};}
const mz=chart('z',s=>s.p[2]),mv=chart('v',s=>s.v);
document.getElementById('note').textContent=data.note+`按 HOME 地面 Z=0 估算；巡航 ${data.heights.cruise} m，投放 ${data.heights.drop} m（投放下降不在图中）。实飞时每段从投放回升点重新生成，真实飞机还有跟踪误差。`+(data.backend==='fixed_min_snap'?'':' 当前不是 fixed_min_snap，虚线只连航点，不是实际轨迹。');
document.getElementById('stats').textContent=data.legs.map(g=>`航段${g.number}：步骤${String(g.first_step).padStart(2,'0')}～${String(g.last_step).padStart(2,'0')} → ${g.ends==='drop'?'投放':'降落'}，${g.duration_s.toFixed(1)} s，最大 ${g.max_velocity_mps.toFixed(2)} m/s${g.retimed?'（局部配时）':''}`).join('\\n')+(data.legs.length?`\\n航段合计 ${data.duration_s.toFixed(1)} s（不含起飞、对准、投放、降落）`:'');
document.getElementById('order').textContent=data.order.join('\\n');
const sl=document.getElementById('time');sl.max=Math.max(0,S.length-1);sl.value=0;
function update(){const s=S[Number(sl.value)];if(!s)return;marker.setAttribute('cx',px(s.p));marker.setAttribute('cy',py(s.p));mz(s);mv(s);
document.getElementById('point').textContent=`t=${s.t.toFixed(2)} s  航段${s.leg}  ${s.lap?'第'+s.lap+'圈':'返航'} · ${N[s.cat]}\\nXYZ=(${s.p.map(v=>v.toFixed(3)).join(', ')}) m  速度=${s.v.toFixed(3)} m/s`;}
sl.oninput=update;update();
</script></html>'''


def write_delivery_preview_html(path, config, document):
    value = delivery_preview_data(config, document)
    data = json.dumps(value, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(TEMPLATE.replace('__DATA__', data), encoding='utf-8')
    return str(target)


def default_preview_path(route_path):
    route_path = Path(route_path)
    return route_path.with_name(route_path.stem+'_preview.html')


def write_for_route(route_path, flight_config, output=None):
    """Resolve delivery-route parameters the same way as the flight entry and write the HTML."""
    from .flight_configuration import apply_flight_configuration, base_configuration
    from .guided_route import load_route
    config = apply_flight_configuration(base_configuration(None), flight_config, task='delivery-route')
    return write_delivery_preview_html(output or default_preview_path(route_path), config,
                                       load_route(route_path, require_complete=True))
