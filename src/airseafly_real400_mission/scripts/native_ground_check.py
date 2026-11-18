#!/usr/bin/env python3
"""Read-only localization observations; no acceptance or active-fusion verdict.

Never arms, changes modes, or sends setpoints.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State


def summarize_samples(stamps, ages, statuses, states, duration):
    span=stamps[-1]-stamps[0] if len(stamps)>1 else 0.
    hz=(len(stamps)-1)/span if span>0. else 0.
    monotonic=all(math.isfinite(s) for s in stamps) and all(b>a for a,b in zip(stamps,stamps[1:]))
    # Freshness is used by the command watchdog in current bridges. Neither it
    # nor an older bridge's fusion_healthy=true proves active EV fusion.
    health_samples=[s['healthy'] for s in statuses if isinstance(s.get('healthy'), bool)]
    healthy=sum(health_samples)/len(health_samples) if health_samples else None
    error=[s['px4_error_m'] for s in statuses if isinstance(s.get('px4_error_m'), (int, float))
           and math.isfinite(s['px4_error_m'])]
    finite_ages=[a for a in ages if math.isfinite(a)]
    return dict(record_kind='native_localization_observations',
        duration_s=duration, output_samples=len(stamps), output_hz=hz,
        unique_monotonic=monotonic, min_sample_age_s=min(finite_ages,default=None),
        max_sample_age_s=max(finite_ages,default=None), nonfinite_age_samples=len(ages)-len(finite_ages),
        localization_healthy_fraction=healthy, health_status_samples=len(health_samples),
        health_is_gate=statuses[-1].get('health_is_gate') if statuses else None,
        fusion_healthy=None, fusion_source_verified=False,
        max_px4_external_disagreement_m=max(error,default=None), final_state=states[-1] if states else None,
        final_status=statuses[-1] if statuses else None, flight_or_payload_commands_sent=False,
        limitation=('Observations only; no acceptance verdict or real-time source-level fusion evidence. '
                    'Inspect health_basis for the bridge version; healthy does not establish position consistency.'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=10.)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0.: parser.error('sample duration must be positive')
    rospy.init_node('native_ground_check',anonymous=True)
    stamps,ages,statuses,states=[],[],[],[]
    lock=threading.Lock()
    def odom(msg):
        with lock:
            stamps.append(msg.header.stamp.to_sec()); ages.append(rospy.get_time()-stamps[-1])
    def status(msg):
        with lock: statuses.append(json.loads(msg.data))
    def state(msg):
        with lock: states.append(dict(armed=msg.armed,connected=msg.connected,mode=msg.mode))
    subs=[rospy.Subscriber('/mavros/odometry/out',Odometry,odom,queue_size=100),
          rospy.Subscriber('/airseafly/native/localization_status',String,status,queue_size=100),
          rospy.Subscriber('/mavros/state',State,state,queue_size=10)]
    until=time.monotonic()+args.seconds
    while time.monotonic()<until and not rospy.is_shutdown(): time.sleep(.05)
    for sub in subs: sub.unregister()
    with lock:
        result=summarize_samples(stamps,ages,statuses,states,args.seconds)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,ensure_ascii=False))
    return 0


if __name__=='__main__': raise SystemExit(main())
