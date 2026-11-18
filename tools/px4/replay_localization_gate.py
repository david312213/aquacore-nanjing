#!/usr/bin/env python3
"""Replay recorded flights through the localization gate; read-only, no ROS.

Feeds each data/mission/*/alignment_control.jsonl through the same DivergenceGate
the onboard node uses. Armed/OFFBOARD comes from the recorded px4ctrl feedback.
The trace samples localization at about 2 Hz (live status is 50 Hz), so a replayed
trip can be up to one sample later than the live gate would trip.

    python3 tools/px4/replay_localization_gate.py [mission_dir ...]
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/airseafly_real400_mission/src'))
from airseafly_real400_mission.localization_gate import DivergenceGate  # noqa: E402


def replay(path, max_error_m, hold_s):
    gate = DivergenceGate(max_error_m, hold_s)
    armed = offboard = False
    t0 = trip = None
    peak = 0.
    flown = False
    with open(path) as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except ValueError:
                continue
            native = ((record.get('feedback') or {}).get('px4_native') or {}).get('values') or {}
            if native:
                was_armed = armed
                armed, offboard = bool(native.get('armed')), bool(native.get('offboard'))
                if was_armed and not armed:
                    gate.reset()
                flown = flown or (armed and offboard)
            if record.get('record_type') != 'diagnostic' or record.get('event') != 'native_localization':
                continue
            values = (record.get('diagnostic') or {}).get('values') or {}
            stamp, error = values.get('stamp'), values.get('px4_error_m')
            if t0 is None and stamp is not None:
                t0 = stamp
            active = armed and offboard
            if active and error is not None:
                peak = max(peak, error)
            if gate.update(stamp, error, active) and trip is None:
                trip = (stamp - t0, error)
    return dict(flown=flown, peak=peak, trip=trip)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('missions', nargs='*', type=Path)
    parser.add_argument('--max-error', type=float, default=.5)
    parser.add_argument('--hold', type=float, default=.5)
    args = parser.parse_args()
    missions = args.missions or sorted((ROOT/'data/mission').glob('2026*'))
    print('gate: > %.2f m held %.2f s while armed in OFFBOARD' % (args.max_error, args.hold))
    for mission in missions:
        path = mission/'alignment_control.jsonl'
        if not path.is_file():
            continue
        try:
            result = replay(path, args.max_error, args.hold)
        except (OSError, ValueError) as exc:
            print('%-24s unreadable: %s' % (mission.name, exc))
            continue
        if not result['flown']:
            print('%-24s not flown in OFFBOARD' % mission.name)
            continue
        trip = result['trip']
        print('%-24s peak %5.2f m  %s' % (mission.name, result['peak'],
              'TRIP at %.1f s (%.2f m)' % trip if trip else 'no trip'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
