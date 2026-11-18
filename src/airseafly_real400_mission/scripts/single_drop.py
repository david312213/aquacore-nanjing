#!/usr/bin/env python3
"""Recorded single target: takeoff, visual delivery, recovery, then hover."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.single_drop_cli import recorded_main

if __name__ == '__main__':
    try:
        sys.exit(recorded_main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        print('单投放未能继续：'+str(exc), file=sys.stderr)
        sys.exit(2)
