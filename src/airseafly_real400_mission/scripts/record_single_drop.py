#!/usr/bin/env python3
"""Record the one single-drop target; reads localization only."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from record_waypoints import main

if __name__ == '__main__':
    try:
        sys.exit(main(single=True))
    except KeyboardInterrupt:
        sys.exit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        print('单投放点录制未能继续：'+str(exc), file=sys.stderr)
        sys.exit(2)
