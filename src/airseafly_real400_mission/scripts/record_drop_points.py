#!/usr/bin/env python3
"""Record up to four delivery targets; reads localization only."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from record_waypoints import main

if __name__ == '__main__':
    try:
        sys.exit(main(delivery=True))
    except KeyboardInterrupt:
        sys.exit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        print('投放点录制未能继续：'+str(exc), file=sys.stderr)
        sys.exit(2)
