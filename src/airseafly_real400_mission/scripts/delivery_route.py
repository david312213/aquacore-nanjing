#!/usr/bin/env python3
"""Entry point for the recorded two-lap delivery task."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.delivery_route_cli import main

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        print('配送任务未能继续：'+str(exc), file=sys.stderr)
        sys.exit(2)
