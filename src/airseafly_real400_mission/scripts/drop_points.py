#!/usr/bin/env python3
"""Recorded delivery test entry."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.drop_points_cli import main

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        print('录点投放测试未能继续：'+str(exc), file=sys.stderr)
        sys.exit(2)
