#!/usr/bin/env python3
"""Source-tree and catkin entry for the ROS-independent native flight report."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from airseafly_real400_mission.native_flight_report import main

if __name__ == '__main__':
    raise SystemExit(main())
