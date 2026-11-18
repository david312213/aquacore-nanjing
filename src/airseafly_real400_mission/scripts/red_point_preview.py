#!/usr/bin/env python3
"""Observe red points; no controller or payload process is started here."""
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT/'src/airseafly_real400_mission/src'))

from airseafly_real400_mission.red_point_preview import main

if __name__ == '__main__':
    sys.exit(main())
