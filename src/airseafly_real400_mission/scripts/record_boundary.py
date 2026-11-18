#!/usr/bin/env python3
from record_waypoints import main
if __name__ == '__main__':
    try:
        raise SystemExit(main(boundary=True))
    except Exception as exc:
        import sys
        print('边界未保存：'+str(exc),file=sys.stderr)
        raise SystemExit(2)
