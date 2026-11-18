#!/usr/bin/env python3
"""Import files from an explicitly selected local aircraft installation.

No SSH, ROS, FCU parameter writes or vehicle commands. Backups go to data/.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', type=Path, default=Path.home(), help='机载用户目录')
    parser.add_argument('--project', type=Path, help='可选：机载原项目，导入控制参数与标定')
    parser.add_argument('--apply', action='store_true', help='复制文件；默认只列出差异')
    args = parser.parse_args()
    pairs = [(args.home/'.config/airseafly/MID360_config.json', 'config/aircraft/MID360_config.json'),
             (args.home/'.ros/camera_info/df200_calib.yaml', 'config/aircraft/df200_calib.yaml')]
    if args.project:
        for source, dest in (
            ('src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml', 'config/px4ctrl.yaml'),
            ('src/airseafly_real400/config/frames.yaml', 'config/frames.yaml'),
            ('src/realflight_modules/mid360_fastlio/src/FAST_LIO/config/mid360.yaml', 'config/fastlio.yaml')):
            pairs.append((args.project/source, dest))
    report = []
    backup = ROOT/'data/config_import'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    for source, rel in pairs:
        dest = ROOT/rel
        item = dict(source=str(source), destination=rel, available=source.is_file())
        if source.is_file():
            data = source.read_bytes()
            item['sha256'] = hashlib.sha256(data).hexdigest()
            item['changed'] = not dest.is_file() or dest.read_bytes() != data
            if args.apply and item['changed']:
                if dest.is_file():
                    saved = backup/rel
                    saved.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(dest, saved)
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)
                item['applied'] = True
        report.append(item)
    if args.apply:
        backup.mkdir(parents=True, exist_ok=True)
        (backup/'import.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
