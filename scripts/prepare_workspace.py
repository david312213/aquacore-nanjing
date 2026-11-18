#!/usr/bin/env python3
"""Prepare internal catkin links. Does not build or launch any node."""
from pathlib import Path
import os
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    dest = ROOT / 'build_ws/src'
    dest.mkdir(parents=True, exist_ok=True)
    names = set()
    for manifest in sorted((ROOT / 'src').rglob('package.xml')):
        package = ET.parse(manifest).getroot().findtext('name')
        if package in names:
            raise ValueError('Duplicate ROS package: ' + package)
        names.add(package)
        link = dest / package
        relative = os.path.relpath(manifest.parent, dest)
        if link.is_symlink() and link.resolve() == manifest.parent.resolve():
            continue
        if link.exists() or link.is_symlink():
            raise ValueError('Existing build source differs: ' + str(link))
        link.symlink_to(relative, target_is_directory=True)
    print('Prepared %d ROS packages in %s' % (len(names), dest))


if __name__ == '__main__':
    main()
