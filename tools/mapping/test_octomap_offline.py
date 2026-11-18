#!/usr/bin/env python3
"""Small synthetic contract checks; requires the compiled offline builder."""
import csv
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

builder = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory(prefix='octomap_test_') as directory:
    root = Path(directory)
    stream = root/'scans.bin'
    with stream.open('wb') as f:
        # Shifted world origin: ray must end at X=12, not at X=2 or X=22.
        for _ in range(4):
            f.write(struct.pack('<Iffffff', 1, 10., 0., 0., 12., 0., 0.))
    subprocess.run([builder, str(stream), str(root/'test'), '.1', '15'], check=True)
    with (root/'test_occupied.csv').open() as f:
        occupied = list(csv.DictReader(f))
    assert len(occupied) == 1 and abs(float(occupied[0]['x'])-12.) < .11
    report = json.loads((root/'test_build.json').read_text())
    assert report['free_leaves'] > 0 and report['reload_ok']
    # Beyond max range: truncate free ray, do not invent an obstacle at its cutoff.
    subprocess.run([builder, str(stream), str(root/'limited'), '.1', '1'], check=True)
    limited = json.loads((root/'limited_build.json').read_text())
    assert limited['occupied_leaves'] == 0 and limited['free_leaves'] > 0
    (root/'empty.bin').write_bytes(b'')
    assert subprocess.run([builder, str(root/'empty.bin'), str(root/'empty'), '.1', '15']).returncode != 0
    (root/'truncated.bin').write_bytes(struct.pack('<I', 1))
    assert subprocess.run([builder, str(root/'truncated.bin'), str(root/'bad'), '.1', '15']).returncode != 0
print('PASS: translated origin, free rays, range truncation, reload, empty/truncated rejection')
