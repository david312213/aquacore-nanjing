"""Perimeter recording and immutable mission snapshot tests."""
from pathlib import Path
import sys,tempfile,subprocess,unittest
P=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(P/'src'))
from airseafly_real400_mission.virtual_boundary import *
from airseafly_real400_mission.waypoint_route import validate_route

class BoundaryTests(unittest.TestCase):
    def test_order_orientation_and_invalid_shapes(self):
        corners=[[0,0],[4,0],[4,3],[0,3]]
        self.assertEqual(validate_corners(corners),corners)
        self.assertEqual(validate_corners(corners[::-1]),corners[::-1])
        for c in (corners[:3],corners+[corners[0]],[[0,0],[4,3],[4,0],[0,3]],[[0,0],[1,0],[2,0],[0,3]],[[0,0],[4,0],[1,1],[0,3]]):
            with self.assertRaises(ValueError):validate_corners(c)
    def test_record_save_resume_without_overwriting_flight_route(self):
        r=BoundaryRecorder()
        for xy in ((0,0),(4,0),(4,3),(0,3)):
            r.observe_xy(*xy);r.key(' ')
        self.assertEqual(r.key(' ')['kind'],'ignored')
        self.assertEqual(r.key('2')['kind'],'ignored')
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'boundary.yaml';save_recording(p,r.to_dict())
            before=p.read_bytes();r.key('x')
            with self.assertRaises(ValueError):save_recording(p,r.to_dict())
            self.assertEqual(p.read_bytes(),before)
            self.assertEqual(load_recording(p)['points'],[[0,0],[4,0],[4,3],[0,3]])
            with self.assertRaises(ValueError):validate_route(yaml.safe_load(before))
    def test_snapshot_and_disabled_no_file_required(self):
        c={'planning':{'virtual_boundary':{'enabled':False,'file':'missing'}}}
        freeze_boundary(c,'.');self.assertEqual(launch_arguments(c),[])
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'boundary.yaml';doc={'points':[[0,0],[4,0],[4,3],[0,3]]};save_recording(p,doc)
            c={'planning':{'virtual_boundary':{'enabled':True,'file':'boundary.yaml','bottom_m':None,'top_m':2}}}
            freeze_boundary(c,d);args=launch_arguments(c);p.unlink()
            self.assertEqual(launch_arguments(c),args)
            self.assertIn('boundary_top:=2',args)
            self.assertEqual(c['_virtual_boundary']['source']['path'],str(p.resolve()))
    def test_real_record_entry_demo(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'boundary.yaml'
            r=subprocess.run([sys.executable,str(P/'scripts/record_boundary.py'),'--demo','--stdin','--output',str(p)],input='xy 0 0\nspace\nxy 4 0\nspace\nxy 4 3\nspace\nxy 0 3\nspace\nq\n',text=True,capture_output=True)
            self.assertEqual(r.returncode,0,r.stderr)
            self.assertEqual(read_boundary(p)[0],[[0,0],[4,0],[4,3],[0,3]])
if __name__=='__main__':unittest.main()
