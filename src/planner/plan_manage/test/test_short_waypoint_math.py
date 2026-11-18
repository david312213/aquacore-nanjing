"""Compile the changed numeric branches, without ROS, Eigen or a flying vehicle.

The real planner source supplies both branches. The tiny vector adapter only
provides Eigen's elementary vector operations; this does not test optimization.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler unavailable')
class ShortWaypointMathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (Path(__file__).resolve().parents[1]/'src/planner_manager.cpp').read_text()
        begin = source.index('          Eigen::Vector3d horizen_dir =')
        axes = source[begin:source.index('          Eigen::Vector3d random_inserted_pt', begin)]
        code = r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
using std::cout; using std::endl;
namespace Eigen {
struct Vector3d {
 double x,y,z;
 Vector3d(double a,double b,double c):x(a),y(b),z(c){}
 static Vector3d UnitX(){return {1,0,0};}
 Vector3d operator-(Vector3d v)const{return {x-v.x,y-v.y,z-v.z};}
 Vector3d cross(Vector3d v)const{return {y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x};}
 double squaredNorm()const{return x*x+y*y+z*z;}
 double norm()const{return std::sqrt(squaredNorm());}
 void normalize(){double n=norm();x/=n;y/=n;z/=n;}
 Vector3d normalized()const{auto v=*this;v.normalize();return v;}
 double dot(Vector3d v)const{return x*v.x+y*v.y+z*v.z;}
 bool finite()const{return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z);}
};
}
void check_axes(Eigen::Vector3d start_pt,Eigen::Vector3d local_target_pt){
 AXES
 assert(horizen_dir.finite() && vertical_dir.finite());
 assert(std::abs(horizen_dir.norm()-1)<1e-9);
 assert(std::abs(vertical_dir.norm()-1)<1e-9);
 assert(std::abs(horizen_dir.dot(vertical_dir))<1e-9);
 assert(std::abs((start_pt-local_target_pt).dot(horizen_dir))<1e-9);
 assert(std::abs((start_pt-local_target_pt).dot(vertical_dir))<1e-9);
}
int main(int argc,char**argv){
  check_axes({1,2,.65},{1,2,.25});
  check_axes({1,2,.25},{1,2,.65});
  check_axes({1,2,.65},{1+1e-12,2,.25});
  check_axes({1,2,.65},{1.13,2,.65});
  check_axes({1,2,.65},{1.13,2.12,.4});
}
'''.replace('AXES', axes)
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        (path/'test.cpp').write_text(code)
        cls.binary = path/'test'
        subprocess.run(['c++', '-std=c++14', '-O2', str(path/'test.cpp'), '-o', str(cls.binary)],
                       check=True, capture_output=True, text=True)

    def test_vertical_and_near_vertical_restart_directions_are_finite(self):
        subprocess.run([str(self.binary), 'axes'], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
