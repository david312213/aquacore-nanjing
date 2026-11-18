#include <plan_manage/short_trajectory.h>
#include <bspline_opt/trajectory_collision.h>
#include <cassert>
#include <iostream>
using namespace ego_planner;
using Eigen::Vector3d;

int main() {
  int cases = 0;
  for (double distance : {0., 1e-10, .18, .20, .22, .25, .40})
    for (double vx : {-.12, 0., .12, .30})
      for (double angle : {0., 1.57}) {
        Vector3d start(2., -3., 1.), end = start + Vector3d(distance * cos(angle), distance * sin(angle), 0.);
        Vector3d velocity(vx, 0., 0.), acceleration = Vector3d::Zero();
        double dt = .01;
        auto points = shortControlPoints(start, velocity, acceleration, end, dt);
        // Include an optimizer displacement of the free middle control points.
        points.col(4).y() += .015;
        assert(retimeShortControlPoints(points, dt, start, velocity, acceleration, end, .3, .5, 0.));
        UniformBspline curve(points, 3, dt);
        auto v = curve.getDerivative();
        auto a = v.getDerivative();
        double duration = curve.getTimeSum(), ratio = 0., collision = 0.;
        assert((curve.evaluateDeBoorT(0.) - start).norm() < 1e-9);
        assert((v.evaluateDeBoorT(0.) - velocity).norm() < 1e-9);
        assert((a.evaluateDeBoorT(0.) - acceleration).norm() < 1e-8);
        assert((curve.evaluateDeBoorT(duration) - end).norm() < 1e-9);
        assert(v.evaluateDeBoorT(duration).norm() < 1e-9);
        curve.setPhysicalLimits(.3, .5, 0.);
        assert(curve.checkFeasibility(ratio, false));
        for (int i = 0; i <= 1000; ++i) {
          assert(v.evaluateDeBoorT(duration * i / 1000.).cwiseAbs().maxCoeff() <= .30011);
          assert(a.evaluateDeBoorT(duration * i / 1000.).cwiseAbs().maxCoeff() <= .50011);
        }
        assert(!firstTrajectoryCollision(curve, .1, [](const Vector3d &) {return false;}, collision));
        assert(firstTrajectoryCollision(curve, .1, [&](const Vector3d &p) {return (p - end).norm() < .001;}, collision));
        ++cases;
      }
  Vector3d p(1.,2.,3.), q(1.25,2.,3.);
  for (auto input : {std::vector<Vector3d>{p}, std::vector<Vector3d>{p,p,p,p},
                     std::vector<Vector3d>{p,p,q,q,q}, std::vector<Vector3d>{p,q,q,p,p}}) {
    const auto output = resamplePolyline(input, .4);
    assert(output.size() >= 7);
    assert((output.front() - input.front()).norm() < 1e-9);
    assert((output.back() - input.back()).norm() < 1e-9);
    for (const auto &point : output) assert(point.allFinite());
  }
  std::cout << "PASS: " << cases << " short-spline endpoint/dynamics/collision cases and repeated-point resampling\n";
}
