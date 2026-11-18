#pragma once
#include <Eigen/Eigen>
#include <memory>
#include <vector>
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
namespace fast_planner {
class ObjPredictor {
public:
  using Ptr = std::shared_ptr<ObjPredictor>;
  int getObjNums() { return 0; }
  Eigen::Vector3d evaluateConstVel(int, double) { return Eigen::Vector3d::Zero(); }
};
}
