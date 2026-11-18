#pragma once
#include <array>
#include <cmath>
#include <limits>

namespace ego_planner {
// Convex ordered perimeter. Outside is occupied, so a sampled trajectory
// cannot skip through a thin wall and treat the far side as free space.
struct VirtualBoundary {
  std::array<std::array<double, 2>, 4> corners{};
  bool enabled=false;
  double origin_z=0.;
  double bottom=-std::numeric_limits<double>::infinity();
  double top=std::numeric_limits<double>::infinity();
  double orientation=1.;
  bool validate() {
    double sign=0.;
    for (int i=0;i<4;++i) {
      const auto &a=corners[i], &b=corners[(i+1)%4], &c=corners[(i+2)%4];
      if (!std::isfinite(a[0]) || !std::isfinite(a[1])) return false;
      const double turn=(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]);
      if (std::abs(turn)<=1e-9 || (sign && sign*turn<=0.)) return false;
      sign=turn;
    }
    orientation=sign>0.?1.:-1.;
    return bottom<top;
  }
  bool occupied(double x,double y,double z,double inflation) const {
    if (!enabled || z<origin_z+bottom || z>origin_z+top) return false;
    for (int i=0;i<4;++i) {
      const auto &a=corners[i], &b=corners[(i+1)%4];
      const double dx=b[0]-a[0],dy=b[1]-a[1];
      const double inward=orientation*(dx*(y-a[1])-dy*(x-a[0]))/std::hypot(dx,dy);
      if (inward<=inflation) return true;
    }
    return false;
  }
};
}
