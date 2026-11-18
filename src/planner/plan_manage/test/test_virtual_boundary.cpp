#include <plan_env/virtual_boundary.h>
#include <cassert>
#include <algorithm>
int main(){
 ego_planner::VirtualBoundary b;
 b.corners={{{{0,0}},{{4,0}},{{4,3}},{{0,3}}}};
 assert(b.validate());assert(!b.occupied(-1,1,1,.2));b.enabled=true;
 assert(!b.occupied(2,1,1,.2));assert(b.occupied(-1,1,1,0));
 assert(b.occupied(.15,1,1,.2));assert(!b.occupied(.3,1,1,.2));
 assert(b.occupied(.3,1,1,.4));assert(b.occupied(4,1,1,0));
 std::reverse(b.corners.begin(),b.corners.end());assert(b.validate());
 assert(!b.occupied(2,1,1,.2));assert(b.occupied(.15,1,1,.2));
 b.origin_z=10;b.bottom=0;b.top=2;
 assert(!b.occupied(-1,1,9.9,.2));assert(b.occupied(-1,1,11,.2));assert(!b.occupied(-1,1,12.1,.2));
 // Rotation must not turn the boundary into an axis-aligned bounding box.
 b.bottom=-100;b.top=100;b.corners={{{{0,2}},{{2,0}},{{4,2}},{{2,4}}}};
 assert(b.validate());assert(!b.occupied(2,2,0,.2));assert(b.occupied(.1,.1,0,0));
 b.corners={{{{0,0}},{{4,3}},{{4,0}},{{0,3}}}};assert(!b.validate());
}
