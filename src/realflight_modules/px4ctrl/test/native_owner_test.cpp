#include "../src/native_owner.h"
#include <cassert>
using namespace native_control;
int main() {
  Health h;
  Gate g;
  assert(g.update(h)==Output::Attitude);
  // Ownership is available even when every diagnostic flag is false.
  assert(g.acquire(h));
  assert(g.update(h)==Output::Local && g.owner==Owner::Native);
  h.connected=h.armed=h.offboard=h.pose=h.estimator=h.bridge=h.rc=h.command=true;
  assert(g.update(h)==Output::Local && g.owner==Owner::Native);
  h.connected=h.armed=h.offboard=h.pose=h.estimator=h.bridge=h.rc=h.command=false;
  for (int i=0; i<100; ++i)
    assert(g.update(h)==Output::Local && g.owner==Owner::Native);
  g.owner=Owner::Hold;
  assert(g.update(h)==Output::Local && g.owner==Owner::Hold);
  assert(g.acquire(h) && g.owner==Owner::Native);
  g.owner=Owner::AutoLand;
  assert(g.update(h)==Output::Local);
  h.auto_land=true;
  assert(g.update(h)==Output::None && g.owner==Owner::AutoLand);
  h.auto_land=false;
  assert(g.update(h)==Output::Local && g.owner==Owner::AutoLand);
  g.owner=Owner::Legacy;
  assert(g.update(h)==Output::Attitude);

  ModeExitLatch mode;
  h = Health();
  assert(!mode.update(Owner::Native,h)); // Before OFFBOARD, prestream is allowed.
  h.offboard = true;
  assert(!mode.update(Owner::Native,h));
  h.offboard = false;
  assert(mode.update(Owner::Native,h));
  h.offboard = true;
  assert(mode.update(Owner::Native,h)); // Re-entering the mode cannot resume.
  mode.reset();
  assert(!mode.update(Owner::Native,h));

  mode.start_landing();
  for (int i=0; i<100; ++i)
    assert(!mode.update(Owner::AutoLand,h)); // Slow mode RPC keeps its hold.
  h.offboard = false;
  assert(!mode.update(Owner::AutoLand,h)); // Still awaiting LAND confirmation.
  h.auto_land = true;
  assert(!mode.update(Owner::AutoLand,h)); // Entering LAND is not a manual exit.
  h.auto_land = false;
  assert(mode.update(Owner::AutoLand,h)); // Pilot leaves confirmed LAND.
  h.offboard = true;
  assert(mode.update(Owner::AutoLand,h)); // OFFBOARD alone never restores output.
  h.auto_land = true;
  assert(mode.update(Owner::AutoLand,h)); // Nor does re-entering AUTO.LAND.
  mode.start_landing();
  assert(mode.paused); // A repeated landing request cannot clear a manual pause.
  mode.reset(); // Only explicit native acquisition clears the pause.
  h.auto_land = false;
  assert(!mode.update(Owner::Native,h));
}
