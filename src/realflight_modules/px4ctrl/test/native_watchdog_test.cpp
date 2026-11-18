#include "../src/native_owner.h"
#include <cassert>
#include <initializer_list>
using namespace native_control;

Health healthy() {
  Health h;
  h.connected=h.armed=h.offboard=h.pose=h.estimator=h.bridge=h.command=true;
  return h;
}

int main() {
  FreshnessWatchdog w;
  auto h=healthy();
  assert(w.update(h)==WatchdogAction::Track);
  assert(output_allowed(Owner::Native,w.action,h,false));
  // Diagnostic estimator flags alone do not stop ordinary tracking.
  h.estimator=false;
  assert(w.update(h)==WatchdogAction::Track);
  h.estimator=true;
  h.command=false;
  assert(w.update(h)==WatchdogAction::Track);
  assert(output_allowed(Owner::Native,w.action,h,false));
  h.pose=h.bridge=false;
  for (int i=0; i<1000; ++i) {
    assert(w.update(h)==WatchdogAction::Track);
    assert(output_allowed(Owner::Native,w.action,h,false));
  }
  h.command=true;
  h.pose=h.bridge=true;
  assert(w.update(h)==WatchdogAction::Track); // Recovery needs no acquisition.
  assert(output_allowed(Owner::Hold,w.action,h,false));
  assert(output_allowed(Owner::AutoLand,w.action,h,true));
  assert(!output_allowed(Owner::AutoLand,w.action,h,false)); // Bounded RPC wait.
  h.pose=false;
  assert(!output_allowed(Owner::AutoLand,w.action,h,true)); // Never hold a stale pose.
  h=healthy();
  assert(w.update(h)==WatchdogAction::Track);
  h.auto_land=true;
  assert(!output_allowed(Owner::AutoLand,w.action,h,true));
  w.reset();
  assert(w.update(h)==WatchdogAction::Track);

  // Invalid estimator flags during an explicitly selected hold still matter.
  h=healthy(); h.command=false;
  assert(w.update(h,true)==WatchdogAction::Track);
  h.estimator=false;
  assert(w.update(h,true)==WatchdogAction::Land);
  w.reset();
  assert(w.update(h,true)==WatchdogAction::Land);

  for (bool pose_loss : {false,true}) {
    w.reset(); h=healthy();
    h.armed=false;
    if (pose_loss) h.pose=false; else h.bridge=false;
    assert(w.update(h)==WatchdogAction::Track);
    assert(output_allowed(Owner::Native,w.action,h,true));
    h=healthy();
    assert(w.update(h)==WatchdogAction::Track);
  }
  w.reset(); h=healthy(); h.connected=false;
  assert(w.update(h)==WatchdogAction::Stop);
  h=healthy();
  assert(w.update(h)==WatchdogAction::Stop); // Actual connection loss remains latched.
  assert(!output_allowed(Owner::Native,w.action,h,true));

  // An explicit hold keeps its captured point through ordinary source-age gaps.
  w.reset(); h=healthy(); h.command=false;
  for (int i=0; i<1000; ++i) assert(w.update(h,true)==WatchdogAction::Track);
  h.bridge=false;
  h.pose=false;
  assert(w.update(h,true)==WatchdogAction::Track);
  assert(output_allowed(Owner::Hold,w.action,h,false));

  // The pilot's OFFBOARD exit still latches after an explicit hold selection.
  ModeExitLatch mode;
  h=healthy();
  assert(!mode.update(Owner::Native,h));
  assert(!mode.update(Owner::Hold,h));
  h.offboard=false;
  assert(mode.update(Owner::Hold,h));
  h.offboard=true;
  assert(mode.update(Owner::Hold,h));
}
