#include "../src/native_setpoint.h"
#include "../src/native_owner.h"
#include <cassert>
#include <initializer_list>

int main() {
  using namespace native_control;
  assert(supported_local_mask(0));
  assert(supported_local_mask(2496));
  // Position and yaw enabled; velocity, acceleration, yaw-rate ignored.
  const unsigned position_yaw = 8 | 16 | 32 | 64 | 128 | 256 | 2048;
  assert(position_yaw == kPositionYawMask);
  assert(supported_local_mask(position_yaw));
  for (unsigned bit : {1u, 2u, 4u, 512u, 1024u})
    assert(!supported_local_mask(position_yaw | bit));
  assert(!supported_local_mask(4095));
  // The existing single-owner gate forwards native local setpoints without
  // letting the attitude controller run, then restores it on legacy handoff.
  Gate gate;
  Health h;
  h.connected = h.pose = h.bridge = h.command = true;
  assert(gate.update(h) == Output::Attitude);
  assert(gate.acquire(h));
  assert(gate.update(h) == Output::Local);
  assert(output_allowed(gate.owner, WatchdogAction::Track, h, false));
  gate.owner = Owner::Legacy;
  assert(gate.update(h) == Output::Attitude);
}
