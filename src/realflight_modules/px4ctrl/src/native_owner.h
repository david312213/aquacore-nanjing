#pragma once

// ROS-independent output arbitration. Diagnostics do not change ownership.
namespace native_control {
enum class Owner { Legacy, Native, Hold, AutoLand };
enum class Output { None, Attitude, Local };
struct Health {
  bool connected = false, armed = false, offboard = false, auto_land = false;
  bool pose = false, estimator = false, bridge = false, rc = false;
  bool command = false;
};

// Only a confirmed mode followed by an exit pauses output. In particular,
// requesting AUTO.LAND must keep the hold stream alive until PX4 enters it.
class ModeExitLatch {
  bool saw_offboard = false, saw_auto_land = false;
 public:
  bool paused = false;
  void reset() { saw_offboard = saw_auto_land = paused = false; }
  void start_landing() { saw_auto_land = false; }
  bool update(Owner owner, const Health& h) {
    if (owner == Owner::Native || owner == Owner::Hold) {
      if (h.offboard) saw_offboard = true;
      else if (saw_offboard) paused = true;
    } else if (owner == Owner::AutoLand) {
      if (h.auto_land) saw_auto_land = true;
      else if (saw_auto_land) paused = true;
    }
    return paused;
  }
};

class Gate {
 public:
  Owner owner = Owner::Legacy;
  bool acquire(const Health&) {
    owner = Owner::Native;
    return true;
  }
  Output update(const Health& h) {
    if (owner == Owner::Legacy) return Output::Attitude;
    if (owner == Owner::AutoLand && h.auto_land) return Output::None;
    return Output::Local;
  }
};

enum class WatchdogAction { Track, Land, Stop };

// Timestamp-age flags are diagnostic during tracking. Actual connection faults
// remain latched; an explicitly requested hold still needs a valid estimator.
class FreshnessWatchdog {
 public:
  WatchdogAction action = WatchdogAction::Track;
  void reset() { action = WatchdogAction::Track; }
  WatchdogAction update(const Health& h, bool intentional_hold=false) {
    if (action==WatchdogAction::Land || action==WatchdogAction::Stop) return action;
    if (!h.connected) return action=WatchdogAction::Stop;
    if (intentional_hold && !h.estimator)
      return action=(h.armed && h.offboard) ? WatchdogAction::Land : WatchdogAction::Stop;
    return action;
  }
};

inline bool output_allowed(Owner owner, WatchdogAction action, const Health& h, bool landing_wait_fresh) {
  if (owner==Owner::Native)
    // NativeControl initialized a validated point on acquisition and keeps it
    // until a new command arrives. A 0.30 s data gap cannot cancel this stream.
    return action==WatchdogAction::Track && h.connected;
  if (owner==Owner::Hold)
    return action!=WatchdogAction::Land && action!=WatchdogAction::Stop &&
      h.connected && h.estimator;
  if (owner==Owner::AutoLand)
    return !h.auto_land && h.connected && h.pose && h.estimator && landing_wait_fresh;
  return false;
}
}  // namespace native_control
