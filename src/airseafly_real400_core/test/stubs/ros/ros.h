#pragma once
// Transport doubles: never register a ROS node, publisher or hardware service.
#include <string>
#define ROS_INFO_STREAM(...) do {} while (0)
#define ROS_WARN_THROTTLE(...) do {} while (0)
namespace ros {
struct Duration { double value; double toSec() const { return value; } };
struct Time {
  double value=0;
  static Time now() { return {100.}; }
  double toSec() const { return value; }
  Duration operator-(const Time& other) const { return {value-other.value}; }
};
using WallTime = Time;
struct Publisher {
  unsigned getNumSubscribers() const { return 0; }
  template<class T> void publish(const T&) const {}
};
struct NodeHandle {
  template<class T> Publisher advertise(const std::string&, int) { return {}; }
};
}
