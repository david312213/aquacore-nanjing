#pragma once
// Offline transport doubles only. Planning/optimization/B-splines are REAL code.
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <string>
#include <unordered_map>
#define ROS_INFO(...) do {} while (0)
#define ROS_WARN(...) do { std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#define ROS_ERROR(...) ROS_WARN(__VA_ARGS__)
namespace ros {
struct Duration {
  double seconds = 0;
  Duration(double value = 0) : seconds(value) {}
  double toSec() const { return seconds; }
  Duration operator+(Duration other) const { return Duration(seconds + other.seconds); }
};
struct Time {
  double seconds = 0;
  Time(double value = 0) : seconds(value) {}
  static Time now() {
    return Time(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
  }
  double toSec() const { return seconds; }
  Duration operator-(Time other) const { return Duration(seconds - other.seconds); }
};
struct NodeHandle {
  std::unordered_map<std::string, double> values;
  template<class T> void param(const std::string &name, T &target, const T &fallback) {
    auto item = values.find(name);
    target = item == values.end() ? fallback : static_cast<T>(item->second);
  }
};
}
