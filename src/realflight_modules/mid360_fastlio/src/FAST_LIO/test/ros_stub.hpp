#pragma once
// Minimal stand-ins for the ROS/Eigen/FAST-LIO symbols predictor_bridge.hpp uses.
// Lets the real bridge header compile and run in a timing simulation off-board.
#include <cmath>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ros {
struct Time {
  double s = 0;
  Time() = default;
  static Time now() { return Time(); }
  Time fromSec(double v) const { Time t; t.s = v; return t; }
  double toSec() const { return s; }
};
}  // namespace ros
namespace geometry_msgs { struct V3 { double x = 0, y = 0, z = 0; }; struct Q4 { double x = 0, y = 0, z = 0, w = 1; }; }
namespace std_msgs { struct String { std::string data; }; }
namespace nav_msgs {
struct Odometry {
  struct { ros::Time stamp; std::string frame_id; } header;
  std::string child_frame_id;
  struct { struct { geometry_msgs::V3 position; geometry_msgs::Q4 orientation; } pose; std::array<double,36> covariance{}; } pose;
  struct { struct { geometry_msgs::V3 linear, angular; } twist; } twist;
};
}  // namespace nav_msgs
namespace sensor_msgs {
struct Imu {
  struct { ros::Time stamp; } header;
  geometry_msgs::V3 linear_acceleration, angular_velocity;
  typedef std::shared_ptr<const Imu> ConstPtr;
};
}  // namespace sensor_msgs

// Every publisher records into a topic-keyed store for the test to inspect.
std::map<std::string, std::vector<nav_msgs::Odometry>> g_odom;
std::map<std::string, std::vector<std::string>> g_text;
std::map<std::string, std::string> g_params;
namespace ros {
struct Publisher {
  std::string topic;
  void publish(const nav_msgs::Odometry &m) const { g_odom[topic].push_back(m); }
  void publish(const std_msgs::String &m) const { g_text[topic].push_back(m.data); }
};
struct NodeHandle {
  template <class T> void param(const std::string &k, T &v, const T &d) {
    auto it = g_params.find(k);
    v = d;
    if (it != g_params.end()) set(v, it->second);
  }
  template <class M> Publisher advertise(const std::string &t, int) { return Publisher{t}; }
 private:
  static void set(std::string &v, const std::string &s) { v = s; }
  static void set(double &v, const std::string &s) { v = std::stod(s); }
  static void set(int &v, const std::string &s) { v = std::stoi(s); }
};
}  // namespace ros
#define ROS_WARN(...) ((void)0)
#define ROS_WARN_THROTTLE(r, ...) ((void)0)

// FAST-LIO globals referenced by the bridge.
#define G_m_s2 (9.81)
double time_diff_lidar_to_imu = 0.0, timediff_lidar_wrt_imu = 0.0, lidar_end_time = 0.0;
bool time_sync_en = false;
struct StatePoint {
  Eigen::Vector3d pos=Eigen::Vector3d::Zero(), vel=Eigen::Vector3d::Zero(),
      ba=Eigen::Vector3d::Zero(), bg=Eigen::Vector3d::Zero(), grav=Eigen::Vector3d(0,0,-9.81);
  Eigen::Quaterniond rot=Eigen::Quaterniond::Identity();
  void S2_Mx(Eigen::Matrix<double,3,2> &out, Eigen::Matrix<double,2,1>, int index) {
    if (index!=21) throw std::invalid_argument("gravity state index");
    out << 0, 9.81, -9.81, 0, 0, 0;
  }
} state_point;
struct FilterStub {
  Eigen::Matrix<double,23,23> covariance=Eigen::Matrix<double,23,23>::Identity()*.002;
  Eigen::Matrix<double,23,23> get_P() const { return covariance; }
} kf;
struct ImuProcessStub {
  double scale = 1.0;
  Eigen::Matrix<double,12,12> Q=Eigen::Matrix<double,12,12>::Identity()*.0001;
  bool prediction_accel_scale(double &s) const { s = scale; return true; }
};
std::unique_ptr<ImuProcessStub> p_imu(new ImuProcessStub());
