// RViz-only map preview. No goals, planning FSM, trajectory server or flight commands.
#include <plan_env/grid_map.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

class MapPreview
{
public:
  explicit MapPreview(ros::NodeHandle nh) : nh_(nh)
  {
    nh_.param<std::string>("grid_map/frame_id", frame_, "fast_lio_local_enu");
    nh_.param("grid_map/resolution", preview_resolution_, 0.15);
    if (!std::isfinite(preview_resolution_) || preview_resolution_ <= 0.)
      throw std::runtime_error("Invalid map preview resolution");
    cloud_out_ = nh_.advertise<sensor_msgs::PointCloud2>("/airseafly/rviz/occupancy_inflate", 1);
    marker_out_ = nh_.advertise<visualization_msgs::Marker>("/airseafly/rviz/occupancy_inflate_marker", 1);
    preview_sub_ = nh_.subscribe("grid_map/occupancy_inflate", 1, &MapPreview::onPreview, this);
    actual_sub_ = nh_.subscribe("/drone_0_ego_planner_node/grid_map/occupancy_inflate", 1,
                                &MapPreview::onActual, this);
    cloud_sub_ = nh_.subscribe("grid_map/cloud", 1, &MapPreview::onCloud, this);
    odom_sub_ = nh_.subscribe("grid_map/odom", 1, &MapPreview::onOdom, this,
                             ros::TransportHints().tcpNoDelay());
    timer_ = nh_.createWallTimer(ros::WallDuration(0.2), &MapPreview::tick, this);
    startPreview();
  }

private:
  static constexpr double source_timeout_ = 0.75;
  static constexpr double actual_timeout_ = 1.0;
  ros::NodeHandle nh_;
  std::unique_ptr<GridMap> map_;
  ros::Publisher cloud_out_, marker_out_;
  ros::Subscriber preview_sub_, actual_sub_, cloud_sub_, odom_sub_;
  ros::WallTimer timer_;
  ros::WallTime cloud_received_, odom_received_, actual_received_;
  ros::Time cloud_stamp_, odom_stamp_;
  std::string frame_;
  double preview_resolution_;
  bool visible_ = false;

  bool fresh(const ros::WallTime &received, const ros::Time &stamp) const
  {
    const double age = (ros::Time::now() - stamp).toSec();
    return !received.isZero() && !stamp.isZero() &&
           (ros::WallTime::now() - received).toSec() < source_timeout_ &&
           age >= -0.05 && age < source_timeout_;
  }

  bool sourcesFresh() const
  {
    return fresh(cloud_received_, cloud_stamp_) && fresh(odom_received_, odom_stamp_);
  }

  bool actualFresh() const
  {
    // GridMap's PCL output currently has stamp=0; use receipt time for source selection.
    // Sensor timestamps are checked independently, so republishing old maps cannot mask loss.
    return !actual_received_.isZero() &&
           (ros::WallTime::now() - actual_received_).toSec() < actual_timeout_;
  }

  void onCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (msg->header.frame_id != frame_) return;
    cloud_received_ = ros::WallTime::now();
    cloud_stamp_ = msg->header.stamp;
  }

  void onOdom(const nav_msgs::OdometryConstPtr &msg)
  {
    if (msg->header.frame_id != frame_) return;
    odom_received_ = ros::WallTime::now();
    odom_stamp_ = msg->header.stamp;
  }

  static std_msgs::ColorRGBA heightColor(double relative_height)
  {
    // Match RViz AxisColor: automatic Z bounds, red at the bottom, magenta at the top.
    // https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/point_cloud_transformers.cpp
    const double hue = 5. * std::max(0., std::min(1., relative_height));
    std_msgs::ColorRGBA color;
    color.r = std::max(0., std::min(1., std::abs(hue - 3.) - 1.));
    color.g = std::max(0., std::min(1., 2. - std::abs(hue - 2.)));
    color.b = std::max(0., std::min(1., 2. - std::abs(hue - 4.)));
    color.a = 1.;
    return color;
  }

  bool publish(const sensor_msgs::PointCloud2ConstPtr &msg, double resolution)
  {
    if (msg->header.frame_id != frame_ || !std::isfinite(resolution) || resolution <= 0.)
      return false;
    visualization_msgs::Marker marker;
    marker.header = msg->header;
    marker.ns = "occupancy_inflate";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.;
    marker.scale.x = marker.scale.y = marker.scale.z = resolution;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.;
    marker.lifetime = ros::Duration(0.6);
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    try
    {
      if (msg->width && msg->height)
      {
        sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
        marker.points.reserve(size_t(msg->width) * msg->height);
        for (; x != x.end(); ++x, ++y, ++z)
        {
          if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
          geometry_msgs::Point p;
          p.x = *x; p.y = *y; p.z = *z;
          marker.points.push_back(p);
          min_z = std::min(min_z, p.z);
          max_z = std::max(max_z, p.z);
        }
      }
    }
    catch (const std::runtime_error &error)
    {
      ROS_WARN_THROTTLE(2., "Cannot display inflated map: %s", error.what());
      return false;
    }
    if (!marker.points.empty())
    {
      const double range = max_z > min_z ? max_z - min_z : 0.001;
      marker.colors.reserve(marker.points.size());
      for (const auto &point : marker.points)
        marker.colors.push_back(heightColor((point.z - min_z) / range));
    }
    cloud_out_.publish(msg);
    marker_out_.publish(marker);
    visible_ = true;
    return true;
  }

  void onPreview(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (map_ && !actualFresh() && sourcesFresh()) publish(msg, preview_resolution_);
  }

  void onActual(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (!sourcesFresh()) return;
    double resolution;
    if (!nh_.getParamCached("/drone_0_ego_planner_node/grid_map/resolution", resolution)) return;
    if (!publish(msg, resolution)) return;
    actual_received_ = ros::WallTime::now();
    // One single-threaded ROS spinner: no GridMap callbacks can be running during destruction.
    // Release preview buffers and subscriptions while the real planner provides the map.
    if (map_)
    {
      map_.reset();
      ROS_INFO("RViz now displays the active EGO map; preview mapping stopped.");
    }
  }

  void clearDisplay()
  {
    if (!visible_) return;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_;
    marker.ns = "occupancy_inflate";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_out_.publish(marker);
    sensor_msgs::PointCloud2 empty;
    empty.header.frame_id = frame_;
    empty.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud2Modifier fields(empty);
    fields.setPointCloud2FieldsByString(1, "xyz");
    fields.resize(0);
    cloud_out_.publish(empty);
    visible_ = false;
  }

  void startPreview()
  {
    clearDisplay();
    map_.reset(new GridMap);
    map_->initMap(nh_);
    ROS_INFO("RViz map preview ready; waiting for localization/cloud, no goal required.");
  }

  void tick(const ros::WallTimerEvent &)
  {
    if (!sourcesFresh()) clearDisplay();
    if (!actualFresh() && !map_) startPreview();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_map_preview");
  ros::NodeHandle nh("~");
  MapPreview preview(nh);
  ros::spin();
  return 0;
}
