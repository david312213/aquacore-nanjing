#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <airseafly_real400/core.hpp>
#include <airseafly_real400_msgs/LioState.h>
#include <map>
namespace ar=airseafly_real400;
template<class T> ar::V3 vec(const T& v) { return {v.x,v.y,v.z}; }
template<class T> void vecout(T& v,const ar::V3& p) { v.x=p.x();v.y=p.y();v.z=p.z(); }
ar::Q quat(const geometry_msgs::Quaternion& q) { return ar::normalized({q.w,q.x,q.y,q.z}); }
void quatout(geometry_msgs::Quaternion& o,const ar::Q& q) {o.x=q.x();o.y=q.y();o.z=q.z();o.w=q.w();}
ar::State state(const airseafly_real400_msgs::LioState& m) {
  ar::State s;s.t=m.header.stamp.toSec();s.p=vec(m.pose.position);s.q=quat(m.pose.orientation);
  s.v=vec(m.velocity_world);s.gravity=vec(m.gravity_world);s.bg=vec(m.gyro_bias);
  s.ba=vec(m.accel_bias);s.accel_scale=m.accel_scale;s.validate();return s;
}
class Bridge {
  ros::NodeHandle nh,pnh{"~"};ar::Frames frames;
  ar::Q q_il=ar::Q::Identity();ar::V3 t_il=ar::V3::Zero();
  ros::Publisher body_pub,world_pub,cloud_pub,health_pub,reason_pub;
  ros::Subscriber posterior_sub,high_sub,cloud_sub,valid_sub;ros::WallTimer timer;
  tf2_ros::TransformBroadcaster dynamic_tf;tf2_ros::StaticTransformBroadcaster static_tf;
  std::map<uint64_t,airseafly_real400_msgs::LioState> posteriors;
  std::map<uint64_t,sensor_msgs::PointCloud2ConstPtr> pending;
  std::string session,fault;ar::State last_posterior;
  double init_since=0,last_high=0,last_clock=0,last_scan=0,last_receipt=0,stable_seconds=2;
  bool prediction_valid=false;
  double radius=.4,body_z_min=-.5,body_z_max=.3,z_min=-.2,z_max=2.7,voxel=.05,range=5.;
  void status(bool valid,const std::string& why) {
    std_msgs::Bool b;b.data=valid;health_pub.publish(b);
    std_msgs::String m;m.data=why;reason_pub.publish(m);
  }
  geometry_msgs::TransformStamped tf(const std::string& parent,const std::string& child,const ar::V3& p,const ar::Q& q,ros::Time stamp) {
    geometry_msgs::TransformStamped t;t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child;
    vecout(t.transform.translation,p);quatout(t.transform.rotation,q);return t;
  }
  void posterior(const airseafly_real400_msgs::LioStateConstPtr& m) {
    try {
      ar::require(fault.empty(),fault.c_str());
      ar::require(m->header.frame_id=="lio_init" && m->child_frame_id=="livox_imu","wrong posterior frame");
      if(session.empty()) session=m->session_id;
      ar::require(!session.empty() && session==m->session_id,"LIO restarted: restart whole session");
      auto s=state(*m);ar::require(s.t>last_posterior.t,"posterior clock reset");
      last_posterior=s;posteriors[m->header.stamp.toNSec()]=*m;
      while(posteriors.size()>30) posteriors.erase(posteriors.begin());
      if(!frames.initialized) {
        if(s.v.norm()>.1 || vec(m->angular_velocity_body).norm()>.1 || !ar::fresh(ros::Time::now().toSec(),s.t,.25)) {init_since=0;return;}
        if(!init_since) init_since=s.t;
        if(s.t-init_since<stable_seconds) return;
        frames.initialize(s);
        static_tf.sendTransform(std::vector<geometry_msgs::TransformStamped>{
          tf("odom","lio_init",frames.t_ow,frames.q_ow,m->header.stamp),
          tf("base_link","livox_imu",frames.t_bi,frames.q_bi,m->header.stamp),
          tf("livox_imu","livox_frame",t_il,q_il,m->header.stamp)});
        ROS_INFO("Frozen odom anchor: session %s",session.c_str());
      }
      flushClouds();
    } catch(const std::exception& e) {fault=e.what();status(false,fault);}
  }
  void high(const nav_msgs::OdometryConstPtr& m) {
    try {
      ar::require(fault.empty(),fault.c_str());
      ar::require(m->header.frame_id=="lio_init" && m->child_frame_id=="livox_imu","wrong high odometry frame");
      const double t=m->header.stamp.toSec();
      ar::require(t>last_high,"high odometry clock reset/duplicate");
      last_high=t;last_receipt=ros::WallTime::now().toSec();
      if(!frames.initialized || !prediction_valid || !ar::fresh(ros::Time::now().toSec(),t)) {status(false,"waiting for fresh initialized prediction");return;}
      ar::State s=last_posterior;s.t=t;s.p=vec(m->pose.pose.position);s.q=quat(m->pose.pose.orientation);s.v=vec(m->twist.twist.linear);
      auto b=frames.transform(s,vec(m->twist.twist.angular));
      nav_msgs::Odometry out;out.header=m->header;out.header.frame_id="odom";out.child_frame_id="base_link";
      vecout(out.pose.pose.position,b.p);quatout(out.pose.pose.orientation,b.q);
      vecout(out.twist.twist.angular,b.omega_body);vecout(out.twist.twist.linear,b.v_body);
      // Conservative diagonals bound rotated posterior covariance and lever arm.
      // No external-EKF injection is performed by this workspace.
      double pvar=0,rvar=0;for(int i=0;i<3;++i) {pvar+=std::max(0.,m->pose.covariance[i*6+i]);rvar+=std::max(0.,m->pose.covariance[(i+3)*6+i+3]);}
      for(int i=0;i<3;++i) {out.pose.covariance[i*6+i]=pvar+frames.t_bi.squaredNorm()*rvar;
        out.pose.covariance[(i+3)*6+i+3]=rvar;out.twist.covariance[i*6+i]=.12;out.twist.covariance[(i+3)*6+i+3]=.12;}
      body_pub.publish(out);
      vecout(out.twist.twist.linear,b.v_world);vecout(out.twist.twist.angular,b.omega_world);world_pub.publish(out);
      dynamic_tf.sendTransform(tf("odom","base_link",b.p,b.q,m->header.stamp));
      status(true,"valid frozen-frame body/world odometry");
    } catch(const std::exception& e) {fault=e.what();status(false,fault);}
  }
  void cloud(const sensor_msgs::PointCloud2ConstPtr& m) {
    if(m->header.frame_id!="livox_imu") {status(false,"cloud must be Livox IMU-frame deskewed scan");return;}
    pending[m->header.stamp.toNSec()]=m;
    while(pending.size()>10) pending.erase(pending.begin());flushClouds();
  }
  void flushClouds() {
    if(!frames.initialized || !fault.empty()) return;
    for(auto it=pending.begin();it!=pending.end();) {
      auto found=posteriors.find(it->first);if(found==posteriors.end()) {++it;continue;}
      const auto cloud=it->second;it=pending.erase(it);
      if(!ar::fresh(ros::Time::now().toSec(),cloud->header.stamp.toSec(),.35)) continue;
      const auto s=state(found->second); // EXACT scan-tail posterior, never latest prediction.
      pcl::PointCloud<pcl::PointXYZI> raw,filtered; pcl::fromROSMsg(*cloud,raw);
      for(const auto& p:raw) {
        ar::V3 pi(p.x,p.y,p.z);if(!pi.allFinite() || pi.norm()>range) continue;
        ar::V3 pb=frames.q_bi*pi+frames.t_bi;
        if(pb.head<2>().norm()<radius && pb.z()>body_z_min && pb.z()<body_z_max) continue;
        ar::V3 po=frames.scanPoint(s,pi);if(po.z()<z_min || po.z()>z_max) continue;
        pcl::PointXYZI o; o.x=po.x();o.y=po.y();o.z=po.z();o.intensity=p.intensity;filtered.push_back(o);
      }
      pcl::VoxelGrid<pcl::PointXYZI> vg;vg.setLeafSize(voxel,voxel,voxel);vg.setInputCloud(filtered.makeShared());
      pcl::PointCloud<pcl::PointXYZI> reduced;vg.filter(reduced);
      sensor_msgs::PointCloud2 out;pcl::toROSMsg(reduced,out);out.header=cloud->header;out.header.frame_id="odom";
      cloud_pub.publish(out);last_scan=cloud->header.stamp.toSec();
    }
  }
  void watchdog(const ros::WallTimerEvent&) {
    double now=ros::Time::now().toSec();
    if(last_clock && now<last_clock) fault="ROS clock reversed: restart session";last_clock=now;
    if(!fault.empty()) status(false,fault);
    else if(!ar::fresh(now,last_high) || ros::WallTime::now().toSec()-last_receipt>.08) status(false,"odometry stalled/clock paused");
  }
 public:
  Bridge() {
    std::vector<double> q,t;
    ar::require(pnh.getParam("mount/quaternion_xyzw",q)&&q.size()==4,"mount quaternion missing");
    ar::require(pnh.getParam("mount/translation_xyz_m",t)&&t.size()==3,"mount translation missing");
    frames.q_bi=ar::normalized({q[3],q[0],q[1],q[2]});frames.t_bi={t[0],t[1],t[2]};
    ar::require(pnh.getParam("lidar_in_imu/quaternion_xyzw",q)&&q.size()==4,"LiDAR factory quaternion missing");
    ar::require(pnh.getParam("lidar_in_imu/translation_xyz_m",t)&&t.size()==3,"LiDAR factory translation missing");
    q_il=ar::normalized({q[3],q[0],q[1],q[2]});t_il={t[0],t[1],t[2]};
    pnh.param("initialization_seconds",stable_seconds,2.);pnh.param("self_filter/radius",radius,.4);
    pnh.param("self_filter/z_min",body_z_min,-.5);pnh.param("self_filter/z_max",body_z_max,.3);
    pnh.param("cloud/z_min",z_min,-.2);pnh.param("cloud/z_max",z_max,2.7);
    pnh.param("cloud/range",range,5.);pnh.param("cloud/voxel",voxel,.05);
    body_pub=nh.advertise<nav_msgs::Odometry>("/airseafly/localization/odom_body",20);
    world_pub=nh.advertise<nav_msgs::Odometry>("/airseafly/localization/odom",20);
    cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/airseafly/lidar/obstacles_odom",2);
    health_pub=nh.advertise<std_msgs::Bool>("/airseafly/localization/healthy",1,true);
    reason_pub=nh.advertise<std_msgs::String>("/airseafly/localization/reason",1,true);
    posterior_sub=nh.subscribe("/airseafly/lio/posterior",30,&Bridge::posterior,this);
    high_sub=nh.subscribe("/airseafly/lio/high_odom",50,&Bridge::high,this,ros::TransportHints().tcpNoDelay());
    cloud_sub=nh.subscribe("/airseafly/lio/cloud_body",3,&Bridge::cloud,this);
    valid_sub=nh.subscribe<std_msgs::Bool>("/airseafly/lio/prediction_valid",1,[this](const std_msgs::BoolConstPtr& m){prediction_valid=m->data;});
    timer=nh.createWallTimer(ros::WallDuration(.02),&Bridge::watchdog,this);status(false,"initializing");
  }
};
int main(int argc,char** argv) {ros::init(argc,argv,"real400_frames");try {Bridge b;ros::spin();} catch(const std::exception& e){ROS_FATAL("%s",e.what());return 2;}return 0;}
