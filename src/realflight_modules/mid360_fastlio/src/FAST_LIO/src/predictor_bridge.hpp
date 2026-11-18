#pragma once
// IMU re-propagation after every LiDAR correction, wired into laserMapping.cpp.
// Included after legacy_imu_cbk; runs in the same single-thread spinOnce loop.
//
// prediction/mode:
//   off     - unchanged: legacy fastPredictIMU publishes /Odom_high_freq
//   shadow  - legacy keeps /Odom_high_freq; re-propagated output on
//             /Odom_high_freq_repropagated for comparison only
//   replace - re-propagated output on /Odom_high_freq; legacy moves to
//             /Odom_high_freq_legacy for comparison
// Both outputs use the legacy message semantics: frame "world", child
// "odom_imu", world-frame linear velocity, bias-corrected body angular rate.
// Pose covariance: [p_world, small-angle_world] at EXACTLY header.stamp.
// Twist covariance remains unknown/zero; the bridge's source_pose experiment
// intentionally retains its independent fixed twist covariance.
#include "imu_predictor.hpp"
#include "input_timing.hpp"
#include <std_msgs/String.h>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>
#include <chrono>

std::string prediction_mode = "off";
std::string legacy_odom_topic = "/Odom_high_freq";
lio_prediction::Predictor prediction_core;
ros::Publisher prediction_pub, prediction_status_pub;
double prediction_time_shift = std::numeric_limits<double>::quiet_NaN();
double prediction_last_status = 0;
double prediction_last_correction = std::numeric_limits<double>::quiet_NaN();
double prediction_latest_imu = std::numeric_limits<double>::quiet_NaN();
double prediction_last_output = std::numeric_limits<double>::quiet_NaN();
double prediction_horizon_limit = 1.0;
unsigned long prediction_published = 0, prediction_withheld = 0, prediction_anchors = 0;
std::string prediction_last_withheld = "";
std::string prediction_current_status = "queued";
lio_prediction::InputTiming prediction_imu_input, prediction_lidar_input;
bool prediction_epoch_fault = false;

// Call after parameters are read and BEFORE the legacy publisher is advertised.
void configurePrediction(ros::NodeHandle &nh)
{
    prediction_last_correction = prediction_latest_imu = prediction_last_output =
        prediction_time_shift = std::numeric_limits<double>::quiet_NaN();
    prediction_last_status = 0;
    prediction_published = prediction_withheld = prediction_anchors = 0;
    prediction_last_withheld.clear();
    prediction_current_status = "queued";
    prediction_imu_input = lio_prediction::InputTiming();
    prediction_lidar_input = lio_prediction::InputTiming();
    prediction_epoch_fault = false;
    nh.param<std::string>("prediction/mode", prediction_mode, "off");
    if (prediction_mode != "off" && prediction_mode != "shadow" && prediction_mode != "replace")
        throw std::invalid_argument("prediction/mode must be off, shadow or replace");
    legacy_odom_topic = prediction_mode == "replace" ? "/Odom_high_freq_legacy" : "/Odom_high_freq";
    if (prediction_mode == "off") return;
    lio_prediction::Options options;
    int max_samples = 4096;
    nh.param<double>("prediction/history_seconds", options.history_seconds, 3.0);
    nh.param<double>("prediction/max_imu_gap", options.max_imu_gap, 0.05);
    // Keep the existing standalone trial's one-second default. This is a finite
    // IMU-minus-correction horizon, not a guarantee of end-to-end fault latency.
    nh.param<double>("prediction/max_prediction_horizon", options.max_prediction_horizon, 1.0);
    if (!std::isfinite(options.max_prediction_horizon) ||
        options.max_prediction_horizon <= 0)
        throw std::invalid_argument("prediction/max_prediction_horizon must be finite and positive");
    prediction_horizon_limit = options.max_prediction_horizon;
    nh.param<int>("prediction/max_samples", max_samples, 4096);
    if (max_samples < 2) throw std::invalid_argument("prediction/max_samples must be >= 2");
    options.max_samples = static_cast<std::size_t>(max_samples);
    prediction_core = lio_prediction::Predictor(options);
    const std::string topic = prediction_mode == "replace" ? "/Odom_high_freq" : "/Odom_high_freq_repropagated";
    prediction_pub = nh.advertise<nav_msgs::Odometry>(topic, 1);
    prediction_status_pub = nh.advertise<std_msgs::String>("/prediction/status", 10);
    ROS_WARN("FAST-LIO prediction mode '%s': re-propagated output on %s, legacy on %s",
             prediction_mode.c_str(), topic.c_str(), legacy_odom_topic.c_str());
}

std::string predictionJsonNumber(double value)
{
    if (!std::isfinite(value)) return "null";
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

void observePredictionInput(lio_prediction::InputTiming &input, double stamp, std::uint32_t sequence)
{
    if (prediction_mode == "off") return;
    const double steady = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    input.observe(stamp, sequence, steady, ros::Time::now().toSec());
}

void observePredictionLidarInput(double stamp, std::uint32_t sequence)
{
    observePredictionInput(prediction_lidar_input, stamp, sequence);
}

std::string predictionInputJson(const lio_prediction::InputTiming &input)
{
    std::ostringstream out;
    out << "{\"samples\":" << input.samples << ",\"sequence_skips\":" << input.sequence_skips
        << ",\"nonadvancing_stamps\":" << input.nonadvancing_stamps
        << ",\"source_dt_s\":" << predictionJsonNumber(input.source_dt)
        << ",\"arrival_dt_s\":" << predictionJsonNumber(input.arrival_dt)
        << ",\"max_source_dt_s\":" << predictionJsonNumber(input.max_source_dt)
        << ",\"max_arrival_dt_s\":" << predictionJsonNumber(input.max_arrival_dt)
        << ",\"receipt_minus_stamp_s\":" << predictionJsonNumber(input.receipt_minus_stamp)
        << ",\"max_receipt_minus_stamp_s\":" << predictionJsonNumber(input.max_receipt_minus_stamp)
        << ",\"max_source_gap_from_s\":" << predictionJsonNumber(input.max_source_gap_from)
        << ",\"max_source_gap_to_s\":" << predictionJsonNumber(input.max_source_gap_to) << "}";
    return out.str();
}

void publishPredictionStatus(double stamp)
{
    if (!std::isfinite(stamp) || (stamp >= prediction_last_status &&
                                stamp - prediction_last_status < 0.1)) return;
    prediction_last_status = stamp;
    const double horizon = prediction_latest_imu - prediction_last_correction;
    std::ostringstream text;
    text << "{\"mode\":\"" << prediction_mode << "\",\"published\":" << prediction_published
         << ",\"withheld\":" << prediction_withheld << ",\"anchors\":" << prediction_anchors
         << ",\"last_withheld\":\"" << prediction_last_withheld << "\",\"buffered\":"
         << prediction_core.bufferedSamples()
         << ",\"last_correction_stamp_s\":" << predictionJsonNumber(prediction_last_correction)
         << ",\"latest_imu_stamp_s\":" << predictionJsonNumber(prediction_latest_imu)
         << ",\"last_output_stamp_s\":" << predictionJsonNumber(prediction_last_output)
         << ",\"prediction_horizon_s\":" << predictionJsonNumber(horizon)
         << ",\"current_status\":\"" << prediction_current_status << "\""
         << ",\"imu_gap_intervals\":" << prediction_core.gapCount()
         << ",\"last_imu_gap_from_s\":" << predictionJsonNumber(prediction_core.lastGapFrom())
         << ",\"last_imu_gap_to_s\":" << predictionJsonNumber(prediction_core.lastGapTo())
         << ",\"imu_input\":" << predictionInputJson(prediction_imu_input)
         << ",\"lidar_input\":" << predictionInputJson(prediction_lidar_input)
         << ",\"max_prediction_horizon_s\":" << predictionJsonNumber(prediction_horizon_limit)
         << "}";
    std_msgs::String msg;
    msg.data = text.str();
    prediction_status_pub.publish(msg);
}

void publishPrediction()
{
    lio_prediction::Prediction p;
    if (!prediction_core.takePublication(p)) return;
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time().fromSec(p.t);
    msg.header.frame_id = "world";
    msg.child_frame_id = "odom_imu";
    msg.pose.pose.position.x = p.position.x;
    msg.pose.pose.position.y = p.position.y;
    msg.pose.pose.position.z = p.position.z;
    msg.pose.pose.orientation.w = p.orientation.w;
    msg.pose.pose.orientation.x = p.orientation.x;
    msg.pose.pose.orientation.y = p.orientation.y;
    msg.pose.pose.orientation.z = p.orientation.z;
    msg.twist.twist.linear.x = p.velocity.x;
    msg.twist.twist.linear.y = p.velocity.y;
    msg.twist.twist.linear.z = p.velocity.z;
    msg.twist.twist.angular.x = p.angular_velocity_body.x;
    msg.twist.twist.angular.y = p.angular_velocity_body.y;
    msg.twist.twist.angular.z = p.angular_velocity_body.z;
    if (p.covariance_valid) {
        const auto covariance = p.poseCovariance();
        for (int i=0; i<6; ++i) for (int j=0; j<6; ++j)
            msg.pose.covariance[i*6+j] = covariance(i,j);
    }
    prediction_pub.publish(msg);
    prediction_last_output = p.t;
    ++prediction_published;
}

void notePredictionStatus(lio_prediction::Status status)
{
    using lio_prediction::Status;
    prediction_current_status = lio_prediction::statusName(status);
    if (status == Status::Ready) { publishPrediction(); return; }
    if (status == Status::Queued || status == Status::Duplicate) return;
    ++prediction_withheld;
    prediction_last_withheld = lio_prediction::statusName(status);
    ROS_WARN_THROTTLE(1.0, "re-propagated prediction withheld: %s", prediction_last_withheld.c_str());
}

// Validate before BOTH the main filter and predictor see a sample. Changing the
// clock mapping while an EKF is running cannot be repaired by clearing only the
// predictor's history. In trial modes stop until node restart instead.
bool acceptPredictionInput(const sensor_msgs::Imu::ConstPtr &msg_in, double last_main_stamp)
{
    if (prediction_mode == "off") return true;
    const double raw = msg_in->header.stamp.toSec();
    const double t = lio_prediction::adjustedImuTime(raw, time_diff_lidar_to_imu,
                                                     time_sync_en, timediff_lidar_wrt_imu);
    std::string reason;
    if (!std::isfinite(t) || !std::isfinite(msg_in->linear_acceleration.x) ||
        !std::isfinite(msg_in->linear_acceleration.y) || !std::isfinite(msg_in->linear_acceleration.z) ||
        !std::isfinite(msg_in->angular_velocity.x) || !std::isfinite(msg_in->angular_velocity.y) ||
        !std::isfinite(msg_in->angular_velocity.z)) reason = "invalid_imu";
    else if (prediction_epoch_fault || (std::isfinite(prediction_time_shift) &&
             std::abs((t-raw)-prediction_time_shift)>1e-6)) {
        prediction_epoch_fault = true;
        prediction_core.reset();
        prediction_last_correction = prediction_latest_imu = std::numeric_limits<double>::quiet_NaN();
        reason = "time_offset_changed_restart_required";
    }
    else if (std::isfinite(last_main_stamp) && t<=last_main_stamp) reason = "nonadvancing_imu";
    if (!reason.empty()) {
        ++prediction_withheld;
        prediction_last_withheld = reason;
        prediction_current_status = reason;
        ROS_WARN_THROTTLE(1.0,"prediction trial rejected IMU: %s",reason.c_str());
        publishPredictionStatus(t);
        return false;
    }
    prediction_time_shift = t - raw;
    return true;
}

// Same time base and unit conversion as legacy_imu_cbk feeds ImuProcess.
void feedPredictionImu(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    if (prediction_mode == "off" || !acceptPredictionInput(msg_in,
            std::numeric_limits<double>::quiet_NaN())) return;
    const double t = lio_prediction::adjustedImuTime(msg_in->header.stamp.toSec(), time_diff_lidar_to_imu,
                                                     time_sync_en, timediff_lidar_wrt_imu);
    lio_prediction::Sample sample;
    sample.t = t;
    sample.accel = lio_prediction::Vec3(msg_in->linear_acceleration.x * G_m_s2,
                                        msg_in->linear_acceleration.y * G_m_s2,
                                        msg_in->linear_acceleration.z * G_m_s2);
    sample.gyro = lio_prediction::Vec3(msg_in->angular_velocity.x, msg_in->angular_velocity.y,
                                       msg_in->angular_velocity.z);
    const auto status = prediction_core.addImu(sample);
    if (status != lio_prediction::Status::Invalid && status != lio_prediction::Status::TooOld &&
        (!std::isfinite(prediction_latest_imu) || t > prediction_latest_imu)) prediction_latest_imu = t;
    notePredictionStatus(status);
    publishPredictionStatus(t);
}

// Call right after the iterated update has produced the posterior at lidar_end_time.
void updatePredictionAnchor()
{
    if (prediction_mode == "off" || prediction_epoch_fault) return;
    double scale = 0;
    if (!p_imu->prediction_accel_scale(scale)) return;
    lio_prediction::State s;
    s.t = lidar_end_time;
    s.position = lio_prediction::Vec3(state_point.pos(0), state_point.pos(1), state_point.pos(2));
    s.velocity = lio_prediction::Vec3(state_point.vel(0), state_point.vel(1), state_point.vel(2));
    const Eigen::Quaterniond q(state_point.rot.toRotationMatrix());
    s.orientation = lio_prediction::Quaternion(q.w(), q.x(), q.y(), q.z());
    s.accel_bias = lio_prediction::Vec3(state_point.ba(0), state_point.ba(1), state_point.ba(2));
    s.gyro_bias = lio_prediction::Vec3(state_point.bg(0), state_point.bg(1), state_point.bg(2));
    s.gravity = lio_prediction::Vec3(state_point.grav[0], state_point.grav[1], state_point.grav[2]);
    s.accel_scale = scale;
    // IKFoM state_ikfom: pos 0, right rotation 3, extrinsics 6/9,
    // velocity 12, gyro bias 15, accelerometer bias 18, gravity S2 at 21.
    // Use the S2 tangent Jacobian; its two coordinates are not world x/y.
    Eigen::Matrix<double,18,23> extract = Eigen::Matrix<double,18,23>::Zero();
    const int source_offsets[5] = {0,3,12,15,18};
    for (int block=0; block<5; ++block)
        extract.block<3,3>(3*block,source_offsets[block]) = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double,3,2> gravity_jacobian;
    const Eigen::Matrix<double,2,1> zero = Eigen::Matrix<double,2,1>::Zero();
    state_point.S2_Mx(gravity_jacobian,zero,21);
    extract.block<3,2>(15,21) = gravity_jacobian;
    const Eigen::Matrix<double,18,18> posterior = extract*kf.get_P()*extract.transpose();
    for (int i=0; i<18; ++i) for (int j=0; j<18; ++j)
        s.covariance(i,j) = .5*(posterior(i,j)+posterior(j,i));
    for (int i=0; i<12; ++i) for (int j=0; j<12; ++j)
        s.process_noise(i,j) = p_imu->Q(i,j);
    s.covariance_valid = true;
    const auto status = prediction_core.correct(s);
    if (status != lio_prediction::Status::Invalid && status != lio_prediction::Status::OldCorrection) {
        ++prediction_anchors;
        prediction_last_correction = s.t;
    }
    notePredictionStatus(status);
}
