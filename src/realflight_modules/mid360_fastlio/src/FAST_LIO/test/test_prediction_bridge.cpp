// Exercises the real ROS adapter with doubles, without any vehicle connection.
#include "ros_stub.hpp"
#include "predictor_bridge.hpp"
#include <cstdlib>
#include <iostream>

int checks = 0;
void check(bool ok, const char *message) {
    ++checks;
    if (!ok) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
void configure(const std::string &mode, const std::string &horizon = "") {
    g_odom.clear(); g_text.clear();
    g_params = {{"prediction/mode", mode}};
    if (!horizon.empty()) g_params["prediction/max_prediction_horizon"] = horizon;
    ros::NodeHandle nh; configurePrediction(nh);
    state_point = StatePoint(); state_point.grav[2] = -9.81;
    time_diff_lidar_to_imu = timediff_lidar_wrt_imu = 0;
    time_sync_en = false;
}
void imu(double t) {
    auto msg = std::make_shared<sensor_msgs::Imu>();
    msg->header.stamp = ros::Time().fromSec(t);
    msg->linear_acceleration.z = 1.;
    feedPredictionImu(msg);
}
void anchor(double t) { lidar_end_time = t; updatePredictionAnchor(); }

int main() {
    for (const std::string mode : {"shadow", "replace"}) {
        configure(mode);
        const std::string topic = mode == "shadow" ? "/Odom_high_freq_repropagated" : "/Odom_high_freq";
        imu(10.); imu(10.005); anchor(10.);
        for (int k=2; k<=240; ++k) {
            const auto published_before = prediction_published;
            imu(10. + k * .005);
            if (k == 61 || k == 200)
                check(prediction_published == published_before + 1,
                      "publishes after 300 ms and exactly at one second");
            if (k == 201)
                check(prediction_published == published_before && prediction_last_withheld == "correction_expired",
                      "first sample beyond one second is withheld");
        }
        check(!g_odom[topic].empty(), "candidate produces data");
        check(std::abs(g_odom[topic].back().header.stamp.toSec()-11.) < 1e-8,
              "restored default publishes beyond 300 ms up to one second");
        check(prediction_last_withheld == "correction_expired", "reports stale correction");
        check(prediction_anchors == 1 && prediction_last_correction == 10., "accepted anchor tracked");
        check(prediction_latest_imu > 11. && prediction_last_output < 11.001, "input and output times distinct");
        check(legacy_odom_topic == (mode == "replace" ? "/Odom_high_freq_legacy" : "/Odom_high_freq"),
              "no automatic legacy fallback after expiry");
        check(g_text["/prediction/status"].back().find("\"last_correction_stamp_s\":10") != std::string::npos,
              "correction timestamp in status");
        check(g_text["/prediction/status"].back().find("prediction_horizon_s") != std::string::npos,
              "horizon in status");
        check(g_text["/prediction/status"].back().find("\"max_prediction_horizon_s\":1}") != std::string::npos,
              "status reports restored one-second default");
        auto before = g_odom[topic].size();
        imu(11.2); imu(11.1);
        check(g_odom[topic].size() == before, "duplicate/reversed samples do not freshen stale anchor");
        anchor(9.9);
        check(prediction_last_correction == 10. && prediction_anchors == 1, "older anchor not counted");
        anchor(11.19); imu(11.205);
        check(g_odom[topic].back().header.stamp.toSec() > 11.2, "fresh correction resumes same candidate stream");
        for (size_t i=1; i<g_odom[topic].size(); ++i)
            check(g_odom[topic][i].header.stamp.toSec() > g_odom[topic][i-1].header.stamp.toSec(),
                  "strictly advancing publication times");
    }
    configure("shadow"); imu(20.); imu(20.005); anchor(20.);
    auto before = prediction_published;
    imu(20.065);
    check(prediction_published == before && prediction_last_withheld == "imu_gap", "60 ms IMU gap withheld");
    check(std::abs(prediction_core.lastGapFrom()-20.005)<1e-10 &&
          std::abs(prediction_core.lastGapTo()-20.065)<1e-10, "exact missing input interval retained");
    check(prediction_core.gapCount()==1, "gap interval counted separately from withheld callbacks");
    imu(20.070); anchor(20.065); imu(20.075);
    check(prediction_published > before, "new anchor beyond gap recovers");
    check(prediction_core.gapCount()==1 && prediction_current_status=="ready",
          "rebuild does not recount gap; recovered status differs from historical withheld reason");
    lio_prediction::InputTiming timing;
    timing.observe(100., 1, 1., 100.01);
    timing.observe(100.005, 2, 1.3, 100.31);
    check(timing.max_source_dt<.006 && timing.max_arrival_dt>.29,
          "callback delay distinguished from source timestamp gap");
    timing.observe(100.2, 5, 1.301, 100.311);
    check(timing.max_source_dt>.19 && timing.sequence_skips==2,
          "source timestamp and ROS message sequence gaps counted independently");
    check(predictionInputJson(timing).find("\"sequence_skips\":2")!=std::string::npos,
          "input timing retained in JSON diagnostics");
    time_diff_lidar_to_imu = .1;
    before = prediction_published;
    imu(20.180);
    check(prediction_published == before && !std::isfinite(prediction_last_correction), "offset reset invalidates correction");
    check(predictionJsonNumber(std::numeric_limits<double>::quiet_NaN()) == "null", "JSON never emits NaN");
    configure("off"); imu(30.); anchor(30.);
    check(g_odom.empty() && g_text.empty(), "off never publishes candidate or status");
    check(legacy_odom_topic == "/Odom_high_freq", "off leaves original topic");
    for (const std::string value : {"0.2", "0.3", "0.5", "1.0", "1.5"}) {
        configure("replace", value);
        imu(40.); imu(40.005); anchor(40.);
        for (int k=2; k<=340; ++k) imu(40. + k * .005);
        check(std::abs(prediction_last_output-(40. + std::stod(value))) < 1e-8,
              "explicit shorter and longer horizons are honored without trial ceiling");
        check(prediction_last_withheld == "correction_expired", "configured horizon still expires");
        check(legacy_odom_topic == "/Odom_high_freq_legacy", "configured expiry does not switch source");
    }
    for (const std::string value : {"0", "-1", "nan", "inf"}) {
        configure("shadow"); g_params["prediction/max_prediction_horizon"] = value;
        ros::NodeHandle nh; bool threw = false;
        try { configurePrediction(nh); } catch (const std::invalid_argument &) { threw = true; }
        check(threw, "nonpositive or nonfinite horizon rejected");
    }
    configure("replace");
    kf.covariance.setZero();
    for (int i=0;i<23;++i) kf.covariance(i,i)=.01*(i+1);
    kf.covariance(0,3)=kf.covariance(3,0)=.004;
    state_point.rot=Eigen::Quaterniond(Eigen::AngleAxisd(1.5707963267948966,Eigen::Vector3d::UnitZ()));
    imu(50.); anchor(50.);
    auto msg=g_odom["/Odom_high_freq"].back();
    check(msg.header.stamp.toSec()==50.,"unpropagated covariance stamp equals posterior time");
    check(std::abs(msg.pose.covariance[0]-.01)<1e-12,"position covariance uses IKFoM pos index zero");
    check(std::abs(msg.pose.covariance[28]-.04)<1e-12,"right attitude covariance rotates into world frame");
    check(std::abs(msg.pose.covariance[4]-.004)<1e-12,"pose cross covariance preserved and rotated");
    imu(50.005);
    msg=g_odom["/Odom_high_freq"].back();
    check(msg.header.stamp.toSec()==50.005 && msg.pose.covariance[0]>.01,
          "covariance and state advance to same new IMU epoch");
    const auto count=prediction_published;
    kf.covariance(0,0)=.1; anchor(50.0025);
    check(prediction_published==count,"posterior replay does not publish old or duplicate stamp");
    imu(50.01);
    check(g_odom["/Odom_high_freq"].back().pose.covariance[0]>.1,
          "new posterior covariance replaces previous estimate before next publication");
    auto invalid=std::make_shared<sensor_msgs::Imu>();
    invalid->header.stamp=ros::Time().fromSec(50.02);
    invalid->linear_acceleration.z=std::numeric_limits<double>::quiet_NaN();
    check(!acceptPredictionInput(invalid,50.01) && prediction_last_withheld=="invalid_imu",
          "invalid accelerometer rejected before either main filter or predictor");
    invalid->linear_acceleration.z=1;
    invalid->angular_velocity.x=std::numeric_limits<double>::infinity();
    check(!acceptPredictionInput(invalid,50.01),"invalid gyroscope rejected before main filter");
    invalid->angular_velocity.x=0; invalid->header.stamp=ros::Time().fromSec(50.005);
    check(!acceptPredictionInput(invalid,50.01) && prediction_last_withheld=="nonadvancing_imu",
          "reversed input does not reach main filter queue");
    time_diff_lidar_to_imu=.02;
    check(!acceptPredictionInput(invalid,50.01) && prediction_epoch_fault,
          "decreased corrected epoch faults before nonadvancing guard");
    invalid->header.stamp=ros::Time().fromSec(51.);
    check(!acceptPredictionInput(invalid,50.01),"clock mapping change requires explicit node restart");
    const auto fault_count=prediction_published; anchor(50.5); imu(51.);
    check(prediction_published==fault_count,"clock mapping fault cannot republish or auto-recover");
    configure("replace"); time_diff_lidar_to_imu=.027;
    imu(60.027); anchor(60.);
    check(std::abs(g_odom["/Odom_high_freq"].back().header.stamp.toSec()-60.)<1e-12,
          "configured time offset applied before predictor propagation and output stamp");
    std::cout << "PASS: " << checks << " bridge contract checks; ROS doubles only\n";
}
