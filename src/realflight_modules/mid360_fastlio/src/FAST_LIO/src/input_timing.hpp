#pragma once
// Passive input timing only. No filtering, deadlines, or readiness decisions.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace lio_prediction {
struct InputTiming {
  std::uint64_t samples = 0, sequence_skips = 0, nonadvancing_stamps = 0;
  std::uint32_t last_sequence = 0;
  double last_stamp = std::numeric_limits<double>::quiet_NaN();
  double last_arrival = std::numeric_limits<double>::quiet_NaN();
  double source_dt = 0, arrival_dt = 0, max_source_dt = 0, max_arrival_dt = 0;
  double receipt_minus_stamp = 0, max_receipt_minus_stamp = 0;
  double max_source_gap_from = 0, max_source_gap_to = 0;

  void observe(double stamp, std::uint32_t sequence, double steady_now, double ros_now) {
    if (samples) {
      source_dt = stamp - last_stamp;
      arrival_dt = steady_now - last_arrival;
      if (source_dt <= 0) ++nonadvancing_stamps;
      if (source_dt > max_source_dt) {
        max_source_dt = source_dt;
        max_source_gap_from = last_stamp;
        max_source_gap_to = stamp;
      }
      max_arrival_dt = std::max(max_arrival_dt, arrival_dt);
      const std::uint32_t step = sequence - last_sequence;
      // Ignore reset/backward/wrap ambiguity; count only forward ROS sequence gaps.
      if (step > 1 && step < 0x80000000u) sequence_skips += step - 1;
    }
    receipt_minus_stamp = ros_now - stamp; // Includes any source clock offset.
    max_receipt_minus_stamp = std::max(max_receipt_minus_stamp, receipt_minus_stamp);
    last_stamp = stamp; last_arrival = steady_now; last_sequence = sequence;
    ++samples;
  }
};
}  // namespace lio_prediction
