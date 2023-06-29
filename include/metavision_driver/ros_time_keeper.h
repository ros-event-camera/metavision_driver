// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef METAVISION_DRIVER__ROS_TIME_KEEPER_H_
#define METAVISION_DRIVER__ROS_TIME_KEEPER_H_

#include <algorithm>  // min/max
#include <chrono>
#include <memory>
#include <string>

#include "metavision_driver/logging.h"

namespace metavision_driver
{
class ROSTimeKeeper
{
  template <class T>
  inline static T clamp(const T & a, const T & low, const T & high)
  {
    return std::max(low, std::min(a, high));
  }
  static inline int64_t clamp_buffering_delay(uint64_t rosT, uint64_t trialTime)
  {
    const int64_t MAX_BUFFER_DELAY = 2000000L;  // 2 ms
    return (
      clamp(static_cast<int64_t>(rosT) - static_cast<int64_t>(trialTime), 0L, MAX_BUFFER_DELAY));
  }

public:
  explicit ROSTimeKeeper(const std::string & name) : loggerName_(name) {}
  inline void setLastROSTime(uint64_t t) { lastROSTime_ = t; }
  inline uint64_t updateROSTimeOffset(double dt_sensor, uint64_t rosT)
  {
    if (rosT0_ == 0) {  // first time here
      resetROSTime(rosT, dt_sensor);
    }
    if (dt_sensor < prevSensorTime_) {
      LOG_ERROR_NAMED("BAD! Sensor time going backwards, replug camera!");
    }
    double sensor_inc = std::max(dt_sensor - prevSensorTime_, 0.0);
    const int64_t sensor_inc_int = static_cast<int64_t>(sensor_inc);
    prevSensorTime_ = dt_sensor;
    const int64_t ros_inc = static_cast<int64_t>(rosT) - static_cast<int64_t>(prevROSTime_);
    prevROSTime_ = rosT;
    // emergency reset if sensor time changed much more than ROS time (wall clock)
    // - sensor time changed more than 20% with respect to ros time
    const int64_t abs_change_diff = std::abs(sensor_inc_int - ros_inc);
    if (abs_change_diff * 20 > std::abs(ros_inc) && abs_change_diff > 20000000L) {
      LOG_WARN_NAMED(
        "sensor time jumped by " << sensor_inc * 1e-9 << "s vs wall clock: " << ros_inc * 1e-9
                                 << ", resetting ROS stamp time!");
      sensor_inc = 0;
      resetROSTime(rosT, dt_sensor);
    }
    // compute time in seconds elapsed since ROS startup
    const double dt_ros = static_cast<double>(rosT - rosT0_);
    // difference between elapsed ROS time and elapsed sensor Time
    const double dt = dt_ros - dt_sensor;
    // compute moving average of elapsed time difference.
    // the averaging constant alpha is picked such that the average is
    // taken over about 10 sec:
    // alpha = elapsed_time / 10sec
    constexpr double f = 1.0 / (10.0e9);
    const double alpha = clamp(sensor_inc * f, 0.0001, 0.1);
    averageTimeDifference_ = averageTimeDifference_ * (1.0 - alpha) + alpha * dt;
    //
    // We want to use sensor time, but adjust it for the average clock
    // skew between sensor time and ros time, plus some unknown buffering delay dt_buf
    // (to be estimated)
    //
    // t_ros_adj
    //  = t_sensor + avg(t_ros - t_sensor) + dt_buf
    //  = t_sensor_0 + dt_sensor + avg(t_ros_0 + dt_ros - (t_sensor_0 + dt_sensor)) + dt_buf
    //          [now use t_sensor_0 and t_ros_0 == constant]
    //  = t_ros_0 + avg(dt_ros - dt_sensor) + dt_sensor + dt_buf
    //  =: ros_time_offset + dt_sensor;
    //
    // Meaning once ros_time_offset has been computed, the adjusted ros timestamp
    // is obtained by just adding the sensor elapsed time (dt_sensor) that is reported
    // by the SDK.

    const uint64_t dt_sensor_int = static_cast<uint64_t>(dt_sensor);
    const int64_t avg_timediff_int = static_cast<int64_t>(averageTimeDifference_);
    const uint64_t MIN_EVENT_DELTA_T = 0LL;  // minimum time gap between packets

    // First test if the new ros time stamp (trialTime) would be in future. If yes, then
    // the buffering delay has been underestimated and must be adjusted.

    const uint64_t trialTime = rosT0_ + avg_timediff_int + dt_sensor_int;

    if (rosT < trialTime + bufferingDelay_) {  // time stamp would be in the future
      // compute buffering delay as clamped rosT - trialTime
      bufferingDelay_ = clamp_buffering_delay(rosT, trialTime);
    }

    // The buffering delay could make the time stamps go backwards.
    // Ensure that this does not happen. This safeguard may cause
    // time stamps to be (temporarily) in the future, there is no way around
    // that.
    if (trialTime + bufferingDelay_ < lastROSTime_ + MIN_EVENT_DELTA_T) {
      // compute buffering delay as clamped lastROSTime_ + MIN_EVENT_DELTA_T - trialTime
      bufferingDelay_ = clamp_buffering_delay(lastROSTime_ + MIN_EVENT_DELTA_T, trialTime);
    }

    // warn if the ros header stamp is off by more than 10ms vs current stamp
    const int64_t ros_tdiff =
      static_cast<int64_t>(rosT) - static_cast<int64_t>(trialTime) - bufferingDelay_;
    if (std::abs(ros_tdiff) > 10000000LL) {
#ifdef USING_ROS_1
      LOG_WARN_NAMED_FMT_THROTTLE(5000, "ROS timestamp off by: %.2fms", ros_tdiff * 1e-6);
#else
      rclcpp::Clock clock;
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(loggerName_), clock, 5000, "ROS timestamp off by: %.2fms",
        ros_tdiff * 1e-6);
#endif
    }

    const uint64_t rosTimeOffset = rosT0_ + avg_timediff_int + bufferingDelay_;

    return (rosTimeOffset);
  }

private:
  inline void resetROSTime(const uint64_t rosT, const double dt_sensor)
  {
    rosT0_ = rosT;
    // initialize to dt_ros - dt_sensor because dt_ros == 0
    averageTimeDifference_ = -dt_sensor;
    lastROSTime_ = rosT;
    prevROSTime_ = rosT;
    bufferingDelay_ = 0;
    prevSensorTime_ = dt_sensor;
  }

  // ---------  variables
  std::string loggerName_;
  uint64_t rosT0_{0};                // time when first callback happened
  double averageTimeDifference_{0};  // average of elapsed_ros_time - elapsed_sensor_time
  double prevSensorTime_{0};         // sensor time during previous update
  int64_t bufferingDelay_{0};        // estimate of buffering delay
  uint64_t lastROSTime_{0};          // the last event's ROS time stamp
  uint64_t prevROSTime_{0};          // last time the ROS offset was updated
};

}  // namespace metavision_driver
#endif  // METAVISION_DRIVER__ROS_TIME_KEEPER_H_
