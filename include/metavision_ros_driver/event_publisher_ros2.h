// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef METAVISION_ROS_DRIVER__EVENT_PUBLISHER_ROS2_H_
#define METAVISION_ROS_DRIVER__EVENT_PUBLISHER_ROS2_H_

#include <event_array_msgs/encode.h>

#include <chrono>
#include <event_array_msgs/msg/event_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "metavision_ros_driver/callback_handler.h"
#include "metavision_ros_driver/metavision_wrapper.h"

// #define DEBUG_PERFORMANCE

// skip first few packets that may have bad time stamps
#define SKIP_TIME 2e9
namespace metavision_ros_driver
{
template <class MsgType>
class EventPublisherROS2 : public CallbackHandler
{
public:
  EventPublisherROS2(
    rclcpp::Node * node, const std::shared_ptr<MetavisionWrapper> & wrapper,
    const std::string & frameId)
  : node_(node),
    wrapper_(wrapper),
    messageTimeThreshold_(0),
    frameId_(frameId),
    readyIntervalTime_(rclcpp::Duration::from_seconds(1.0))
  {
    const double mtt = node->declare_parameter<double>("message_time_threshold", 100e-6);
    messageTimeThreshold_ = static_cast<uint64_t>(1e9 * mtt);
    RCLCPP_INFO_STREAM(node->get_logger(), "message time threshold: " << mtt << "s");
    reserveSize_ =
      (size_t)(node->declare_parameter<double>("sensors_max_mevs", 50.0) / std::max(mtt, 1e-6));
    RCLCPP_INFO_STREAM(node->get_logger(), "using reserve size: " << reserveSize_);
    auto qosProf = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
#ifdef DEBUG_PERFORMANCE
    startTime_ = std::chrono::high_resolution_clock::now();
#endif
    pub_ = node->create_publisher<MsgType>("~/events", qosProf);
    bool hasSync = node->get_parameter("sync_mode", syncMode_);
    if (hasSync && syncMode_ == "secondary") {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
      secondaryReadyPub_ = node->create_publisher<std_msgs::msg::Header>("~/ready", qos);
    }

    width_ = wrapper_->getWidth();
    height_ = wrapper_->getHeight();
    const union {
      uint32_t i;
      char c[4];
    } combined_int = {0x01020304};  // from stackoverflow
    isBigEndian_ = (combined_int.c[0] == 1);
    lastReadyTime_ = node->now() - readyIntervalTime_;  // move to past
  }

  ~EventPublisherROS2() {}

  void publish(const Metavision::EventCD * start, const Metavision::EventCD * end) override
  {
    const double sensorElapsedTime = start->t * 1e3;  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      return;
    }
    const uint64_t sensorElapsedTimeInt = static_cast<uint64_t>(sensorElapsedTime);
    const size_t n = end - start;
    int eventCount[2] = {0, 0};
    if (pub_->get_subscription_count() > 0 && sensorElapsedTime > SKIP_TIME) {
      if (!msg_) {  // must allocate new message
        rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
        msg_.reset(new MsgType());
        msg_->header.frame_id = frameId_;
        msg_->width = width_;
        msg_->height = height_;
        msg_->header.stamp = rclcpp::Time(rosTimeOffset_ + sensorElapsedTimeInt, RCL_SYSTEM_TIME);
        msg_->events.reserve(reserveSize_ * 2);
      }
      auto & events = msg_->events;
      const size_t old_size = events.size();
      // The resize should not trigger a
      // copy with proper reserved capacity.
      events.resize(events.size() + n);
      // copy data into ROS message. For the SilkyEvCam
      // the full load packet size delivered by the SDK is 320
      for (unsigned int i = 0; i < n; i++) {
        const auto & e_src = start[i];
        auto & e_trg = events[i + old_size];
        e_trg.x = e_src.x;
        e_trg.y = e_src.y;
        e_trg.polarity = e_src.p;
        e_trg.ts = rclcpp::Time(rosTimeOffset_ + (uint64_t)(e_src.t * 1e3), RCL_SYSTEM_TIME);
        eventCount[e_src.p]++;
      }
      lastROSTime_ = rosTimeOffset_ + static_cast<uint64_t>(start[n - 1].t * 1e3);
      if (lastROSTime_ > rclcpp::Time(msg_->header.stamp).nanoseconds() + messageTimeThreshold_) {
        wrapper_->updateEventsSent(events.size());
        wrapper_->updateMsgsSent(1);
        pub_->publish(std::move(msg_));
      }
    } else {
      // no subscribers, just gather event statistics
      for (unsigned int i = 0; i < n; i++) {
        eventCount[start[i].p]++;
      }
    }
    wrapper_->updateEventCount(0, eventCount[0]);
    wrapper_->updateEventCount(1, eventCount[1]);
  }

  bool keepRunning() override { return (rclcpp::ok()); }

private:
  inline uint64_t updateROSTimeOffset(double dt_sensor)
  {
    const uint64_t rosT = node_->now().nanoseconds();
    if (rosT0_ == 0) {
      rosT0_ = rosT;
      // initialize to dt_ros - dt_sensor because dt_ros == 0
      averageTimeDifference_ = -dt_sensor;
      lastROSTime_ = rosT;
      bufferingDelay_ = 0;
      prevSensorTime_ = dt_sensor;
    }
    // compute time in seconds elapsed since ROS startup
    const double dt_ros = static_cast<double>(rosT - rosT0_);
    // difference between elapsed ROS time and elapsed sensor Time
    const double dt = dt_ros - dt_sensor;
    // compute moving average of elapsed time difference
    // average over 10 seconds
    constexpr double f = 1.0 / (10e9);
    const double sensor_inc = dt_sensor - prevSensorTime_;
    const double alpha = std::min(sensor_inc * f, 0.1);
    averageTimeDifference_ = averageTimeDifference_ * (1.0 - alpha) + alpha * dt;
    prevSensorTime_ = dt_sensor;
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
      bufferingDelay_ = -(trialTime - rosT);
    }

    // The buffering delay could make the time stamps go backwards.
    // Ensure that this does not happen. This safeguard may cause
    // time stamps to be (temporarily) in the future, there is no way around
    // that.
    if (trialTime + bufferingDelay_ < lastROSTime_ + MIN_EVENT_DELTA_T) {
      bufferingDelay_ = (int64_t)(lastROSTime_ + MIN_EVENT_DELTA_T) - (int64_t)trialTime;
    }

    const uint64_t rosTimeOffset = rosT0_ + avg_timediff_int + bufferingDelay_;

    return (rosTimeOffset);
  }

  inline bool waitForGoodTimestamp(double sensorElapsedTime)
  {
    if (sensorElapsedTime == 0 && syncMode_ == "secondary") {
      const rclcpp::Time t = node_->now();
      if (lastReadyTime_ < t - readyIntervalTime_) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "secondary waiting for primary to come up");
        std_msgs::msg::Header header;
        header.stamp = t;
        header.frame_id = frameId_;
        secondaryReadyPub_->publish(header);
        lastReadyTime_ = t;
      }
      return (true);
    }
    return (false);
  }
  // ---------  variables
  rclcpp::Node * node_;
  std::shared_ptr<MetavisionWrapper> wrapper_;
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr secondaryReadyPub_;
  std::unique_ptr<MsgType> msg_;
  uint64_t messageTimeThreshold_;    // duration (nsec) for triggering a ROS message
  uint64_t t0_{0};                   // time base
  int width_;                        // image width
  int height_;                       // image height
  std::string frameId_;
  std::string syncMode_;             // primary, secondary, standalone
  size_t reserveSize_{0};            // how many events to preallocate per message
  uint64_t seq_{0};                  // sequence number for gap detection
  uint64_t rosT0_{0};                // time when first callback happened
  double averageTimeDifference_{0};  // average of elapsed_ros_time - elapsed_sensor_time
  double prevSensorTime_{0};         // sensor time during previous update
  int64_t bufferingDelay_{0};        // estimate of buffering delay
  uint64_t rosTimeOffset_{0};        // roughly rosT0_ + averageTimeDifference_
  uint64_t lastROSTime_{0};          // the last event's ROS time stamp
  bool isBigEndian_{false};
  rclcpp::Duration readyIntervalTime_;  // frequency of publishing ready messages
  rclcpp::Time lastReadyTime_;          // last time ready message was published
#ifdef DEBUG_PERFORMANCE
  std::chrono::microseconds dt_{0};  // total time spent in ros calls (perf debugging)
  std::chrono::high_resolution_clock::time_point startTime_;
  size_t msgCnt_{0};
#endif
};

event_array_msgs::msg::EventArray * allocate_message(
  uint64_t time_base, uint64_t stamp, uint16_t width, uint16_t height, const std::string & frameId,
  bool isBigEndian, size_t reserve)
{
  auto msg = new event_array_msgs::msg::EventArray();
  msg->header.frame_id = frameId;
  msg->header.stamp = rclcpp::Time(stamp, RCL_SYSTEM_TIME);
  msg->width = width;
  msg->height = height;
  msg->is_bigendian = isBigEndian;
  msg->encoding = "mono";
  msg->time_base = time_base;
  msg->events.reserve(reserve * 8);  // 8 bytes per event
  return (msg);
}

inline size_t resize_message(event_array_msgs::msg::EventArray * msg, size_t n)
{
  const size_t oldSize = msg->events.size();
  const size_t newSize = oldSize + n * 8;
  msg->events.resize(newSize);
  return (oldSize);
}

template <>
void EventPublisherROS2<event_array_msgs::msg::EventArray>::publish(
  const Metavision::EventCD * start, const Metavision::EventCD * end)
{
  const double sensorElapsedTime = start->t * 1e3;  // nanosec
  if (waitForGoodTimestamp(sensorElapsedTime)) {
    return;
  }
  const uint64_t sensorElapsedTimeInt = static_cast<uint64_t>(sensorElapsedTime);
  const size_t n = end - start;
  int eventCount[2] = {0, 0};
  if (pub_->get_subscription_count() > 0 && sensorElapsedTime > SKIP_TIME) {
    if (!msg_) {  // must allocate new message
      rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
      msg_.reset(allocate_message(
        sensorElapsedTimeInt, rosTimeOffset_ + sensorElapsedTimeInt, width_, height_, frameId_,
        isBigEndian_, reserveSize_));
      msg_->seq = seq_++;
    }
    // If capacity is sufficient the resize should not trigger a copy
    const size_t old_size = resize_message(msg_.get(), n);

    // Copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is n = 320
    uint64_t * pyxt = reinterpret_cast<uint64_t *>(&(msg_->events[old_size]));
    const uint64_t headerStamp = rclcpp::Time(msg_->header.stamp).nanoseconds();

    for (unsigned int i = 0; i < n; i++) {
      const auto & e = start[i];
      const uint64_t ts = rosTimeOffset_ + static_cast<uint64_t>(e.t * 1e3);
      const uint32_t dt = static_cast<uint32_t>((ts - headerStamp) & 0xFFFFFFFFULL);
      event_array_msgs::mono::encode(pyxt + i, e.p, e.x, e.y, dt);
      eventCount[e.p]++;
    }

    lastROSTime_ = rosTimeOffset_ + static_cast<uint64_t>(start[n - 1].t * 1e3);
    if (lastROSTime_ > headerStamp + messageTimeThreshold_) {
#ifdef DEBUG_PERFORMANCE
      auto t_start = std::chrono::high_resolution_clock::now();
#endif
      wrapper_->updateEventsSent(msg_->events.size() / 8);
      wrapper_->updateMsgsSent(1);
      // the move() will reset msg_ and transfer ownership
      pub_->publish(std::move(msg_));

#ifdef DEBUG_PERFORMANCE
      msgCnt_++;
      auto t_stop = std::chrono::high_resolution_clock::now();
      dt_ = dt_ + std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start);
#endif
    }
#ifdef DEBUG_PERFORMANCE
    if (msgCnt_ >= 1000) {
      auto t_now = std::chrono::high_resolution_clock::now();
      auto dt_tot =
        std::chrono::duration_cast<std::chrono::microseconds>(t_now - startTime_).count();
      std::cout << "call duration [us]: " << dt_.count() / msgCnt_
                << " rate: " << (msgCnt_ * 1e6) / dt_tot << std::endl;
      msgCnt_ = 0;
      dt_ = std::chrono::microseconds::zero();
      startTime_ = t_now;
    }
#endif
  } else {
    // no subscribers: clear out unfinished message and gather event statistics
    msg_.reset();
    for (unsigned int i = 0; i < n; i++) {
      eventCount[start[i].p]++;
    }
  }
  wrapper_->updateEventCount(0, eventCount[0]);
  wrapper_->updateEventCount(1, eventCount[1]);
}

}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__EVENT_PUBLISHER_ROS2_H_
