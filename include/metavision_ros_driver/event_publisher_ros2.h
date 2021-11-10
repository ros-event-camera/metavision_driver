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

//#define DEBUG_PERFORMANCE

namespace metavision_ros_driver
{
template <class MsgType>
class EventPublisherROS2 : public CallbackHandler
{
public:
  EventPublisherROS2(
    rclcpp::Node * node, const std::shared_ptr<MetavisionWrapper> & wrapper,
    const std::string & frameId)
  : node_(node), wrapper_(wrapper), messageTimeThreshold_(0, 0), frameId_(frameId)
  {
    const double mtt = node->declare_parameter<double>("message_time_threshold", 100e-6);
    RCLCPP_INFO_STREAM(node->get_logger(), "using message time threshold: " << mtt << "s");
    messageTimeThreshold_ = rclcpp::Duration::from_nanoseconds((uint64_t)(1e9 * mtt));
    reserveSize_ =
      (size_t)(node->declare_parameter<double>("sensors_max_mevs", 50.0) / std::max(mtt, 1e-6));
    RCLCPP_INFO_STREAM(node->get_logger(), "using reserve size: " << reserveSize_);
    auto qosProf = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
#ifdef DEBUG_PERFORMANCE
    startTime_ = std::chrono::high_resolution_clock::now();
#endif
    pub_ = node->create_publisher<MsgType>("~/events", qosProf);

    width_ = wrapper_->getWidth();
    height_ = wrapper_->getHeight();
    const union {
      uint32_t i;
      char c[4];
    } combined_int = {0x01020304};  // from stackoverflow
    isBigEndian_ = (combined_int.c[0] == 1);
  }

  ~EventPublisherROS2() {}

  void publish(const Metavision::EventCD * start, const Metavision::EventCD * end) override
  {
    if (t0_ == 0) {
      t0_ = node_->now().nanoseconds();
    }
    const size_t n = end - start;
    int eventCount[2] = {0, 0};
    if (pub_->get_subscription_count() > 0) {
      if (!msg_) {  // must allocate new message
        msg_.reset(new MsgType());
        msg_->header.frame_id = frameId_;
        msg_->width = width_;
        msg_->height = height_;
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
        e_trg.ts = rclcpp::Time(t0_ + (uint64_t)(e_src.t * 1e3), RCL_SYSTEM_TIME);
        eventCount[e_src.p]++;
      }
      const rclcpp::Time t_msg(msg_->events.begin()->ts);
      const rclcpp::Time t_last(msg_->events.rbegin()->ts);
      if (t_last > t_msg + messageTimeThreshold_) {
        msg_->header.stamp = t_msg;
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
  // ---------  variables
  rclcpp::Node * node_;
  std::shared_ptr<MetavisionWrapper> wrapper_;
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
  std::unique_ptr<MsgType> msg_;
  rclcpp::Duration messageTimeThreshold_;  // duration for triggering a message
  uint64_t t0_{0};                         // time base
  int width_;                              // image width
  int height_;                             // image height
  std::string frameId_;
  size_t reserveSize_{0};  // how many events to preallocate per message
  uint64_t seq_{0};        // sequence number for gap detection
  bool isBigEndian_{false};
#ifdef DEBUG_PERFORMANCE
  std::chrono::microseconds dt_{0};  // total time spent in ros calls (perf debugging)
  std::chrono::high_resolution_clock::time_point startTime_;
  size_t msgCnt_{0};
#endif
};

event_array_msgs::msg::EventArray * allocate_message(
  uint64_t time_base, uint16_t width, uint16_t height, const std::string & frameId,
  bool isBigEndian, size_t reserve)
{
  auto msg = new event_array_msgs::msg::EventArray();
  msg->header.frame_id = frameId;
  msg->header.stamp = rclcpp::Time(time_base, RCL_SYSTEM_TIME);
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
  if (t0_ == 0) {
    t0_ = node_->now().nanoseconds();
  }
  const size_t n = end - start;
  int eventCount[2] = {0, 0};
  if (pub_->get_subscription_count() > 0) {
    if (!msg_) {  // must allocate new message
      const uint64_t time_base = t0_ + (uint64_t)(start->t * 1e3);
      msg_.reset(
        allocate_message(time_base, width_, height_, frameId_, isBigEndian_, reserveSize_));
      msg_->seq = seq_++;
    }
    // The resize should not trigger a
    // copy if the capacity is sufficient
    const size_t old_size = resize_message(msg_.get(), n);

    // Copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is n = 320
    uint64_t * pyxt = reinterpret_cast<uint64_t *>(&(msg_->events[old_size]));
    const uint64_t t_base = msg_->time_base;
    for (unsigned int i = 0; i < n; i++) {
      const auto & e = start[i];
      const uint64_t ts = t0_ + (uint64_t)(e.t * 1e3);
      const uint32_t dt = static_cast<uint32_t>((ts - t_base) & 0xFFFFFFFFULL);
      event_array_msgs::mono::encode(pyxt + i, e.p, e.x, e.y, dt);
      eventCount[e.p]++;
    }

    const rclcpp::Time t_last(t0_ + (uint64_t)(start[n - 1].t * 1e3), RCL_SYSTEM_TIME);
    if (t_last > rclcpp::Time(msg_->time_base, RCL_SYSTEM_TIME) + messageTimeThreshold_) {
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
