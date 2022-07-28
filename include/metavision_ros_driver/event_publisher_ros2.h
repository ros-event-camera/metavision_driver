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

#include <algorithm>  // clamp
#include <chrono>
#include <event_array_msgs/msg/event_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "metavision_ros_driver/callback_handler.h"
#include "metavision_ros_driver/metavision_wrapper.h"
#include "metavision_ros_driver/ros_time_keeper.h"

// #define DEBUG_PERFORMANCE

namespace metavision_ros_driver
{
template <class MsgType>
class EventPublisherROS2 : public CallbackHandler
{
public:
  EventPublisherROS2(
    rclcpp::Node * node, const std::shared_ptr<MetavisionWrapper> & wrapper,
    const std::string & frameId, bool publishTrigger)
  : node_(node),
    wrapper_(wrapper),
    rosTimeKeeper_(node->get_name()),
    frameId_(frameId),
    readyIntervalTime_(rclcpp::Duration::from_seconds(1.0))
  {
    // set up event message state
    double ett;
    node->get_parameter_or("event_message_time_threshold", ett, 100e-6);
    RCLCPP_INFO_STREAM(node->get_logger(), "event message time threshold: " << ett << "s");
    eventState_.msgThreshold = static_cast<uint64_t>(1e9 * ett);
    double mmevs;
    node->get_parameter_or("sensor_max_mevs", mmevs, 50.0);
    eventState_.reserveSize = static_cast<size_t>(mmevs * 1.0e6 * ett);
    RCLCPP_INFO_STREAM(node->get_logger(), "using event reserve size: " << eventState_.reserveSize);
    // set up trigger message state
    double ttt;
    node->get_parameter_or("trigger_message_time_threshold", ttt, 100e-6);
    RCLCPP_INFO_STREAM(node->get_logger(), "trigger message time threshold: " << ttt << "s");
    triggerState_.msgThreshold = static_cast<uint64_t>(1e9 * ttt);
    double ttmf;
    node->get_parameter_or("trigger_max_freq", ttmf, 1000.0);
    triggerState_.reserveSize = static_cast<size_t>(ttmf * ttt);

#ifdef DEBUG_PERFORMANCE
    startTime_ = std::chrono::high_resolution_clock::now();
#endif
    auto qosProf = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort().durability_volatile();

    if (publishTrigger) {
      triggerState_.pub = node->create_publisher<MsgType>("~/trigger", qosProf);
    }
    eventState_.pub = node->create_publisher<MsgType>("~/events", qosProf);
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

  void eventCallback(const Metavision::EventCD * start, const Metavision::EventCD * end) override
  {
    const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      // I'm the secondary and the primary is not running yet, so my time stamps
      // are bad (0)
      return;
    }
    MsgState & state = eventState_;
    const size_t n = end - start;
    int eventCount[2] = {0, 0};
    if (state.pub->get_subscription_count() > 0) {
      if (!state.msg) {
        // update the difference between ROS time and sensor time.
        // Only do so on message start
        rosTimeOffset_ =
          rosTimeKeeper_.updateROSTimeOffset(sensorElapsedTime, node_->now().nanoseconds());
      }
      allocateMessageIfNeeded(&state, sensorElapsedTime, width_, height_);
      auto & events = state.msg->events;
      const size_t old_size = events.size();
      // With proper reserved capacity, the resize should not trigger a copy.
      events.resize(events.size() + n);
      // copy data into ROS message. For the SilkyEvCam
      // the full load packet size delivered by the SDK is 320
      for (unsigned int i = 0; i < n; i++) {
        const auto & e_src = start[i];
        auto & e_trg = events[i + old_size];
        e_trg.x = e_src.x;
        e_trg.y = e_src.y;
        e_trg.polarity = e_src.p;
        e_trg.ts = rclcpp::Time(state.rosTimeOffset + e_src.t * 1000, RCL_SYSTEM_TIME);
        eventCount[e_src.p]++;
      }
      // must keep the rostime of the last event for maintaining
      // the offset
      const int64_t lastEventTime = start[n - 1].t * 1000;
      rosTimeKeeper_.setLastROSTime(rosTimeOffset_ + lastEventTime);
      (void)sendMessageIfComplete(&state, lastEventTime, events.size());
    } else {
      // no subscribers: discard unfinished message and gather event statistics
      state.msg.reset();
      for (unsigned int i = 0; i < n; i++) {
        eventCount[start[i].p]++;
      }
    }
    wrapper_->updateEventCount(0, eventCount[0]);
    wrapper_->updateEventCount(1, eventCount[1]);
  }

  void triggerCallback(
    const Metavision::EventExtTrigger * start, const Metavision::EventExtTrigger * end) override
  {
    const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      // I'm the secondary and the primary is not running yet, so my time stamps
      // are bad (0)
      return;
    }
    int eventCount[2] = {0, 0};
    const size_t n = end - start;
    MsgState & state = triggerState_;
    if (state.pub->get_subscription_count() > 0) {
      allocateMessageIfNeeded(&state, sensorElapsedTime, 1 /* width */, 1 /*height */);
      auto & events = state.msg->events;
      const size_t old_size = events.size();
      // With proper reserved capacity, the resize should not trigger a copy.
      events.resize(events.size() + n);
      // copy data into ROS message. This part differs between trigger and events
      for (unsigned int i = 0; i < n; i++) {
        const auto & e_src = start[i];
        auto & e_trg = events[i + old_size];
        e_trg.x = 0;
        e_trg.y = 0;
        e_trg.polarity = e_src.p;
        e_trg.ts = rclcpp::Time(state.rosTimeOffset + e_src.t * 1000, RCL_SYSTEM_TIME);
        eventCount[e_src.p]++;
      }
      (void)sendMessageIfComplete(&state, start[n - 1].t * 1000, events.size());
    } else {
      // no subscribers: discard unfinished message and gather event statistics
      state.msg.reset();
      for (unsigned int i = 0; i < n; i++) {
        eventCount[start[i].p]++;
      }
    }
    wrapper_->updateEventCount(0, eventCount[0]);
    wrapper_->updateEventCount(1, eventCount[1]);
  }

  bool keepRunning() override { return (rclcpp::ok()); }

private:
  // MsgState holds the message and all other
  // pieces that are needed to build and send the message
  struct MsgState
  {
    uint64_t seq{0};            // sequence number
    uint64_t rosTimeOffset{0};  // rosTimeOffset for current message
    size_t reserveSize{0};
    uint64_t msgThreshold;                               // min duration (nsec) until msg is sent
    std::unique_ptr<MsgType> msg;                        // pointer to message itself
    typename rclcpp::Publisher<MsgType>::SharedPtr pub;  // ros publisher
  };

  // in a synchronization scenario, the secondary will get sensor timestamps
  // with value 0 until it receives a sync signal from the primary.
  // The primary however must start the camera stream *after* the secondary
  // in order for the two to have correct timestamps. So the primary will
  // not start up until it gets the "ready" message from the secondary
  inline bool waitForGoodTimestamp(int64_t sensorElapsedTime)
  {
    if (sensorElapsedTime == 0 && syncMode_ == "secondary") {
      // secondary does not see sync signal from the master
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

  void allocateMessageIfNeeded(MsgState * state, int64_t sensorElapsedTime, int width, int height)
  {
    auto & msg = state->msg;
    if (!state->msg) {  // must allocate new message
                        // remember the current rostime offset such that it will be kept
                        // constant until this message is sent out.
      state->rosTimeOffset = rosTimeOffset_;
      msg.reset(new MsgType());
      msg->header.frame_id = frameId_;
      msg->width = width;
      msg->height = height;
      msg->header.stamp = rclcpp::Time(state->rosTimeOffset + sensorElapsedTime, RCL_SYSTEM_TIME);
      msg->events.reserve(state->reserveSize * 2);  // * 2 for good measure
    }
  }

  // same as above, but with a few extras because of the richer message type
  void allocateMessageIfNeeded(
    MsgState * state, int64_t sensorElapsedTime, int width, int height,
    const std::string & encoding)
  {
    auto & msg = state->msg;
    if (!state->msg) {  // must allocate new message
      // remember the current rostime offset such that it will be kept
      // constant until this message is sent out.
      state->rosTimeOffset = rosTimeOffset_;
      msg.reset(new event_array_msgs::msg::EventArray());
      msg->header.frame_id = frameId_;
      msg->width = width;
      msg->height = height;
      msg->header.stamp = rclcpp::Time(state->rosTimeOffset + sensorElapsedTime, RCL_SYSTEM_TIME);
      msg->events.reserve(state->reserveSize * 8);  // 8 bytes per event
      msg->is_bigendian = isBigEndian_;
      msg->encoding = encoding;
      msg->time_base = sensorElapsedTime;  // to allow original time stamp reconstruction
      msg->seq = state->seq++;
    }
  }

  inline bool sendMessageIfComplete(MsgState * state, int64_t last_event_time, size_t events_sent)
  {
    const uint64_t latestTime = state->rosTimeOffset + last_event_time;
    const rclcpp::Time msgStartTime(state->msg->header.stamp);
    if (latestTime >= msgStartTime.nanoseconds() + state->msgThreshold) {
      wrapper_->updateEventsSent(events_sent);
      wrapper_->updateMsgsSent(1);
      // the std::move should reset the message
      state->pub->publish(std::move(state->msg));
      return (true);
    }
    return (false);
  }
  // ---------  variables
  rclcpp::Node * node_;
  std::shared_ptr<MetavisionWrapper> wrapper_;
  ROSTimeKeeper rosTimeKeeper_;
  uint64_t rosTimeOffset_{0};  // roughly ros_start_time + avg diff elapsed (ros - sensor)
  int width_;                  // image width
  int height_;                 // image height
  std::string frameId_;
  bool isBigEndian_{false};
  // ------- state related to message publishing
  MsgState eventState_;    // state for sending event message
  MsgState triggerState_;  // state for sending trigger message
  // ------ related to sync
  rclcpp::Duration readyIntervalTime_;  // frequency of publishing ready messages
  rclcpp::Time lastReadyTime_;          // last time ready message was published
  std::string syncMode_;                // primary, secondary, standalone
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr secondaryReadyPub_;
  // ------- misc other stuff
#ifdef DEBUG_PERFORMANCE
  std::chrono::microseconds dt_{0};  // total time spent in ros calls (perf debugging)
  std::chrono::high_resolution_clock::time_point startTime_;
#endif
  size_t msgCnt_{0};
};

inline size_t resize_message(event_array_msgs::msg::EventArray * msg, size_t n)
{
  const size_t oldSize = msg->events.size();
  const size_t newSize = oldSize + n * 8;
  msg->events.resize(newSize);
  return (oldSize);
}

template <>
void EventPublisherROS2<event_array_msgs::msg::EventArray>::eventCallback(
  const Metavision::EventCD * start, const Metavision::EventCD * end)
{
  const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
  if (waitForGoodTimestamp(sensorElapsedTime)) {
    return;
  }
  int eventCount[2] = {0, 0};
  const size_t n = end - start;
  MsgState & state = eventState_;

  if (state.pub->get_subscription_count() > 0) {
    if (!state.msg) {
      // update the difference between ROS time and sensor time.
      // Only do so on message start
      rosTimeOffset_ =
        rosTimeKeeper_.updateROSTimeOffset(sensorElapsedTime, node_->now().nanoseconds());
    }

    allocateMessageIfNeeded(&state, sensorElapsedTime, width_, height_, "mono");
    // If capacity is sufficient the resize should not trigger a copy
    const size_t old_size = resize_message(state.msg.get(), n);

    // Copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is n = 320
    auto & events = state.msg->events;
    uint64_t * pyxt = reinterpret_cast<uint64_t *>(&(events[old_size]));
    const uint64_t headerStamp = rclcpp::Time(state.msg->header.stamp).nanoseconds();

    for (unsigned int i = 0; i < n; i++) {
      const auto & e = start[i];
      const uint64_t ts = state.rosTimeOffset + e.t * 1000;
      const uint32_t dt = static_cast<uint32_t>((ts - headerStamp) & 0xFFFFFFFFULL);
#ifdef CHECK_IF_OUTSIDE_ROI
      wrapper_->checkROI(e.x, e.y);
#endif
      event_array_msgs::mono::encode(pyxt + i, e.p, e.x, e.y, dt);
      eventCount[e.p]++;
    }
    // update lastROSTime with latest event time stamp
    const int64_t lastEventTime = start[n - 1].t * 1000;
    rosTimeKeeper_.setLastROSTime(rosTimeOffset_ + lastEventTime);

#ifdef DEBUG_PERFORMANCE
    auto t_start = std::chrono::high_resolution_clock::now();
    bool msgSent = sendMessageIfComplete(&state, lastEventTime, events.size() / 8);
    if (msgSent) {
      auto t_stop = std::chrono::high_resolution_clock::now();
      dt_ = dt_ + std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start);
      msgCnt_++;
    }
#else
    (void)sendMessageIfComplete(&state, lastEventTime, events.size() / 8);
#endif
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
    state.msg.reset();
    for (unsigned int i = 0; i < n; i++) {
      eventCount[start[i].p]++;
    }
  }
  wrapper_->updateEventCount(0, eventCount[0]);
  wrapper_->updateEventCount(1, eventCount[1]);
}

template <>
void EventPublisherROS2<event_array_msgs::msg::EventArray>::triggerCallback(
  const Metavision::EventExtTrigger * start, const Metavision::EventExtTrigger * end)
{
  const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
  if (waitForGoodTimestamp(sensorElapsedTime)) {
    // I'm the secondary and the primary is not running yet, so my time stamps
    // are bad (0)
    return;
  }
  int eventCount[2] = {0, 0};
  const size_t n = end - start;
  MsgState & state = triggerState_;

  if (state.pub->get_subscription_count() > 0) {
    allocateMessageIfNeeded(&state, sensorElapsedTime, 1 /* width */, 1 /*height */, "trigger");
    // If capacity is sufficient the resize should not trigger a copy
    const size_t old_size = resize_message(state.msg.get(), n);

    auto & events = state.msg->events;
    uint64_t * pyxt = reinterpret_cast<uint64_t *>(&(events[old_size]));
    const uint64_t headerStamp = rclcpp::Time(state.msg->header.stamp).nanoseconds();

    // Copy data into ROS message.
    for (unsigned int i = 0; i < n; i++) {
      const auto & e = start[i];
      const uint64_t ts = state.rosTimeOffset + e.t * 1000;
      const uint32_t dt = static_cast<uint32_t>((ts - headerStamp) & 0xFFFFFFFFULL);
      event_array_msgs::trigger::encode(pyxt + i, e.p, dt);
      eventCount[e.p]++;
    }
    const int64_t lastEventTime = start[n - 1].t * 1000;
    (void)sendMessageIfComplete(&state, lastEventTime, events.size() / 8);
  } else {
    // no subscribers: clear out unfinished message and gather event statistics
    state.msg.reset();
    for (unsigned int i = 0; i < n; i++) {
      eventCount[start[i].p]++;
    }
  }
  wrapper_->updateEventCount(0, eventCount[0]);
  wrapper_->updateEventCount(1, eventCount[1]);
}

}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__EVENT_PUBLISHER_ROS2_H_
