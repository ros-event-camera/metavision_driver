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
// (time is in nanoseconds)
#define SKIP_TIME 2000000000

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
    auto qosProf = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

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
    if (state.pub->get_subscription_count() > 0 && sensorElapsedTime > SKIP_TIME) {
      if (!state.msg) {
        // update the difference between ROS time and sensor time.
        // Only do so on message start
        rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
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
      lastROSTime_ = rosTimeOffset_ + lastEventTime;
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
  int width_;   // image width
  int height_;  // image height
  std::string frameId_;
  bool isBigEndian_{false};
  // ------- related to time keeping
  uint64_t rosT0_{0};                // time when first callback happened
  double averageTimeDifference_{0};  // average of elapsed_ros_time - elapsed_sensor_time
  double prevSensorTime_{0};         // sensor time during previous update
  int64_t bufferingDelay_{0};        // estimate of buffering delay
  uint64_t rosTimeOffset_{0};        // roughly rosT0_ + averageTimeDifference_
  uint64_t lastROSTime_{0};          // the last event's ROS time stamp
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
  size_t msgCnt_{0};
#endif
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

  if (state.pub->get_subscription_count() > 0 && sensorElapsedTime > SKIP_TIME) {
    if (!state.msg) {
      // update the difference between ROS time and sensor time.
      // Only do so on message start
      rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
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
      event_array_msgs::mono::encode(pyxt + i, e.p, e.x, e.y, dt);
      eventCount[e.p]++;
    }
    // update lastROSTime_ with latest event time stamp
    const int64_t lastEventTime = start[n - 1].t * 1000;
    lastROSTime_ = rosTimeOffset_ + lastEventTime;

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
