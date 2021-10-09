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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "metavision_ros_driver/callback_handler.h"
#include "metavision_ros_driver/metavision_wrapper.h"

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
    messageTimeThreshold_ = rclcpp::Duration::from_nanoseconds(
      (uint64_t)(1e9 * node->declare_parameter<double>("message_time_threshold", 10e-6)));
    const int qs = node->declare_parameter<int>("send_queue_size", 1000);

    pub_ = node->create_publisher<MsgType>("~/events", qs);

    width_ = wrapper_->getWidth();
    height_ = wrapper_->getHeight();
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
        // under full load a 50 Mev/s camera will
        // produce about 5000 events in a 100us
        // time slice.
        msg_->events.reserve(6000);
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
        // the move() will reset msg_ and transfer ownership
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

  uint64_t t0_{0};  // time base
  int width_;       // image width
  int height_;      // image height
  std::string frameId_;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__EVENT_PUBLISHER_ROS2_H_
