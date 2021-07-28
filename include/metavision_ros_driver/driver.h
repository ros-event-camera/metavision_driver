// -*-c++-*--------------------------------------------------------------------
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

#ifndef METAVISION_ROS_DRIVER_H_
#define METAVISION_ROS_DRIVER_H_

#include <dvs_msgs/EventArray.h>
#include <metavision/sdk/driver/camera.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace metavision_ros_driver
{
class Driver
{
public:
  using EventCD = Metavision::EventCD;

  Driver(const ros::NodeHandle & nh);
  ~Driver();
  bool initialize();

private:
  enum EventMsgMode { DVS, PROPHESEE };
  void shutdown();
  bool startCamera();
  void runtimeErrorCallback(const Metavision::CameraException & e);
  void statusChangeCallback(const Metavision::CameraStatus & s);
  void updateStatistics(const EventCD * start, const EventCD * end);
  void eventCallback(const EventCD * start, const EventCD * end);
  template <class T>
  void init_message(
    boost::shared_ptr<T> * msg, const std::string & frameId, int width,
    int height)
  {
    *msg = boost::make_shared<T>();
    (*msg)->header.frame_id = frameId;
    (*msg)->width = width;
    (*msg)->height = height;
    (*msg)->header.seq = 0;
  }
  template <typename T>
  void updateAndPublish(
    const boost::shared_ptr<T> & msg, const EventCD * start,
    const EventCD * end)
  {
    const size_t n = end - start;
    auto & events = msg->events;
    const size_t old_size = events.size();
    // This reallocate is certainly inefficient!
    events.resize(events.size() + n);
    // copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is 320
    for (unsigned int i = 0; i < n; i++) {
      const auto & e_src = start[i];
      auto & e_trg = events[i + old_size];
      e_trg.x = e_src.x;
      e_trg.y = e_src.y;
      e_trg.polarity = e_src.p;
      e_trg.ts.fromNSec(t0_ + e_src.t * 1e3);
    }
    const ros::Time & t_msg = msg->events.begin()->ts;
    const ros::Time & t_last = msg->events.rbegin()->ts;
    if (t_last > t_msg + messageTimeThreshold_) {
      msg->header.seq++;
      msg->header.stamp = t_msg;
      eventPublisher_.publish(msg);
      totalEventsSent_ += events.size();
      totalMsgsSent_++;
      events.clear();
    }
  }

  // ------------ variables
  ros::NodeHandle nh_;
  ros::Publisher eventPublisher_;
  Metavision::Camera cam_;
  Metavision::CallbackId statusChangeCallbackId_;
  bool statusChangeCallbackActive_{false};
  Metavision::CallbackId runtimeErrorCallbackId_;
  bool runtimeErrorCallbackActive_{false};
  Metavision::CallbackId contrastCallbackId_;
  bool contrastCallbackActive_{false};
  uint64_t t0_;  // base for time stamp calc
  // time span that will trigger a message to be send
  ros::Duration messageTimeThreshold_;
  boost::shared_ptr<dvs_msgs::EventArray> dvsMsg_;
  boost::shared_ptr<prophesee_event_msgs::EventArray> propheseeMsg_;
  EventMsgMode msgMode_;
  // related to statistics
  int64_t statisticsPrintInterval_{1000000};
  float maxRate_{0};
  uint64_t totalEvents_{0};
  float totalTime_{0};
  int64_t lastPrintTime_{0};
  size_t totalMsgsSent_{0};
  size_t totalEventsSent_{0};
};
}  // namespace metavision_ros_driver
#endif
