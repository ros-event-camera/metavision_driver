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
#include <ros/ros.h>

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
  void shutdown();
  bool startCamera();
  void runtimeErrorCallback(const Metavision::CameraException & e);
  void statusChangeCallback(const Metavision::CameraStatus & s);
  void updateStatistics(const EventCD * start, const EventCD * end);
  void eventCallback(const EventCD * start, const EventCD * end);

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
  dvs_msgs::EventArray msg_;
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
