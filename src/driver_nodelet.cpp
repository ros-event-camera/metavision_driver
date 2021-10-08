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

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <memory>

#include "metavision_ros_driver/camera_driver_ros1.h"

namespace metavision_ros_driver
{
class DriverNodelet : public nodelet::Nodelet
{
public:
  template <class T>
  std::shared_ptr<CameraDriverROS1<T>> initDriver(ros::NodeHandle & pnh)
  {
    auto ptr = std::make_shared<CameraDriverROS1<T>>(pnh);
    return (ptr);
  }

  void onInit() override
  {
    nh_ = getPrivateNodeHandle();
    const std::string msg_mode = nh_.param<std::string>("message_type", "dvs");
    ROS_INFO_STREAM("running in message mode: " << msg_mode);
    if (msg_mode == "prophesee") {
      prophDriver_ = initDriver<prophesee_event_msgs::EventArray>(nh_);
    } else if (msg_mode == "dvs") {
      dvsDriver_ = initDriver<dvs_msgs::EventArray>(nh_);
    } else {
      ROS_ERROR_STREAM("exiting due to invalid message mode: " << msg_mode);
    }
  }

private:
  // ------ variables --------
  std::shared_ptr<CameraDriverROS1<prophesee_event_msgs::EventArray>> prophDriver_;
  std::shared_ptr<CameraDriverROS1<dvs_msgs::EventArray>> dvsDriver_;
  ros::NodeHandle nh_;
};
}  // namespace metavision_ros_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(metavision_ros_driver::DriverNodelet, nodelet::Nodelet)
