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

#ifndef METAVISION_ROS_DRIVER__RECORDER_ROS2_H_
#define METAVISION_ROS_DRIVER__RECORDER_ROS2_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace metavision_ros_driver
{
class Recorder : public rosbag2_transport::Recorder
{
public:
  explicit Recorder(const rclcpp::NodeOptions & options);
  ~Recorder();

private:
  void initialize();
  bool startRecording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ---- variables
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  bool isRecording_{false};
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__RECORDER_ROS2_H_
