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

#include "metavision_ros_driver/recorder_ros2.h"

#include <stdio.h>

#include <chrono>
#include <iomanip>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>

#include "metavision_ros_driver/logging.h"

namespace metavision_ros_driver
{
Recorder::Recorder(const rclcpp::NodeOptions & options)
: rosbag2_transport::Recorder("recorder", options)
{
  initialize();
}

void Recorder::initialize()
{
  std::stringstream datetime;
  auto now = std::chrono::system_clock::now();
  auto t_now = std::chrono::system_clock::to_time_t(now);
  datetime << std::put_time(std::localtime(&t_now), "%Y-%m-%d-%H-%M-%S");
  const std::string base_name = declare_parameter<std::string>("base_name", "rosbag2_");
  const std::string name = base_name + datetime.str();
  std::vector<std::string> topics = declare_parameter<std::vector<std::string>>("topics");
  for (const auto topic : topics) {
    LOG_INFO("recording topic: " << topic);
  }
  rosbag2_storage::StorageOptions & sopt =
    const_cast<rosbag2_storage::StorageOptions &>(storage_options());
  sopt.storage_id = declare_parameter<std::string>("storage_id", "sqlite3");
  sopt.uri = name;

  rosbag2_transport::RecordOptions & ropt =
    const_cast<rosbag2_transport::RecordOptions &>(record_options());
  ropt.all = declare_parameter<bool>("record_all", false);
  ropt.is_discovery_disabled = declare_parameter<bool>("disable_discovery", false);
  ropt.rmw_serialization_format = declare_parameter<std::string>("serialization_format", "cdr");
  ropt.topic_polling_interval = std::chrono::milliseconds(100);
  ropt.topics.insert(ropt.topics.end(), topics.begin(), topics.end());

  // stop_discovery_ = ropt.is_discovery_disabled;
  if (declare_parameter<bool>("start_recording_immediately", false)) {
    record();
  } else {
    service_ = create_service<std_srvs::srv::Trigger>(
      "start_recording",
      std::bind(&Recorder::startRecording, this, std::placeholders::_1, std::placeholders::_2));
  }
}

bool Recorder::startRecording(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  res->success = false;
  if (isRecording_) {
    LOG_WARN("already recording!");
    res->message = "already recording!";
  } else {
    LOG_INFO("starting recording!");
    try {
      record();
      isRecording_ = true;
      LOG_INFO("started recording");
      res->success = true;
      res->message = "started recoding!";
    } catch (const std::runtime_error & e) {
      LOG_ERROR("cannot toggle recording: " << e.what());
      res->message = "runtime error occurred: " + std::string(e.what());
    }
  }
  return (true);
}

Recorder::~Recorder() {}
}  // namespace metavision_ros_driver

RCLCPP_COMPONENTS_REGISTER_NODE(metavision_ros_driver::Recorder)
