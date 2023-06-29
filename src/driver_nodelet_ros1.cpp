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

#include "metavision_driver/driver_ros1.h"

namespace metavision_driver
{
class DriverNodeletROS1 : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    nh_ = getPrivateNodeHandle();
    driver_ = std::make_shared<DriverROS1>(nh_);
  }

private:
  // ------ variables --------
  std::shared_ptr<DriverROS1> driver_;
  ros::NodeHandle nh_;
};
}  // namespace metavision_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(metavision_driver::DriverNodeletROS1, nodelet::Nodelet)
