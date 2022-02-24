// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <ros/ros.h>

#include <memory>

#include "metavision_ros_driver/sync_test_ros1.h"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "sync_test_node");
  ros::NodeHandle pnh("~");

  metavision_ros_driver::SyncTestROS1 node(pnh);
  ros::spin();  // should not return
  return 0;
}
