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

#ifndef METAVISION_ROS_DRIVER__EVENT_ARRAY_TYPEDEF_H_
#define METAVISION_ROS_DRIVER__EVENT_ARRAY_TYPEDEF_H_

#include "metavision_ros_driver/ros1_ros2_compatibility.h"
#ifdef USING_ROS_1
#include <event_array_msgs/EventArray.h>
#else
#include <event_array_msgs/msg/event_array.hpp>
#endif

namespace metavision_ros_driver
{
typedef GENERIC_ROS_MESSAGE_TYPE(event_array_msgs, EventArray) EventArray;
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__EVENT_ARRAY_TYPEDEF_H_