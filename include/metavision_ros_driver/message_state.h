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

#ifndef METAVISION_ROS_DRIVER__MESSAGE_STATE_H_
#define METAVISION_ROS_DRIVER__MESSAGE_STATE_H_

#include <memory>

#include "metavision_ros_driver/ros1_ros2_compatibility.h"

namespace metavision_ros_driver
{
template <typename MsgType>
struct MessageState
{
  uint64_t seq{0};            // sequence number
  uint64_t rosTimeOffset{0};  // rosTimeOffset for current message
  size_t reserveSize{0};      // estimate of best reserve size
  uint64_t msgThreshold;      // min duration (nsec) until msg is sent
  uint64_t msgStartTime{0};   // time (nsec) of start of message
  typename GENERIC_ROS_MSG_PTR(MsgType) msg;
  GENERIC_ROS_PUBLISHER(MsgType) pub;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__MESSAGE_STATE_H_
