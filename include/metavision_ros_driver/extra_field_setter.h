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

#ifndef METAVISION_ROS_DRIVER__EXTRA_FIELD_SETTER_H_
#define METAVISION_ROS_DRIVER__EXTRA_FIELD_SETTER_H_

#include "metavision_ros_driver/encoder.h"
#include "metavision_ros_driver/event_array_typedef.h"
#include "metavision_ros_driver/message_state.h"
#include "metavision_ros_driver/ros1_ros2_compatibility.h"

namespace metavision_ros_driver
{
template <typename MsgType, class MVEventType>
struct ExtraFieldSetter
{
  // default implementation is for DVS and Prophesee: no-op
  static inline void setExtraFields(MessageState<MsgType> *, int64_t, bool) {}
};

// partially specialize for EventArray
template <class MVEventType>
struct ExtraFieldSetter<EventArray, MVEventType>
{
  // set extra fields for EventArray
  static inline void setExtraFields(
    MessageState<EventArray> * state, int64_t sensorElapsedTime, bool isBigEndian)
  {
    setCommonExtraFields(state, sensorElapsedTime, isBigEndian);
    state->msg->encoding = Encoder<EventArray, MVEventType>::getEncoding();
  }
  static void setCommonExtraFields(
    MessageState<EventArray> * state, int64_t sensorElapsedTime, bool isBigEndian)
  {
    auto & msg = state->msg;
    msg->is_bigendian = isBigEndian;
    msg->time_base = sensorElapsedTime;  // to allow original time stamp reconstruction
    msg->seq = state->seq++;
  }
};

}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__EXTRA_FIELD_SETTER_H_
