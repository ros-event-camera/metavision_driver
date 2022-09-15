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

#ifndef METAVISION_ROS_DRIVER__ENCODER_H_
#define METAVISION_ROS_DRIVER__ENCODER_H_

#include <event_array_msgs/decode.h>  // for bytes_per_event
#include <event_array_msgs/encode.h>
#include <metavision/sdk/driver/camera.h>

#include "metavision_ros_driver/event_array_typedef.h"
#include "metavision_ros_driver/message_state.h"
#include "metavision_ros_driver/ros1_ros2_compatibility.h"

namespace metavision_ros_driver
{
template <typename MsgType, class MVEventType>
struct Encoder
{
  // encoding of cd vs trigger events.
  // DVS & Prophesee / EventCD are the default template
  inline static void encode(
    typename decltype(MsgType::events)::value_type * er, const MVEventType & em,
    const uint64_t rosTimeOffset, const uint64_t)
  {
    er->x = em.x;
    er->y = em.y;
    er->polarity = em.p;
    er->ts = GENERIC_ROS_SYSTEM_TIME_FROM_NSEC(rosTimeOffset + em.t * 1000);
  }

  // get encoding string
  static inline constexpr const char * getEncoding() { return ("mono"); }

  // get stride:. how many elements of type data to skip to get to the next event
  static inline constexpr uint32_t getStride() { return (1); }

  // convert image geometry to message geometry
  static inline void getGeometry(
    const uint32_t width, const uint32_t height, uint32_t * w, uint32_t * h)
  {
    *w = width;
    *h = height;
  }
};

//
// Partial specialization for ExtTriggerEvent at the struct level
//
template <typename MsgType>
struct Encoder<MsgType, Metavision::EventExtTrigger>
{
  // DVS&Prophesee / ExtTrigger partial specialization
  inline static void encode(
    typename decltype(MsgType::events)::value_type * er, const Metavision::EventExtTrigger & em,
    const uint64_t rosTimeOffset, const uint64_t)
  {
    er->x = 0;
    er->y = 0;
    er->polarity = em.p;
    er->ts = GENERIC_ROS_SYSTEM_TIME_FROM_NSEC(rosTimeOffset + em.t * 1000);
  }

  constexpr static const char * getEncoding() { return ("trigger"); }

  constexpr static uint32_t getStride() { return (event_array_msgs::trigger::bytes_per_event); }

  // for trigger messages set image size to 1 x 1
  static inline void getGeometry(const uint32_t, const uint32_t, uint32_t * w, uint32_t * h)
  {
    *w = 1;
    *h = 1;
  }
};
// EventArray / EventCD full specialization
template <>
void Encoder<EventArray, Metavision::EventCD>::encode(
  decltype(EventArray::events)::value_type * er, const Metavision::EventCD & em,
  const uint64_t rosTimeOffset, const uint64_t headerStamp)
{
  uint64_t * pyxt = reinterpret_cast<uint64_t *>(er);
  const uint64_t ts = rosTimeOffset + em.t * 1000;
  const uint32_t dt = static_cast<uint32_t>((ts - headerStamp) & 0xFFFFFFFFULL);
  event_array_msgs::mono::encode(pyxt, em.p, em.x, em.y, dt);
}

// EventArray / EventExtTrigger full specialization
template <>
void Encoder<EventArray, Metavision::EventExtTrigger>::encode(
  decltype(EventArray::events)::value_type * er, const Metavision::EventExtTrigger & em,
  const uint64_t rosTimeOffset, const uint64_t headerStamp)
{
  uint64_t * pyxt = reinterpret_cast<uint64_t *>(er);
  const uint64_t ts = rosTimeOffset + em.t * 1000;
  const uint32_t dt = static_cast<uint32_t>((ts - headerStamp) & 0xFFFFFFFFULL);
  event_array_msgs::trigger::encode(pyxt, em.p, dt);
}

// stride full specialization
template <>
constexpr uint32_t Encoder<EventArray, Metavision::EventCD>::getStride()
{
  return (event_array_msgs::mono::bytes_per_event);
}


// EventArray / RawData full specialization
template <>
void Encoder<EventArray, Metavision::RawData>::encode(
  decltype(EventArray::events)::value_type * er, const Metavision::RawData & em,
  const uint64_t rosTimeOffset, const uint64_t headerStamp)
{
}
// stride full specialization
template <>
constexpr uint32_t Encoder<EventArray, Metavision::RawData>::getStride()
{
  return (event_array_msgs::mono::bytes_per_event);
}
// stride full specialization
template <>
constexpr const char * Encoder<EventArray, Metavision::RawData>::getEncoding()
{
  return "evt3";
}

}  // namespace metavision_ros_driver

#endif  // METAVISION_ROS_DRIVER__ENCODER_H_
