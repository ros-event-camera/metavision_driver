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

#ifndef METAVISION_ROS_DRIVER__EVENT_PUBLISHER_H_
#define METAVISION_ROS_DRIVER__EVENT_PUBLISHER_H_

#include <chrono>
#include <memory>
#include <string>

#include "metavision_ros_driver/check_endian.h"
#include "metavision_ros_driver/encoder.h"
#include "metavision_ros_driver/event_publisher_base.h"
#include "metavision_ros_driver/extra_field_setter.h"
#include "metavision_ros_driver/message_state.h"
#include "metavision_ros_driver/metavision_wrapper.h"
#include "metavision_ros_driver/ros1_ros2_compatibility.h"
#include "metavision_ros_driver/ros_time_keeper.h"
#include "metavision_ros_driver/synchronizer.h"

template<typename V>
void resize_hack(V& v, size_t newSize){
	struct vt {typename V::value_type v; vt() {}};
	static_assert(sizeof(vt[10]) ==sizeof(typename V::value_type[10]), "alignment error");
	typedef std::vector<vt, typename std::allocator_traits<typename V::allocator_type>::template rebind_alloc<vt>> V2;
	reinterpret_cast<V2&>(v).resize(newSize);
}

namespace metavision_ros_driver
{
template <typename MsgType>
class EventPublisher : public EventPublisherBase
{
public:
  EventPublisher(
    GENERIC_ROS_NODE_TYPE * node, Synchronizer * sync,
    const std::shared_ptr<MetavisionWrapper> & wrapper, const std::string & frameId)
  : node_(node),
    synchronizer_(sync),
    wrapper_(wrapper),
    rosTimeKeeper_(GENERIC_ROS_GET_NODE_NAME(node)),
    frameId_(frameId)
  {
    width_ = wrapper_->getWidth();
    height_ = wrapper_->getHeight();
    isBigEndian_ = check_endian::isBigEndian();
  }

  ~EventPublisher() {}

  void setupRawState(
    size_t reserveSize, double timeThreshold, const std::string & topic, int qSize)
  {
    setupState(&packetState_, reserveSize, timeThreshold, topic, qSize);
  }

  void setupEventState(
    size_t reserveSize, double timeThreshold, const std::string & topic, int qSize)
  {
    setupState(&eventState_, reserveSize, timeThreshold, topic, qSize);
  }

  void setupTriggerState(
    size_t reserveSize, double timeThreshold, const std::string & topic, int qSize)
  {
    setupState(&triggerState_, reserveSize, timeThreshold, topic, qSize);
  }

  // ---------------- inherited from CallbackHandler -----------
  void rawDataCallback(const uint8_t* data, size_t size) override
  {
    rawDataPacketCallback(data, size);
  }
  void cdEventCallback(const Metavision::EventCD * start, const Metavision::EventCD * end) override
  {
    eventCallback<Metavision::EventCD>(start, end);
  }
  void triggerEventCallback(
    const Metavision::EventExtTrigger * start, const Metavision::EventExtTrigger * end) override
  {
    eventCallback<Metavision::EventExtTrigger>(start, end);
  }
  bool keepRunning() override { return (GENERIC_ROS_OK()); }
  // ---------------- end of inherited from CallbackHandler -----------

private:
  using MsgState = MessageState<MsgType>;

  inline void rawDataPacketCallback(const void* data, size_t size)
  {
    ros::Time ct = ros::Time::now();
    const int64_t sensorElapsedTime = ct.toNSec();  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      // I'm the secondary and the primary is not running yet, my time stamps are bad (0)
      return;
    }
    MsgState & state = packetState_;
    const size_t n = size;
    int eventCount[2] = {0, 0};
    if (GENERIC_ROS_SUBSCRIPTION_COUNT(state.pub) > 0) {
      // state.msg->encoding = "evt3";
      // state.msg->header.stamp = ct;

      allocateMessageIfNeeded<Metavision::RawData>(&state, sensorElapsedTime);

      // convert metavision events to ROS events
      auto & events = state.msg->events;
      const size_t old_size = events.size();
      // With proper reserved capacity, the resize should not trigger a copy.
      resize_hack(events, events.size() + size);
      finalReserveSize_ = std::max(finalReserveSize_, int(events.size()));
      // copy data into ROS message
      void* memblk = (void*)( events.data() + old_size );
      memcpy(memblk, data, size);

      (void)sendMessageIfComplete<Metavision::RawData>(&state, sensorElapsedTime);
    } else {
      state.msg.reset();
    }
  }

  // This is the main entry point for all callbacks (CD and trigger events)
  template <class MVEventType>
  inline void eventCallback(const MVEventType * start, const MVEventType * end)
  {
    const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      // I'm the secondary and the primary is not running yet, my time stamps are bad (0)
      return;
    }
    MsgState & state = getMsgState(MVEventType());
    const size_t n = end - start;
    int eventCount[2] = {0, 0};
    if (GENERIC_ROS_SUBSCRIPTION_COUNT(state.pub) > 0) {
      updateROSTimeIfNeeded(MVEventType(), !state.msg, sensorElapsedTime);
      allocateMessageIfNeeded<MVEventType>(&state, sensorElapsedTime);
      // convert metavision events to ROS events
      copyEventsToMessage(&state, eventCount, start, n);
      // must keep the rostime of the last event for maintaining the rostime offset
      const int64_t lastEventTime = start[n - 1].t * 1000;
      rememberLastEventTimeIfNeeded(MVEventType(), lastEventTime);
      (void)sendMessageIfComplete<MVEventType>(&state, lastEventTime);
    } else {
      // no subscribers: discard unfinished message and gather event statistics
      state.msg.reset();
      for (unsigned int i = 0; i < n; i++) {
        eventCount[start[i].p]++;
      }
    }
    wrapper_->updateEventCount(0, eventCount[0]);
    wrapper_->updateEventCount(1, eventCount[1]);
  }

  template <class MVEventType>
  void copyEventsToMessage(
    MsgState * state, int * eventCount, const MVEventType * start, const size_t n)
  {
    const uint32_t stride = Encoder<MsgType, MVEventType>::getStride();
    auto & events = state->msg->events;
    const size_t old_size = events.size();
    // With proper reserved capacity, the resize should not trigger a copy.
    resize_hack(events, events.size() + n * stride);
    finalReserveSize_ = std::max(finalReserveSize_, int(events.size()));
    // copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is 320
    uint32_t j = 0;
    for (uint32_t i = 0; i < n; i++, j += stride) {
      const auto & e_src = start[i];
      Encoder<MsgType, MVEventType>::encode(
        &events[old_size + j], e_src, state->rosTimeOffset, state->msgStartTime);
      eventCount[e_src.p]++;
    }
  }

  template <class MVEventType>
  void allocateMessageIfNeeded(MsgState * state, int64_t sensorElapsedTime)
  {
    auto & msg = state->msg;
    if (!state->msg) {
      // must allocate new message
      // remember the current rostime offset such that it will be kept
      // constant until this message is sent out.
      state->rosTimeOffset = rosTimeOffset_;
      msg.reset(new MsgType());
      msg->header.frame_id = frameId_;
      Encoder<MsgType, MVEventType>::getGeometry(width_, height_, &msg->width, &msg->height);
      state->msgStartTime = state->rosTimeOffset + sensorElapsedTime;
      msg->header.stamp = GENERIC_ROS_SYSTEM_TIME_FROM_NSEC(state->msgStartTime);
      finalReserveSize_ = std::max(finalReserveSize_, int(state->reserveSize * Encoder<MsgType, MVEventType>::getStride()));
      msg->events.reserve(finalReserveSize_);
      // populate the extra fields for EventArray messages
      ExtraFieldSetter<MsgType, MVEventType>::setExtraFields(
        state, sensorElapsedTime, isBigEndian_);
    }
  }

  template <class MVEventType>
  bool sendMessageIfComplete(MsgState * state, int64_t last_event_time)
  {
    const uint64_t latestTime = state->rosTimeOffset + last_event_time;
    if (latestTime >= state->msgStartTime + state->msgThreshold) {
      wrapper_->updateEventsSent(
        state->msg->events.size() / Encoder<MsgType, MVEventType>::getStride());
      wrapper_->updateMsgsSent(1);
      GENERIC_ROS_PUBLISH(state->pub, state->msg);
      state->msg.reset();
      return (true);
    }
    return (false);
  }

  // setup the message state
  void setupState(
    MsgState * msgState, size_t reserveSize, double timeThreshold, const std::string & topic,
    int qSize)
  {
    msgState->reserveSize = reserveSize;
    msgState->msgThreshold = static_cast<uint64_t>(1e9 * timeThreshold);
    msgState->pub = GENERIC_ROS_CREATE_PUBLISHER(node_, MsgType, topic, qSize);
  }

  // ----- get the right message state depending on event or trigger
  inline MsgState & getMsgState(Metavision::EventCD) { return (eventState_); }
  inline MsgState & getMsgState(Metavision::EventExtTrigger) { return (triggerState_); }
  inline MsgState & getMsgState(Metavision::RawData) { return (packetState_); }

  // the first argument is just a type marker for the overriding
  inline void updateROSTimeIfNeeded(
    Metavision::EventCD, bool startingNewMessage, const int64_t sensorElapsedTime)
  {
    if (startingNewMessage) {
      // update the difference between ROS time and sensor time, but only on message start
      rosTimeOffset_ =
        rosTimeKeeper_.updateROSTimeOffset(sensorElapsedTime, GENERIC_ROS_NSEC_NOW(node_));
    }
  }
  // don't update ROS time for trigger events
  inline void updateROSTimeIfNeeded(Metavision::EventExtTrigger, bool, const int64_t) {}

  inline void rememberLastEventTimeIfNeeded(Metavision::EventCD, const int64_t lastEventTime)
  {
    rosTimeKeeper_.setLastROSTime(rosTimeOffset_ + lastEventTime);
  }
  // no-op for trigger events
  inline void rememberLastEventTimeIfNeeded(Metavision::EventExtTrigger, const int64_t) {}
  inline void rememberLastEventTimeIfNeeded(Metavision::RawData, const int64_t) {}

  // in a synchronization scenario, the secondary will get sensor timestamps
  // with value 0 until it receives a sync signal from the primary.
  // The primary however must start the camera stream *after* the secondary
  // in order for the two to have correct timestamps. So the primary will
  // not start up until it gets the "ready" message from the secondary
  inline bool waitForGoodTimestamp(int64_t sensorElapsedTime)
  {
    return (sensorElapsedTime == 0 && synchronizer_->sendReadyMessage());
  }

  //  --------------------------- variables -----------------------
  GENERIC_ROS_NODE_TYPE * node_;                // ROS1 / ROS2 handle
  Synchronizer * synchronizer_;                 // sync class that does primary / secondary sync
  std::shared_ptr<MetavisionWrapper> wrapper_;  // update for camera wrapper
  ROSTimeKeeper rosTimeKeeper_;                 // keeps track of time
  uint64_t rosTimeOffset_{0};  // roughly ros_start_time + avg diff elapsed (ros - sensor)
  int width_;                  // image width
  int height_;                 // image height
  std::string frameId_;        // ROS frame id
  bool isBigEndian_{false};    // cache this to avoid recomputing
  // ------ state related to message publishing
  // these are the only variables that are templated !
  MsgState eventState_;    // state for sending event message
  MsgState triggerState_;  // state for sending trigger message
  MsgState packetState_;  // state for sending trigger message
  int finalReserveSize_{0};
};

}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__EVENT_PUBLISHER_H_
