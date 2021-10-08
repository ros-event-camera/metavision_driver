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

#ifndef METAVISION_ROS_DRIVER__METAVISION_WRAPPER_H_
#define METAVISION_ROS_DRIVER__METAVISION_WRAPPER_H_

#include <metavision/sdk/driver/camera.h>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "metavision_ros_driver/callback_handler.h"

namespace ph = std::placeholders;

namespace metavision_ros_driver
{
class MetavisionWrapper
{
public:
  using EventCD = Metavision::EventCD;
  typedef std::pair<size_t, const void *> QueueElement;

  explicit MetavisionWrapper(CallbackHandler * h);
  ~MetavisionWrapper();

  int getBias(const std::string & name);
  int setBias(const std::string & name, int val);
  bool initialize(bool useMultithreading, double statItv, const std::string & biasFile);
  bool saveBiases();
  inline void updateEventCount(int i, int inc) { eventCount_[i] += inc; }
  inline void updateEventsSent(int inc) { totalEventsSent_ += inc; }
  inline void updateMsgsSent(int inc) { totalMsgsSent_ += inc; }
  bool stop();
  int getWidth() const { return (width_); }
  int getHeight() const { return (height_); }
  const std::string & getSerialNumber() const { return (serialNumber_); }

private:
  bool startCamera();
  void runtimeErrorCallback(const Metavision::CameraException & e);
  void statusChangeCallback(const Metavision::CameraStatus & s);
  void updateStatistics(const EventCD * start, const EventCD * end);
  void eventCallback(const EventCD * start, const EventCD * end);
  void eventCallbackMultithreaded(const EventCD * start, const EventCD * end);
  void processingThread();
  // ------------ variables
  CallbackHandler * callbackHandler_{0};
  Metavision::Camera cam_;
  Metavision::CallbackId statusChangeCallbackId_;
  bool statusChangeCallbackActive_{false};
  Metavision::CallbackId runtimeErrorCallbackId_;
  bool runtimeErrorCallbackActive_{false};
  Metavision::CallbackId contrastCallbackId_;
  bool contrastCallbackActive_{false};
  int width_{0};   // image width
  int height_{0};  // image height
  // related to statistics
  int64_t statisticsPrintInterval_{1000000};
  float maxRate_{0};
  uint64_t totalEvents_{0};
  float totalTime_{0};
  int64_t lastPrintTime_{0};
  size_t totalMsgsSent_{0};
  size_t totalEventsSent_{0};
  size_t maxQueueSize_{0};
  uint32_t eventCount_[2];
  std::string biasFile_;
  std::string serialNumber_;
  std::string loggerName_{"driver"};
  // related to multi threading
  bool useMultithreading_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::pair<size_t, const void *>> queue_;
  std::shared_ptr<std::thread> thread_;
  bool keepRunning_{true};
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__METAVISION_WRAPPER_H_
