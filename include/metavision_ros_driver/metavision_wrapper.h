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
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "metavision_ros_driver/callback_handler.h"

namespace ph = std::placeholders;

// temporary feature to check for events outside ROI. To be removed
// when related SDK/driver/camera issues have been resolved.
// #define CHECK_IF_OUTSIDE_ROI

namespace metavision_ros_driver
{
class MetavisionWrapper
{
public:
  using RawData = Metavision::RawData;
  using EventCD = Metavision::EventCD;
  using EventExtTrigger = Metavision::EventExtTrigger;
  enum EventType { CD, ExtTrigger, RAW };
  struct QueueElement
  {
    QueueElement() {}
    QueueElement(EventType t, const void * s, uint32_t n) : eventType(t), start(s), numEvents(n) {}
    // ----- variables
    EventType eventType{CD};
    const void * start{0};
    uint32_t numEvents{0};
  };

  typedef std::map<std::string, std::map<std::string, int>> HardwarePinConfig;

  explicit MetavisionWrapper(const std::string & loggerName);
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
  const std::string & getSoftwareInfo() const { return (softwareInfo_); }
  const std::string & getExternalTriggerInMode() const { return (triggerInMode_); }
  const std::string & getSyncMode() const { return (syncMode_); }
  void setSerialNumber(const std::string & sn) { serialNumber_ = sn; }
  void setFromFile(const std::string & f) { fromFile_ = f; }
  void setSyncMode(const std::string & sm) { syncMode_ = sm; }
  bool startCamera(CallbackHandler * h);
  void setLoggerName(const std::string & s) { loggerName_ = s; }
  // ROI is a double vector with length multiple of 4:
  // (x_top_1, y_top_1, width_1, height_1,
  //  x_top_2, y_top_2, width_2, height_2, .....)
  void setROI(const std::vector<int> & roi) { roi_ = roi; }
  void setExternalTriggerInMode(const std::string & mode) { triggerInMode_ = mode; }
  void setExternalTriggerOutMode(
    const std::string & mode, const int period, const double duty_cycle);
  void setHardwarePinConfig(const HardwarePinConfig & config) { hardwarePinConfig_ = config; }
  void setEventRateController(const std::string & mode, const int rate)
  {
    ercMode_ = mode;
    ercRate_ = rate;
  }
  void setCallbackHandler2(CallbackHandler * h) { callbackHandler2_ = h; }
  bool triggerActive() const
  {
    return (triggerInMode_ != "disabled" || triggerOutMode_ != "disabled");
  }
  bool triggerInActive() const { return (triggerInMode_ != "disabled"); }
#ifdef CHECK_IF_OUTSIDE_ROI
  inline void checkROI(uint16_t x, uint16_t y)
  {
    if (x < x_min_ || x >= x_max_ || y < y_min_ || y >= y_max_) {
      outsideROI_++;
    }
  }
#endif

private:
  bool initializeCamera();
  void runtimeErrorCallback(const Metavision::CameraException & e);
  void statusChangeCallback(const Metavision::CameraStatus & s);
  void updateStatistics(const EventCD * start, const EventCD * end);
  void extTriggerCallback(const EventExtTrigger * start, const EventExtTrigger * end);

  void eventCallback(const EventCD * start, const EventCD * end);
  void eventCallbackMultithreaded(const EventCD * start, const EventCD * end);

  void rawDataCallback(const uint8_t * data, size_t size);
  void rawDataCallbackMultithreaded(const uint8_t * data, size_t size);

  void extTriggerCallbackMultithreaded(const EventExtTrigger * start, const EventExtTrigger * end);
  void processingThread();
  void applyROI(const std::vector<int> & roi);
  void applySyncMode(const std::string & mode);
  void configureExternalTriggers(
    const std::string & mode_in, const std::string & mode_out, const int period,
    const double duty_cycle);
  void configureEventRateController(const std::string & mode, const int rate);
  // ------------ variables
  CallbackHandler * callbackHandler_{0};
  CallbackHandler * callbackHandler2_{0};  // additional callback handler
  Metavision::Camera cam_;
  Metavision::CallbackId statusChangeCallbackId_;
  bool statusChangeCallbackActive_{false};
  Metavision::CallbackId runtimeErrorCallbackId_;
  bool runtimeErrorCallbackActive_{false};
  Metavision::CallbackId contrastCallbackId_;
  bool contrastCallbackActive_{false};
  Metavision::CallbackId extTriggerCallbackId_;
  bool rawDataCallbackActive_{false};
  Metavision::CallbackId rawDataCallbackId_;
  bool extTriggerCallbackActive_{false};
  int width_{0};   // image width
  int height_{0};  // image height
  // related to statistics
  int64_t statisticsPrintInterval_{1000000};
  float maxRate_{0};
  uint64_t totalEvents_{0};
  float totalTime_{0};
  int64_t lastPrintTime_{0};
  int64_t lastEventTime_{-1};
  size_t totalMsgsSent_{0};
  size_t totalEventsSent_{0};
  size_t maxQueueSize_{0};
  uint32_t eventCount_[2];
  std::string biasFile_;
  std::string serialNumber_;
  std::string fromFile_;
  std::string softwareInfo_;
  std::string syncMode_;
  std::string triggerInMode_;   // disabled, enabled, loopback
  std::string triggerOutMode_;  // disabled, enabled
  int triggerOutPeriod_;        // period (in microseconds) of trigger out
  double triggerOutDutyCycle_;  // duty cycle (fractional) of trigger out
  HardwarePinConfig hardwarePinConfig_;
  std::string ercMode_;
  int ercRate_;
  std::string loggerName_{"driver"};
  std::vector<int> roi_;
  // related to multi threading
  bool useMultithreading_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<QueueElement> queue_;
  std::shared_ptr<std::thread> thread_;
  bool keepRunning_{true};
#ifdef CHECK_IF_OUTSIDE_ROI
  size_t outsideROI_{0};
  uint16_t x_min_;
  uint16_t x_max_;
  uint16_t y_min_;
  uint16_t y_max_;
#endif
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__METAVISION_WRAPPER_H_
