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

#ifndef METAVISION_DRIVER__METAVISION_WRAPPER_H_
#define METAVISION_DRIVER__METAVISION_WRAPPER_H_

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

#include "metavision_driver/callback_handler.h"

namespace ph = std::placeholders;

// temporary feature to check for events outside ROI. To be removed
// when related SDK/driver/camera issues have been resolved.
// #define CHECK_IF_OUTSIDE_ROI

namespace metavision_driver
{
class MetavisionWrapper
{
public:
  struct QueueElement
  {
    QueueElement() {}
    QueueElement(const void * s, size_t n, uint64_t t) : start(s), numBytes(n), timeStamp(t) {}
    // ----- variables
    const void * start{0};
    size_t numBytes{0};
    uint64_t timeStamp{0};
  };

  struct Stats
  {
    size_t msgsSent{0};
    size_t msgsRecv{0};
    size_t bytesSent{0};
    size_t bytesRecv{0};
    size_t maxQueueSize{0};
  };

  typedef std::map<std::string, std::map<std::string, int>> HardwarePinConfig;

  explicit MetavisionWrapper(const std::string & loggerName);
  ~MetavisionWrapper();

  int getBias(const std::string & name);
  bool hasBias(const std::string & name);
  int setBias(const std::string & name, int val);
  bool initialize(bool useMultithreading, const std::string & biasFile);
  bool saveBiases();
  inline void updateMsgsSent(int inc)
  {
    std::unique_lock<std::mutex> lock(statsMutex_);
    stats_.msgsSent += inc;
  }
  inline void updateBytesSent(int inc)
  {
    std::unique_lock<std::mutex> lock(statsMutex_);
    stats_.bytesSent += inc;
  }
  bool stop();
  int getWidth() const { return (width_); }
  int getHeight() const { return (height_); }
  const std::string & getSerialNumber() const { return (serialNumber_); }
  const std::string & getSoftwareInfo() const { return (softwareInfo_); }
  const std::string & getExternalTriggerInMode() const { return (triggerInMode_); }
  const std::string & getSyncMode() const { return (syncMode_); }
  const std::string & getSensorVersion() const { return (sensorVersion_); }

  void setSerialNumber(const std::string & sn) { serialNumber_ = sn; }
  void setFromFile(const std::string & f) { fromFile_ = f; }
  void setSyncMode(const std::string & sm) { syncMode_ = sm; }
  bool startCamera(CallbackHandler * h);
  void setLoggerName(const std::string & s) { loggerName_ = s; }
  void setStatisticsInterval(double sec) { statsInterval_ = sec; }

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
  void setMIPIFramePeriod(int usec) { mipiFramePeriod_ = usec; }

  bool triggerActive() const
  {
    return (triggerInMode_ != "disabled" || triggerOutMode_ != "disabled");
  }
  bool triggerInActive() const { return (triggerInMode_ != "disabled"); }
  void setDecodingEvents(bool decodeEvents);

private:
  bool initializeCamera();
  void runtimeErrorCallback(const Metavision::CameraException & e);
  void statusChangeCallback(const Metavision::CameraStatus & s);

  void rawDataCallback(const uint8_t * data, size_t size);
  void rawDataCallbackMultithreaded(const uint8_t * data, size_t size);
  void cdCallback(const Metavision::EventCD * start, const Metavision::EventCD * end);
  void extTriggerCallback(
    const Metavision::EventExtTrigger * start, const Metavision::EventExtTrigger * end);

  void processingThread();
  void statsThread();
  void applyROI(const std::vector<int> & roi);
  void applySyncMode(const std::string & mode);
  void configureExternalTriggers(
    const std::string & mode_in, const std::string & mode_out, const int period,
    const double duty_cycle);
  void configureEventRateController(const std::string & mode, const int rate);
  void configureMIPIFramePeriod(int usec, const std::string & sensorName);
  void printStatistics();
  // ------------ variables
  CallbackHandler * callbackHandler_{0};
  Metavision::Camera cam_;
  Metavision::CallbackId statusChangeCallbackId_;
  bool statusChangeCallbackActive_{false};
  Metavision::CallbackId runtimeErrorCallbackId_;
  bool runtimeErrorCallbackActive_{false};
  Metavision::CallbackId rawDataCallbackId_;
  bool rawDataCallbackActive_{false};
  Metavision::CallbackId contrastCallbackId_;
  bool contrastCallbackActive_{false};
  Metavision::CallbackId extTriggerCallbackId_;
  bool extTriggerCallbackActive_{false};
  int width_{0};   // image width
  int height_{0};  // image height
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
  int mipiFramePeriod_{-1};
  std::string loggerName_{"driver"};
  std::vector<int> roi_;
  std::string sensorVersion_{"0.0"};
  // --  related to statistics
  double statsInterval_{2.0};  // time between printouts
  std::chrono::time_point<std::chrono::system_clock> lastPrintTime_;
  Stats stats_;
  std::mutex statsMutex_;
  std::shared_ptr<std::thread> statsThread_;

  // -----------
  // related to multi threading
  bool useMultithreading_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<QueueElement> queue_;
  std::shared_ptr<std::thread> processingThread_;
  bool keepRunning_{true};
};
}  // namespace metavision_driver
#endif  // METAVISION_DRIVER__METAVISION_WRAPPER_H_
