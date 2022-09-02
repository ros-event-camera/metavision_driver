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

#include "metavision_ros_driver/metavision_wrapper.h"

#include <metavision/hal/facilities/i_device_control.h>
#include <metavision/hal/facilities/i_erc.h>
#include <metavision/hal/facilities/i_plugin_software_info.h>
#include <metavision/hal/facilities/i_trigger_in.h>

#include <chrono>
#include <set>
#include <thread>

#include "metavision_ros_driver/logging.h"

namespace metavision_ros_driver
{
MetavisionWrapper::MetavisionWrapper(const std::string & loggerName)
{
  setLoggerName(loggerName);
  eventCount_[0] = 0;
  eventCount_[1] = 0;
}

MetavisionWrapper::~MetavisionWrapper() { stop(); }

int MetavisionWrapper::getBias(const std::string & name)
{
  const Metavision::Biases biases = cam_.biases();
  Metavision::I_LL_Biases * hw_biases = biases.get_facility();
  const auto pmap = hw_biases->get_all_biases();
  auto it = pmap.find(name);
  if (it == pmap.end()) {
    LOG_ERROR_NAMED("unknown bias parameter: " << name);
    throw(std::runtime_error("bias parameter not found!"));
  }
  return (it->second);
}

int MetavisionWrapper::setBias(const std::string & name, int val)
{
  std::set<std::string> dont_touch_set = {{"bias_diff"}};
  if (dont_touch_set.count(name) != 0) {
    LOG_WARN_NAMED("ignoring change to parameter: " << name);
    return (val);
  }
  Metavision::Biases & biases = cam_.biases();
  Metavision::I_LL_Biases * hw_biases = biases.get_facility();
  const int prev = hw_biases->get(name);
  if (val != prev) {
    if (!hw_biases->set(name, val)) {
      LOG_WARN_NAMED("cannot set parameter " << name << " to " << val);
    }
  }
  const int now = hw_biases->get(name);  // read back what actually took hold
  LOG_INFO_NAMED("changed  " << name << " from " << prev << " to " << val << " adj to: " << now);
  return (now);
}

bool MetavisionWrapper::initialize(
  bool useMultithreading, double statItv, const std::string & biasFile)
{
  biasFile_ = biasFile;
  useMultithreading_ = useMultithreading;

  statisticsPrintInterval_ = static_cast<int>(statItv * 1e6);
  if (!initializeCamera()) {
    LOG_ERROR_NAMED("could not initialize camera!");
    return (false);
  }
  return (true);
}

bool MetavisionWrapper::stop()
{
  bool status = false;
  if (cam_.is_running()) {
    cam_.stop();
    status = true;
  }
  if (contrastCallbackActive_) {
    cam_.cd().remove_callback(contrastCallbackId_);
  }
  if (runtimeErrorCallbackActive_) {
    cam_.remove_runtime_error_callback(runtimeErrorCallbackId_);
  }
  if (statusChangeCallbackActive_) {
    cam_.remove_status_change_callback(statusChangeCallbackId_);
  }
  if (extTriggerCallbackActive_) {
    cam_.ext_trigger().remove_callback(extTriggerCallbackId_);
  }
  if (thread_) {
    keepRunning_ = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.notify_all();
    }
    thread_->join();
    thread_.reset();
  }
  return (status);
}

void MetavisionWrapper::applyROI(const std::vector<int> & roi)
{
  if (!roi.empty()) {
    if (roi.size() % 4 != 0) {
      LOG_ERROR_NAMED("ROI vec must be multiple of 4, but is: " << roi.size());
    } else {
#ifdef CHECK_IF_OUTSIDE_ROI
      x_min_ = std::numeric_limits<uint16_t>::max();
      x_max_ = std::numeric_limits<uint16_t>::min();
      y_min_ = std::numeric_limits<uint16_t>::max();
      y_max_ = std::numeric_limits<uint16_t>::min();
#endif
      std::vector<Metavision::Roi::Rectangle> rects;
      for (size_t i = 0; i < roi.size(); i += 4) {
        Metavision::Roi::Rectangle rect;
        rect.x = roi[i];
        rect.y = roi[i + 1];
        rect.width = roi[i + 2];
        rect.height = roi[i + 3];
        rects.push_back(rect);
#ifdef CHECK_IF_OUTSIDE_ROI
        x_min_ = std::min(static_cast<uint16_t>(rect.x), x_min_);
        x_max_ = std::max(static_cast<uint16_t>(rect.x + rect.width), x_max_);
        y_min_ = std::min(static_cast<uint16_t>(rect.y), y_min_);
        y_max_ = std::max(static_cast<uint16_t>(rect.y + rect.height), y_max_);
#endif
      }
      cam_.roi().set(rects);
    }
  } else {
#ifdef CHECK_IF_OUTSIDE_ROI
    x_min_ = 0;
    x_max_ = std::numeric_limits<uint16_t>::max();
    y_min_ = 0;
    y_max_ = std::numeric_limits<uint16_t>::max();
#endif
  }
}

void MetavisionWrapper::applySyncMode(const std::string & mode)
{
  Metavision::I_DeviceControl * control =
    cam_.get_device().get_facility<Metavision::I_DeviceControl>();
  if (!control) {  // happens when playing from file
    if (mode != "standalone") {
      LOG_WARN_NAMED("cannot set sync mode to: " << mode);
    }
    return;
  }

  if (mode == "standalone") {
    if (control->get_mode() != Metavision::I_DeviceControl::SyncMode::STANDALONE) {
      control->set_mode_standalone();
    }
  } else if (mode == "primary") {
    control->set_mode_master();
  } else if (mode == "secondary") {
    control->set_mode_slave();
  } else {
    LOG_ERROR_NAMED("INVALID SYNC MODE: " << mode);
    throw std::runtime_error("invalid sync mode!");
  }
}

void MetavisionWrapper::configureExternalTriggers(
  const std::string & mode_in, const std::string & mode_out, const int period,
  const double duty_cycle)
{
  if (mode_out == "enabled") {
    Metavision::I_TriggerOut * i_trigger_out =
      cam_.get_device().get_facility<Metavision::I_TriggerOut>();
    if (i_trigger_out) {
      i_trigger_out->set_period(period);  // in usec
      i_trigger_out->set_duty_cycle(duty_cycle);
      i_trigger_out->enable();
      LOG_INFO_NAMED("Enabled trigger output");
    } else {
      LOG_ERROR_NAMED("Failed enabling trigger output");
    }
  }

  if (mode_in == "external" || mode_in == "loopback") {
    Metavision::I_TriggerIn * i_trigger_in =
      cam_.get_device().get_facility<Metavision::I_TriggerIn>();

    if (i_trigger_in) {
      int pin = hardwarePinConfig_[softwareInfo_][mode_in];
      i_trigger_in->enable(pin);
      LOG_INFO_NAMED("Enabled trigger input " << mode_in << " on " << pin);
    } else {
      LOG_ERROR_NAMED("Failed enabling trigger input");
    }
    extTriggerCallbackId_ = cam_.ext_trigger().add_callback(std::bind(
      useMultithreading_ ? &MetavisionWrapper::extTriggerCallbackMultithreaded
                         : &MetavisionWrapper::extTriggerCallback,
      this, ph::_1, ph::_2));
    extTriggerCallbackActive_ = true;
  }
}

void MetavisionWrapper::configureEventRateController(
  const std::string & mode, const int events_per_sec)
{
  if (mode == "enabled" || mode == "disabled") {
    Metavision::I_Erc * i_erc = cam_.get_device().get_facility<Metavision::I_Erc>();

    i_erc->enable(mode == "enabled");
    i_erc->set_cd_event_rate(events_per_sec);
  }
}

bool MetavisionWrapper::initializeCamera()
{
  const int num_tries = 5;
  for (int i = 0; i < num_tries; i++) {
    try {
      if (!fromFile_.empty()) {
        LOG_INFO_NAMED("reading events from file: " << fromFile_);
        cam_ = Metavision::Camera::from_file(fromFile_);
      } else {
        if (!serialNumber_.empty()) {
          cam_ = Metavision::Camera::from_serial(serialNumber_);
        } else {
          cam_ = Metavision::Camera::from_first_available();
        }
      }
      break;  // were able to open the camera, exit the for loop
    } catch (const Metavision::CameraException & e) {
      const std::string src =
        fromFile_.empty() ? (serialNumber_.empty() ? "default" : serialNumber_) : fromFile_;
      LOG_WARN_NAMED(
        "cannot open " << src << " on attempt " << i + 1 << ", retrying " << num_tries - i
                       << " more times");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  try {
    // Record the plugin software information about the camera.
    Metavision::I_PluginSoftwareInfo * psi =
      cam_.get_device().get_facility<Metavision::I_PluginSoftwareInfo>();
    softwareInfo_ = psi->get_plugin_name();
    LOG_INFO_NAMED("Plugin Software Name: " << softwareInfo_);

    if (!biasFile_.empty()) {
      try {
        cam_.biases().set_from_file(biasFile_);
        LOG_INFO_NAMED("using bias file: " << biasFile_);
      } catch (const Metavision::CameraException & e) {
        LOG_WARN_NAMED("reading bias file failed with error: " << e.what());
        LOG_WARN_NAMED("continuing with default biases!");
      }
    } else {
      LOG_INFO_NAMED("no bias file provided, using camera defaults");
    }
    // overwrite serial in case it was not set
    serialNumber_ = cam_.get_camera_configuration().serial_number;
    LOG_INFO_NAMED("camera serial number: " << serialNumber_);
    const auto & g = cam_.geometry();
    width_ = g.width();
    height_ = g.height();
    LOG_INFO_NAMED("sensor geometry: " << width_ << " x " << height_);
    if (fromFile_.empty()) {
      applySyncMode(syncMode_);
      applyROI(roi_);
      configureExternalTriggers(
        triggerInMode_, triggerOutMode_, triggerOutPeriod_, triggerOutDutyCycle_);
      configureEventRateController(ercMode_, ercRate_);
    }
    statusChangeCallbackId_ = cam_.add_status_change_callback(
      std::bind(&MetavisionWrapper::statusChangeCallback, this, ph::_1));
    statusChangeCallbackActive_ = true;
    runtimeErrorCallbackId_ = cam_.add_runtime_error_callback(
      std::bind(&MetavisionWrapper::runtimeErrorCallback, this, ph::_1));
    runtimeErrorCallbackActive_ = true;
    contrastCallbackId_ = cam_.cd().add_callback(std::bind(
      useMultithreading_ ? &MetavisionWrapper::eventCallbackMultithreaded
                         : &MetavisionWrapper::eventCallback,
      this, ph::_1, ph::_2));
    contrastCallbackActive_ = true;
  } catch (const Metavision::CameraException & e) {
    LOG_ERROR_NAMED("unexpected sdk error: " << e.what());
    return (false);
  }
  return (true);
}

bool MetavisionWrapper::startCamera(CallbackHandler * h)
{
  try {
    callbackHandler_ = h;
    if (useMultithreading_) {
      thread_ = std::make_shared<std::thread>(&MetavisionWrapper::processingThread, this);
    }
    // this will actually start the camera
    cam_.start();
  } catch (const Metavision::CameraException & e) {
    LOG_ERROR_NAMED("unexpected sdk error: " << e.what());
    return (false);
  }
  return (true);
}

void MetavisionWrapper::runtimeErrorCallback(const Metavision::CameraException & e)
{
  LOG_ERROR_NAMED("camera runtime error occured: " << e.what());
}

void MetavisionWrapper::statusChangeCallback(const Metavision::CameraStatus & s)
{
  LOG_INFO_NAMED("camera " << (s == Metavision::CameraStatus::STARTED ? "started." : "stopped."));
}

bool MetavisionWrapper::saveBiases()
{
  if (biasFile_.empty()) {
    LOG_WARN_NAMED("no bias file specified at startup, no biases saved!");
    return (false);
  } else {
    try {
      cam_.biases().save_to_file(biasFile_);
      LOG_INFO_NAMED("biases written to file: " << biasFile_);
    } catch (const Metavision::CameraException & e) {
      LOG_WARN_NAMED("failed to write bias file: " << e.what());
      return (false);
    }
  }
  return (true);
}

void MetavisionWrapper::updateStatistics(const EventCD * start, const EventCD * end)
{
  const int64_t t_end = (end - 1)->t;
  const unsigned int num_events = end - start;
  const float dt = static_cast<float>(t_end - (lastEventTime_ < 0 ? t_end : lastEventTime_));
  const float dt_inv = dt != 0 ? (1.0 / dt) : 0;
  const float rate = num_events * dt_inv;
  maxRate_ = std::max(rate, maxRate_);
  totalEvents_ += num_events;
  totalTime_ += dt;
  lastEventTime_ = t_end;

  if (t_end > lastPrintTime_ + statisticsPrintInterval_) {
    const float avgRate = totalEvents_ * (totalTime_ > 0 ? 1.0 / totalTime_ : 0);
    const float avgSize = totalEventsSent_ * (totalMsgsSent_ != 0 ? 1.0 / totalMsgsSent_ : 0);
    const uint32_t totCount = eventCount_[1] + eventCount_[0];
    const int pctOn = (100 * eventCount_[1]) / (totCount == 0 ? 1 : totCount);
#ifndef USING_ROS_1
#ifdef CHECK_IF_OUTSIDE_ROI
    LOG_INFO_NAMED_FMT(
      "avg: %9.5f Mevs, max: %7.3f, out sz: %7.2f ev, %%on: %3d, qs: "
      "%4zu !roi: %8zu",
      avgRate, maxRate_, avgSize, pctOn, maxQueueSize_, outsideROI_);
#else
    LOG_INFO_NAMED_FMT(
      "avg: %9.5f Mevs, max: %7.3f, out sz: %7.2f ev, %%on: %3d, qs: "
      "%4zu",
      avgRate, maxRate_, avgSize, pctOn, maxQueueSize_);
#endif
#else
    LOG_INFO_NAMED_FMT(
      "%s: avg: %9.5f Mevs, max: %7.3f, out sz: %7.2f ev, %%on: %3d, qs: "
      "%4zu",
      loggerName_.c_str(), avgRate, maxRate_, avgSize, pctOn, maxQueueSize_);
#endif
    maxRate_ = 0;
    lastPrintTime_ = t_end;
    totalEvents_ = 0;
    totalTime_ = 0;
    totalMsgsSent_ = 0;
    totalEventsSent_ = 0;
    eventCount_[0] = 0;
    eventCount_[1] = 0;
    maxQueueSize_ = 0;
#ifdef CHECK_IF_OUTSIDE_ROI
    outsideROI_ = 0;
#endif
  }
}

void MetavisionWrapper::extTriggerCallback(
  const EventExtTrigger * start, const EventExtTrigger * end)
{
  const size_t n = end - start;
  if (n != 0) {
    callbackHandler_->triggerEventCallback(start, end);
  }
}

void MetavisionWrapper::extTriggerCallbackMultithreaded(
  const EventExtTrigger * start, const EventExtTrigger * end)
{
  // queue stuff away quickly to prevent events from being
  // dropped at the SDK level
  const size_t n = end - start;
  if (n != 0) {
    const size_t n_bytes = n * sizeof(EventCD);
    void * memblock = malloc(n_bytes);
    memcpy(memblock, start, n_bytes);
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_front(QueueElement(ExtTrigger, memblock, n));
    cv_.notify_all();
  }
}

void MetavisionWrapper::eventCallback(const EventCD * start, const EventCD * end)
{
  const size_t n = end - start;
  if (n != 0) {
    updateStatistics(start, end);
    callbackHandler_->cdEventCallback(start, end);
    if (callbackHandler2_) {
      callbackHandler2_->cdEventCallback(start, end);
    }
  }
}

void MetavisionWrapper::eventCallbackMultithreaded(const EventCD * start, const EventCD * end)
{
  // queue stuff away quickly to prevent events from being
  // dropped at the SDK level
  const size_t n = end - start;
  if (n != 0) {
    const size_t n_bytes = n * sizeof(EventCD);
    void * memblock = malloc(n_bytes);
    memcpy(memblock, start, n_bytes);
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_front(QueueElement(CD, memblock, n));
    cv_.notify_all();
  }
}

void MetavisionWrapper::processingThread()
{
  const std::chrono::microseconds timeout((int64_t)(1000000LL));
  while (callbackHandler_->keepRunning() && keepRunning_) {
    QueueElement qe;
    size_t qs = 0;
    {  // critical section, no processing done here
      std::unique_lock<std::mutex> lock(mutex_);
      while (callbackHandler_->keepRunning() && keepRunning_ && queue_.empty()) {
        cv_.wait_for(lock, timeout);
      }
      if (!queue_.empty()) {
        qs = queue_.size();
        qe = queue_.back();  // makes copy
        queue_.pop_back();
      }
    }
    if (qe.numEvents != 0) {
      maxQueueSize_ = std::max(maxQueueSize_, qs);
      switch (qe.eventType) {
        case CD: {
          const EventCD * start = static_cast<const EventCD *>(qe.start);
          const EventCD * end = start + qe.numEvents;
          updateStatistics(start, end);
          callbackHandler_->cdEventCallback(start, end);
          if (callbackHandler2_) {
            callbackHandler2_->cdEventCallback(start, end);
          }
          break;
        }
        case ExtTrigger: {
          const EventExtTrigger * start = static_cast<const EventExtTrigger *>(qe.start);
          const EventExtTrigger * end = start + qe.numEvents;
          callbackHandler_->triggerEventCallback(start, end);
          break;
        }
      }
      free(const_cast<void *>(qe.start));
    }
  }
  LOG_INFO_NAMED("processing thread exited!");
}
void MetavisionWrapper::setExternalTriggerOutMode(
  const std::string & mode, const int period, const double duty_cycle)
{
  triggerOutMode_ = mode;
  triggerOutPeriod_ = period;
  triggerOutDutyCycle_ = duty_cycle;
}

}  // namespace metavision_ros_driver
