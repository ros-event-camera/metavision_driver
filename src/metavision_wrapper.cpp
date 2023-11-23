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

#include "metavision_driver/metavision_wrapper.h"

#ifdef USING_METAVISION_3
#include <metavision/hal/facilities/i_device_control.h>
#include <metavision/hal/facilities/i_erc.h>
#else
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_erc_module.h>
#endif

#include <metavision/hal/facilities/i_hw_identification.h>
#include <metavision/hal/facilities/i_hw_register.h>
#include <metavision/hal/facilities/i_plugin_software_info.h>
#include <metavision/hal/facilities/i_trigger_in.h>

#include <chrono>
#include <map>
#include <set>
#include <thread>

#ifdef USING_ROS_1
#define GENERIC_ROS_OK() (ros::ok())
#else
#define GENERIC_ROS_OK() (rclcpp::ok())
#endif

#include "metavision_driver/logging.h"

namespace metavision_driver
{
#ifdef USING_METAVISION_3
using CameraSynchronization = Metavision::I_DeviceControl;
using Window = Metavision::Roi::Rectangle;
using ErcModule = Metavision::I_Erc;
#else
using CameraSynchronization = Metavision::I_CameraSynchronization;
using Window = Metavision::Roi::Window;
using ErcModule = Metavision::I_ErcModule;
static const std::map<std::string, Metavision::I_TriggerIn::Channel> channelMap = {
  {"external", Metavision::I_TriggerIn::Channel::Main},
  {"aux", Metavision::I_TriggerIn::Channel::Aux},
  {"loopback", Metavision::I_TriggerIn::Channel::Loopback}};
#endif

// Map sensor name to mipi frame period register. This should really be done by the SDK...

static const std::map<std::string, uint32_t> sensorToMIPIAddress = {
  {"IMX636", 0xB028}, {"Gen3.1", 0x1508}};

static std::string to_lower(const std::string upper)
{
  std::string lower(upper);
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  return (lower);
}

MetavisionWrapper::MetavisionWrapper(const std::string & loggerName)
{
  setLoggerName(loggerName);
  lastPrintTime_ = std::chrono::system_clock::now();
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

bool MetavisionWrapper::hasBias(const std::string & name)
{
  Metavision::Biases & biases = cam_.biases();
  Metavision::I_LL_Biases * hw_biases = biases.get_facility();
  const auto pmap = hw_biases->get_all_biases();
  auto it = pmap.find(name);
  return (it != pmap.end());
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

bool MetavisionWrapper::initialize(bool useMultithreading, const std::string & biasFile)
{
  biasFile_ = biasFile;
  useMultithreading_ = useMultithreading;

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
  if (rawDataCallbackActive_) {
    cam_.raw_data().remove_callback(rawDataCallbackId_);
  }
  if (statusChangeCallbackActive_) {
    cam_.remove_status_change_callback(statusChangeCallbackId_);
  }
  if (contrastCallbackActive_) {
    cam_.cd().remove_callback(contrastCallbackId_);
  }
  if (extTriggerCallbackActive_) {
    cam_.ext_trigger().remove_callback(extTriggerCallbackId_);
  }

  keepRunning_ = false;
  if (processingThread_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.notify_all();
    }
    processingThread_->join();
    processingThread_.reset();
  }
  if (statsThread_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.notify_all();
    }
    statsThread_->join();
    statsThread_.reset();
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
      std::vector<Window> rects;
      for (size_t i = 0; i < roi.size(); i += 4) {
        decltype(rects)::value_type rect;
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
  auto * sync = cam_.get_device().get_facility<CameraSynchronization>();
  if (!sync) {  // happens when playing from file
    if (mode != "standalone") {
      LOG_WARN_NAMED("cannot set sync mode to: " << mode);
    }
    return;
  }

  if (mode == "standalone") {
    if (sync->get_mode() != CameraSynchronization::SyncMode::STANDALONE) {
      sync->set_mode_standalone();
    }
  } else if (mode == "primary") {
    sync->set_mode_master();
  } else if (mode == "secondary") {
    sync->set_mode_slave();
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

  if (mode_in != "disabled") {
    Metavision::I_TriggerIn * i_trigger_in =
      cam_.get_device().get_facility<Metavision::I_TriggerIn>();
    if (i_trigger_in) {
#ifdef USING_METAVISION_3
      auto it = hardwarePinConfig_[softwareInfo_].find(mode_in);
      if (it == hardwarePinConfig_[softwareInfo_].end()) {
        LOG_ERROR_NAMED("no pin defined for trigger in mode " << mode_in);
      } else {
        i_trigger_in->enable(it->second);
        LOG_INFO_NAMED("Enabled trigger input " << mode_in << " on " << it->second);
      }
#else
      auto channel = channelMap.find(mode_in);
      if (channel == channelMap.end()) {
        LOG_ERROR_NAMED("invalid trigger mode: " << mode_in);
      } else {
        i_trigger_in->enable(channel->second);
        LOG_INFO_NAMED("Enabled trigger input " << mode_in);
      }
#endif
    } else {
      LOG_ERROR_NAMED("Failed enabling trigger input");
    }
  }
}

void MetavisionWrapper::configureEventRateController(
  const std::string & mode, const int events_per_sec)
{
  if (mode == "enabled" || mode == "disabled") {
    auto * i_erc = cam_.get_device().get_facility<ErcModule>();
    if (i_erc) {
      i_erc->enable(mode == "enabled");
      i_erc->set_cd_event_rate(events_per_sec);
    } else {
      LOG_WARN_NAMED("cannot set event rate control for this camera!");
    }
  }
}

bool MetavisionWrapper::initializeCamera()
{
  const int num_tries = 5;
  for (int i = 0; i < num_tries; i++) {
    try {
      if (!fromFile_.empty()) {
        LOG_INFO_NAMED("reading events from file: " << fromFile_);
        const auto cfg = Metavision::FileConfigHints().real_time_playback(true);
        cam_ = Metavision::Camera::from_file(fromFile_, cfg);
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
      if (i < num_tries - 1) {
        LOG_WARN_NAMED(
          "cannot open " << src << " on attempt " << i + 1 << ", retrying " << num_tries - i - 1
                         << " more times");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else {
        LOG_ERROR_NAMED("cannot open " << src << ", giving up!");
      }
    }
  }

  try {
    // Record the plugin software information about the camera.
    using PSI = Metavision::I_PluginSoftwareInfo;
    const PSI * psi = cam_.get_device().get_facility<PSI>();
    softwareInfo_ = psi->get_plugin_name();
    LOG_INFO_NAMED("plugin software name: " << softwareInfo_);
    using HWI = Metavision::I_HW_Identification;
    const HWI * hwi = cam_.get_device().get_facility<HWI>();
    const auto sinfo = hwi->get_sensor_info();
    encodingFormat_ = to_lower(hwi->get_current_data_encoding_format());
    LOG_INFO_NAMED("encoding format: " << encodingFormat_);
    sensorVersion_ =
      std::to_string(sinfo.major_version_) + "." + std::to_string(sinfo.minor_version_);
    LOG_INFO_NAMED("sensor version: " << sensorVersion_);
    LOG_INFO_NAMED("sensor name: " << sinfo.name_);
    if (!biasFile_.empty()) {
      try {
        cam_.biases().set_from_file(biasFile_);
        LOG_INFO_NAMED("using bias file: " << biasFile_);
      } catch (const Metavision::CameraException & e) {
        LOG_WARN_NAMED("reading bias file failed with error: " << e.what());
        LOG_WARN_NAMED("continuing with default biases!");
      }
    } else if (fromFile_.empty()) {  // only load biases when not playing from file!
      LOG_INFO_NAMED("no bias file provided, using camera defaults:");
      const Metavision::Biases biases = cam_.biases();
      Metavision::I_LL_Biases * hw_biases = biases.get_facility();
      const auto pmap = hw_biases->get_all_biases();
      for (const auto & bp : pmap) {
        LOG_INFO_NAMED("found bias param: " << bp.first << " " << bp.second);
      }
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
      if (mipiFramePeriod_ > 0) {
        configureMIPIFramePeriod(mipiFramePeriod_, sinfo.name_);
      }
    }
    statusChangeCallbackId_ = cam_.add_status_change_callback(
      std::bind(&MetavisionWrapper::statusChangeCallback, this, ph::_1));
    statusChangeCallbackActive_ = true;
    runtimeErrorCallbackId_ = cam_.add_runtime_error_callback(
      std::bind(&MetavisionWrapper::runtimeErrorCallback, this, ph::_1));
    runtimeErrorCallbackActive_ = true;
    rawDataCallbackId_ = cam_.raw_data().add_callback(std::bind(
      useMultithreading_ ? &MetavisionWrapper::rawDataCallbackMultithreaded
                         : &MetavisionWrapper::rawDataCallback,
      this, ph::_1, ph::_2));
    rawDataCallbackActive_ = true;
  } catch (const Metavision::CameraException & e) {
    LOG_ERROR_NAMED("unexpected sdk error: " << e.what());
    return (false);
  }
  return (true);
}

void MetavisionWrapper::configureMIPIFramePeriod(int usec, const std::string & sensorName)
{
  const auto it = sensorToMIPIAddress.find(sensorName);
  if (it == sensorToMIPIAddress.end()) {
    LOG_WARN_NAMED("cannot configure mipi frame period for sensor " << sensorName);
  } else {
    const uint32_t mfpa = it->second;
    auto hwrf = cam_.get_device().get_facility<Metavision::I_HW_Register>();
    const int prev_mfp = hwrf->read_register(mfpa);
    hwrf->write_register(mfpa, usec);
    const int new_mfp = hwrf->read_register(mfpa);
    LOG_INFO_NAMED("mipi frame period changed from " << prev_mfp << " to " << new_mfp << "us");
  }
}

void MetavisionWrapper::setDecodingEvents(bool decodeEvents)
{
  if (decodeEvents && !contrastCallbackActive_) {
    contrastCallbackId_ =
      cam_.cd().add_callback(std::bind(&MetavisionWrapper::cdCallback, this, ph::_1, ph::_2));
    contrastCallbackActive_ = true;
  }
  if (decodeEvents && !extTriggerCallbackActive_) {
    extTriggerCallbackId_ = cam_.ext_trigger().add_callback(
      std::bind(&MetavisionWrapper::extTriggerCallback, this, ph::_1, ph::_2));
    extTriggerCallbackActive_ = true;
  }

  if (!decodeEvents && contrastCallbackActive_) {
    cam_.cd().remove_callback(contrastCallbackId_);
    contrastCallbackActive_ = false;
  }
  if (!decodeEvents && extTriggerCallbackActive_) {
    cam_.ext_trigger().remove_callback(extTriggerCallbackId_);
    extTriggerCallbackActive_ = false;
  }
}

bool MetavisionWrapper::startCamera(CallbackHandler * h)
{
  try {
    callbackHandler_ = h;
    if (useMultithreading_) {
      processingThread_ = std::make_shared<std::thread>(&MetavisionWrapper::processingThread, this);
    }
    statsThread_ = std::make_shared<std::thread>(&MetavisionWrapper::statsThread, this);
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

void MetavisionWrapper::rawDataCallback(const uint8_t * data, size_t size)
{
  if (size != 0) {
    const uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    callbackHandler_->rawDataCallback(t, data, data + size);
    {
      std::unique_lock<std::mutex> lock(statsMutex_);
      stats_.msgsRecv++;
      stats_.bytesRecv += size;
    }
  }
}

void MetavisionWrapper::rawDataCallbackMultithreaded(const uint8_t * data, size_t size)
{
  // queue stuff away quickly to prevent events from being
  // dropped at the SDK level
  if (size != 0) {
    const uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    {
      void * memblock = malloc(size);
      memcpy(memblock, data, size);
      std::unique_lock<std::mutex> lock(mutex_);
      queue_.push_front(QueueElement(memblock, size, t));
      cv_.notify_all();
    }
    {
      std::unique_lock<std::mutex> lock(statsMutex_);
      stats_.msgsRecv++;
      stats_.bytesRecv += size;
    }
  }
}

void MetavisionWrapper::cdCallback(
  const Metavision::EventCD * start, const Metavision::EventCD * end)
{
  // this code only used during startup for synchronization,
  // not during regular operation
  const uint64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  callbackHandler_->eventCDCallback(t, start, end);
}

void MetavisionWrapper::extTriggerCallback(
  const Metavision::EventExtTrigger *, const Metavision::EventExtTrigger *)
{
  // do nothing for now
}

void MetavisionWrapper::processingThread()
{
  const std::chrono::microseconds timeout((int64_t)(1000000LL));
  while (GENERIC_ROS_OK() && keepRunning_) {
    QueueElement qe;
    size_t qs = 0;
    {  // critical section, no processing done here
      std::unique_lock<std::mutex> lock(mutex_);
      while (GENERIC_ROS_OK() && keepRunning_ && queue_.empty()) {
        cv_.wait_for(lock, timeout);
      }
      if (!queue_.empty()) {
        qs = queue_.size();
        qe = queue_.back();  // makes copy of element, not the data
        queue_.pop_back();
      }
    }
    if (qe.numBytes != 0) {
      const uint8_t * data = static_cast<const uint8_t *>(qe.start);
      callbackHandler_->rawDataCallback(qe.timeStamp, data, data + qe.numBytes);
      free(const_cast<void *>(qe.start));
      {
        std::unique_lock<std::mutex> lock(statsMutex_);
        stats_.maxQueueSize = std::max(stats_.maxQueueSize, qs);
      }
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

void MetavisionWrapper::statsThread()
{
  while (GENERIC_ROS_OK() && keepRunning_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(statsInterval_ * 1000)));
    printStatistics();
  }
  LOG_INFO_NAMED("statistics thread exited!");
}

void MetavisionWrapper::printStatistics()
{
  Stats stats;
  {
    std::unique_lock<std::mutex> lock(statsMutex_);
    stats = stats_;
    stats_ = Stats();  // reset statistics
  }
  std::chrono::time_point<std::chrono::system_clock> t_now = std::chrono::system_clock::now();
  const double dt = std::chrono::duration<double>(t_now - lastPrintTime_).count();
  lastPrintTime_ = t_now;
  const double invT = dt > 0 ? 1.0 / dt : 0;
  const double recvByteRate = 1e-6 * stats.bytesRecv * invT;

  const int recvMsgRate = static_cast<int>(stats.msgsRecv * invT);
  const int sendMsgRate = static_cast<int>(stats.msgsSent * invT);

#ifndef USING_ROS_1
  if (useMultithreading_) {
    LOG_INFO_NAMED_FMT(
      "bw in: %9.5f MB/s, msgs/s in: %7d, "
      "out: %7d, maxq: %4zu",
      recvByteRate, recvMsgRate, sendMsgRate, stats.maxQueueSize);
  } else {
    LOG_INFO_NAMED_FMT(
      "bw in: %9.5f MB/s, msgs/s in: %7d, "
      "out: %7d",
      recvByteRate, recvMsgRate, sendMsgRate);
  }
#else
  if (useMultithreading_) {
    LOG_INFO_NAMED_FMT(
      "%s: bw in: %9.5f MB/s, msgs/s in: %7d, out: %7d, maxq: %4zu", loggerName_.c_str(),
      recvByteRate, recvMsgRate, sendMsgRate, stats.maxQueueSize);
  } else {
    LOG_INFO_NAMED_FMT(
      "%s: bw in: %9.5f MB/s, msgs/s in: %7d, out: %7d", loggerName_.c_str(), recvByteRate,
      recvMsgRate, sendMsgRate);
  }
#endif
}

}  // namespace metavision_driver
