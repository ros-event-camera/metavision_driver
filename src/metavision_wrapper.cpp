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

#include "metavision_ros_driver/metavision_wrapper.h"

#include <metavision/hal/facilities/i_device_control.h>

#include <set>

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
    LOG_NAMED_ERROR("unknown bias parameter: " << name);
    throw(std::runtime_error("bias parameter not found!"));
  }
  return (it->second);
}

int MetavisionWrapper::setBias(const std::string & name, int val)
{
  std::set<std::string> dont_touch_set = {{"bias_diff"}};
  if (dont_touch_set.count(name) != 0) {
    LOG_NAMED_WARN("ignoring change to parameter: " << name);
    return (val);
  }
  Metavision::Biases & biases = cam_.biases();
  Metavision::I_LL_Biases * hw_biases = biases.get_facility();
  const int prev = hw_biases->get(name);
  if (val != prev) {
    if (!hw_biases->set(name, val)) {
      LOG_NAMED_WARN("cannot set parameter" << name << " to " << val);
    }
  }
  const int now = hw_biases->get(name);  // read back what actually took hold
  LOG_NAMED_INFO("changed  " << name << " from " << prev << " to " << val << " adj to: " << now);
  return (now);
}

bool MetavisionWrapper::initialize(
  bool useMultithreading, double statItv, const std::string & biasFile)
{
  biasFile_ = biasFile;
  useMultithreading_ = useMultithreading;

  statisticsPrintInterval_ = static_cast<int>(statItv * 1e6);
  if (!initializeCamera()) {
    LOG_NAMED_ERROR("could not initialize camera!");
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
      LOG_NAMED_ERROR("ROI vec must be multiple of 4, but is: " << roi.size());
    } else {
      std::vector<Metavision::Roi::Rectangle> rects;
      for (size_t i = 0; i < roi.size(); i += 4) {
        Metavision::Roi::Rectangle rect;
        rect.x = roi[i];
        rect.y = roi[i + 1];
        rect.width = roi[i + 2];
        rect.height = roi[i + 3];
        rects.push_back(rect);
      }
      cam_.roi().set(rects);
    }
  } else {
    cam_.roi().unset();
  }
}

void MetavisionWrapper::applySyncMode(const std::string & mode)
{
  Metavision::I_DeviceControl * control =
    cam_.get_device().get_facility<Metavision::I_DeviceControl>();
  if (mode == "standalone") {
    control->set_mode_standalone();
  } else if (mode == "primary") {
    control->set_mode_master();
  } else if (mode == "secondary") {
    control->set_mode_slave();
  } else {
    LOG_NAMED_ERROR("INVALID SYNC MODE: " << mode);
    throw std::runtime_error("invalid sync mode!");
  }
}

bool MetavisionWrapper::initializeCamera()
{
  try {
    if (!serialNumber_.empty()) {
      cam_ = Metavision::Camera::from_serial(serialNumber_);
    } else {
      cam_ = Metavision::Camera::from_first_available();
    }
    if (!biasFile_.empty()) {
      try {
        cam_.biases().set_from_file(biasFile_);
        LOG_NAMED_INFO("using bias file: " << biasFile_);
      } catch (const Metavision::CameraException & e) {
        LOG_NAMED_WARN("reading bias file failed with error: " << e.what());
        LOG_NAMED_WARN("continuing with default biases!");
      }
    } else {
      LOG_NAMED_INFO("no bias file provided, using camera defaults");
    }
    // overwrite serial in case it was not set
    serialNumber_ = cam_.get_camera_configuration().serial_number;
    LOG_NAMED_INFO("camera serial number: " << serialNumber_);
    const auto & g = cam_.geometry();
    width_ = g.width();
    height_ = g.height();
    LOG_NAMED_INFO("sensor geometry: " << width_ << " x " << height_);
    applySyncMode(syncMode_);
    applyROI(roi_);
    statusChangeCallbackId_ = cam_.add_status_change_callback(
      std::bind(&MetavisionWrapper::statusChangeCallback, this, ph::_1));
    statusChangeCallbackActive_ = true;
    runtimeErrorCallbackId_ = cam_.add_runtime_error_callback(
      std::bind(&MetavisionWrapper::runtimeErrorCallback, this, ph::_1));
    runtimeErrorCallbackActive_ = true;
    if (useMultithreading_) {
      contrastCallbackId_ = cam_.cd().add_callback(
        std::bind(&MetavisionWrapper::eventCallbackMultithreaded, this, ph::_1, ph::_2));
    } else {
      contrastCallbackId_ =
        cam_.cd().add_callback(std::bind(&MetavisionWrapper::eventCallback, this, ph::_1, ph::_2));
    }
    contrastCallbackActive_ = true;
  } catch (const Metavision::CameraException & e) {
    LOG_NAMED_ERROR("unexpected sdk error: " << e.what());
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
    LOG_NAMED_ERROR("unexpected sdk error: " << e.what());
    return (false);
  }
  return (true);
}

void MetavisionWrapper::runtimeErrorCallback(const Metavision::CameraException & e)
{
  LOG_NAMED_ERROR("camera runtime error occured: " << e.what());
}

void MetavisionWrapper::statusChangeCallback(const Metavision::CameraStatus & s)
{
  LOG_NAMED_INFO("camera " << (s == Metavision::CameraStatus::STARTED ? "started." : "stopped."));
}

bool MetavisionWrapper::saveBiases()
{
  if (biasFile_.empty()) {
    LOG_NAMED_WARN("no bias file specified at startup, no biases saved!");
    return (false);
  } else {
    try {
      cam_.biases().save_to_file(biasFile_);
      LOG_NAMED_INFO("biases written to file: " << biasFile_);
    } catch (const Metavision::CameraException & e) {
      LOG_NAMED_WARN("failed to write bias file: " << e.what());
      return (false);
    }
  }
  return (true);
}

void MetavisionWrapper::updateStatistics(const EventCD * start, const EventCD * end)
{
  const int64_t t_end = (end - 1)->t;
  const unsigned int num_events = end - start;
  const float dt = static_cast<float>(t_end - start->t);
  const float dt_inv = dt != 0 ? (1.0 / dt) : 0;
  const float rate = num_events * dt_inv;
  maxRate_ = std::max(rate, maxRate_);
  totalEvents_ += num_events;
  totalTime_ += dt;

  if (t_end > lastPrintTime_ + statisticsPrintInterval_) {
    const float avgRate = totalEvents_ * (totalTime_ > 0 ? 1.0 / totalTime_ : 0);
    const float avgSize = totalEventsSent_ * (totalMsgsSent_ != 0 ? 1.0 / totalMsgsSent_ : 0);
    const uint32_t totCount = eventCount_[1] + eventCount_[0];
    const int pctOn = (100 * eventCount_[1]) / (totCount == 0 ? 1 : totCount);
#ifndef USING_ROS_1
    LOG_NAMED_INFO_FMT(
      "rate[Mevs] avg: %7.3f, max: %7.3f, out sz: %7.2f ev, %%on: %3d, qs: "
      "%4zu",
      avgRate, maxRate_, avgSize, pctOn, maxQueueSize_);
#else
    LOG_NAMED_INFO_FMT(
      "%s: rate[Mevs] avg: %7.3f, max: %7.3f, out sz: %7.2f ev, %%on: %3d, qs: "
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
  }
}

void MetavisionWrapper::eventCallback(const EventCD * start, const EventCD * end)
{
  const size_t n = end - start;
  if (n != 0) {
    updateStatistics(start, end);
    callbackHandler_->publish(start, end);
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
    queue_.push_front(std::pair<size_t, void *>(n, memblock));
    cv_.notify_all();
  }
}

void MetavisionWrapper::processingThread()
{
  const std::chrono::microseconds timeout((int64_t)(1000000LL));
  while (callbackHandler_->keepRunning() && keepRunning_) {
    QueueElement qe(0, 0);
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
    if (qe.first != 0) {
      const EventCD * start = static_cast<const EventCD *>(qe.second);
      const EventCD * end = start + qe.first;
      maxQueueSize_ = std::max(maxQueueSize_, qs);
      updateStatistics(start, end);
      callbackHandler_->publish(start, end);
      free(const_cast<void *>(qe.second));
    }
  }
  LOG_NAMED_INFO("processing thread exited!");
}
}  // namespace metavision_ros_driver
