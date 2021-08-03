// -*-c++-*--------------------------------------------------------------------
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

#ifndef METAVISION_ROS_DRIVER_H_
#define METAVISION_ROS_DRIVER_H_

#include <dvs_msgs/EventArray.h>
#include <dynamic_reconfigure/server.h>
#include <metavision/sdk/driver/camera.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>

#include "metavision_ros_driver/MetaVisionDynConfig.h"

namespace metavision_ros_driver
{
namespace ph = std::placeholders;
template <class MsgType>

class Driver
{
public:
  using EventCD = Metavision::EventCD;
  using Config = MetaVisionDynConfig;
  typedef std::pair<size_t, const void *> QueueElement;

  Driver(const ros::NodeHandle & nh) : nh_(nh)
  {
    eventCount_[0] = 0;
    eventCount_[1] = 0;
  }
  ~Driver() { shutdown(); }

  static void set_from_map(
    const std::map<std::string, int> & pmap, int * targ,
    const std::string & name)
  {
    auto it = pmap.find(name);
    if (it != pmap.end()) {
      *targ = it->second;
      ROS_INFO("setting camera parameter %-15s to %4d", name.c_str(), *targ);
    }
  }

  void checkAndSet(int prev, int * current, const std::string & name)
  {
    std::set<std::string> dont_touch_set = {{"bias_diff"}};
    if (dont_touch_set.count(name) != 0 && prev != *current) {
      ROS_WARN_STREAM("ignoring change to parameter: " << name);
      *current = prev;  // reset it to old level
    } else if (*current != prev) {
      Metavision::Biases & biases = cam_.biases();
      Metavision::I_LL_Biases * hw_biases = biases.get_facility();
      hw_biases->set(name, *current);
      int now = hw_biases->get(name);  // read back what actually took hold
      ROS_INFO_STREAM(
        "changed param: " << name << " from " << prev << " to " << *current
                          << " adj to: " << now);
      *current = now;
    }
  }

  void configure(Config & config, int level)
  {
    if (level < 0) {  // initial call
      Metavision::Biases & biases = cam_.biases();
      Metavision::I_LL_Biases * hw_biases = biases.get_facility();
      auto bias_map = hw_biases->get_all_biases();
      // initialize config from current settings
      set_from_map(bias_map, &config.bias_diff, "bias_diff");
      set_from_map(bias_map, &config.bias_diff_off, "bias_diff_off");
      set_from_map(bias_map, &config.bias_diff_on, "bias_diff_on");
      set_from_map(bias_map, &config.bias_fo, "bias_fo");
      set_from_map(bias_map, &config.bias_hpf, "bias_hpf");
      set_from_map(bias_map, &config.bias_pr, "bias_pr");
      set_from_map(bias_map, &config.bias_refr, "bias_refr");
      ROS_INFO("initialized config to camera biases");
    } else {
      checkAndSet(config_.bias_diff, &config.bias_diff, "bias_diff");
      checkAndSet(
        config_.bias_diff_off, &config.bias_diff_off, "bias_diff_off");
      checkAndSet(config_.bias_diff_on, &config.bias_diff_on, "bias_diff_on");
      checkAndSet(config_.bias_fo, &config.bias_fo, "bias_fo");
      checkAndSet(config_.bias_hpf, &config.bias_hpf, "bias_hpf");
      checkAndSet(config_.bias_pr, &config.bias_pr, "bias_pr");
      checkAndSet(config_.bias_refr, &config.bias_refr, "bias_refr");
    }
    config_ = config;  // remember current values
  }

  bool initialize()
  {
    useMultithreading_ = nh_.param<bool>("use_multithreading", false);
    if (useMultithreading_) {
      thread_ = std::make_shared<std::thread>(&Driver::processingThread, this);
    }
    const double statItv = nh_.param<double>("statistics_print_interval", 1.0);
    statisticsPrintInterval_ = (int)(statItv * 1e6);
    messageTimeThreshold_ =
      ros::Duration(nh_.param<double>("message_time_threshold", 1e-9));
    int qs = nh_.param<int>("send_queue_size", 1000);
    eventPublisher_ = nh_.advertise<MsgType>("events", qs);
    if (!startCamera()) {
      ROS_ERROR_STREAM("could not start camera!");
      return (false);
    }
    // hook up dynamic config server *after* the camera has
    // been initialized so we can read the bias values
    configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
    configServer_->setCallback(boost::bind(&Driver::configure, this, _1, _2));

    saveService_ =
      nh_.advertiseService("save_biases", &Driver::saveBiases, this);

    ROS_INFO_STREAM("driver initialized successfully.");
    return (true);
  }

private:
  void shutdown()
  {
    if (cam_.is_running()) {
      ROS_INFO_STREAM("driver exiting, shutting down camera");
      cam_.stop();
    } else {
      ROS_INFO_STREAM("driver exiting, camera already stopped.");
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
  }

  bool startCamera()
  {
    try {
      cam_ = Metavision::Camera::from_first_available();
      biasFile_ = nh_.param<std::string>("bias_file", "");
      if (!biasFile_.empty()) {
        cam_.biases().set_from_file(biasFile_);
        ROS_INFO_STREAM("biases loaded from file: " << biasFile_);
      } else {
        ROS_WARN("no bias file provided, starting with default biases");
      }
      std::string sn = cam_.get_camera_configuration().serial_number;
      ROS_INFO_STREAM("camera serial number: " << sn);
      const auto & g = cam_.geometry();

      // default frame id to last 4 digits of serial number
      auto tail = sn.substr(sn.size() - 4);
      frameId_ = nh_.param<std::string>("frame_id", tail);
      width_ = g.width();
      height_ = g.height();
      ROS_INFO_STREAM(
        "frame_id: " << frameId_ << ", size: " << width_ << " x " << height_);

      statusChangeCallbackId_ = cam_.add_status_change_callback(
        std::bind(&Driver::statusChangeCallback, this, ph::_1));
      statusChangeCallbackActive_ = true;
      runtimeErrorCallbackId_ = cam_.add_runtime_error_callback(
        std::bind(&Driver::runtimeErrorCallback, this, ph::_1));
      runtimeErrorCallbackActive_ = true;
      if (useMultithreading_) {
        contrastCallbackId_ = cam_.cd().add_callback(
          std::bind(&Driver::eventCallbackMultithreaded, this, ph::_1, ph::_2));
      } else {
        contrastCallbackId_ = cam_.cd().add_callback(
          std::bind(&Driver::eventCallback, this, ph::_1, ph::_2));
      }
      contrastCallbackActive_ = true;
      // this will actually start the camera
      cam_.start();
      // remember first ROS timestamp and hope that this is close
      // to the same timestamp that the metavision SDK is using
      // for t = 0

      t0_ = ros::Time::now().toNSec();
    } catch (const Metavision::CameraException & e) {
      ROS_WARN_STREAM("sdk error: " << e.what());
      return (false);
    }
    return (true);
  }

  void runtimeErrorCallback(const Metavision::CameraException & e)
  {
    ROS_WARN_STREAM("camera runtime error occured: " << e.what());
  }

  void statusChangeCallback(const Metavision::CameraStatus & s)
  {
    ROS_INFO_STREAM(
      "camera "
      << (s == Metavision::CameraStatus::STARTED ? "started." : "stopped."));
  }

  bool saveBiases(
    std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
  {
    (void)req;
    if (biasFile_.empty()) {
      ROS_WARN_STREAM("no bias file name specified!");
      res.message = "no bias file name specified!";
      res.success = false;
    }
    try {
      /* const auto & biasMap = cam_.biases().get_facility()->get_all_biases();
         for (const auto kv : biasMap) {
         std::cout << kv.first << " -> " << kv.second << std::endl;
         }
      */
      cam_.biases().save_to_file(biasFile_);
      res.message = "bias file saved to " + biasFile_;
      res.success = true;
      ROS_INFO_STREAM("wrote bias file to: " << biasFile_);
    } catch (const Metavision::CameraException & e) {
      ROS_WARN_STREAM("failed to write bias file: " << e.what());
      res.message = "failed to write bias file to: " + biasFile_;
      res.success = false;
    }

    return (true);
  }

  void updateStatistics(const EventCD * start, const EventCD * end)
  {
    const long t_end = (end - 1)->t;
    const unsigned int num_events = end - start;
    const float dt = (float)(t_end - start->t);
    const float dt_inv = dt != 0 ? (1.0 / dt) : 0;
    const float rate = num_events * dt_inv;
    maxRate_ = std::max(rate, maxRate_);
    totalEvents_ += num_events;
    totalTime_ += dt;

    if (t_end > lastPrintTime_ + statisticsPrintInterval_) {
      const float avgRate =
        totalEvents_ * (totalTime_ > 0 ? 1.0 / totalTime_ : 0);
      const float avgSize =
        totalEventsSent_ * (totalMsgsSent_ != 0 ? 1.0 / totalMsgsSent_ : 0);
      const uint32_t totCount = eventCount_[1] + eventCount_[0];
      const int pctOn = (100 * eventCount_[1]) / (totCount == 0 ? 1 : totCount);
      ROS_INFO(
        "rate[Mevs] avg: %7.3f, max: %7.3f, out sz: %7.2f ev, %%on: %3d, qs: "
        "%4zu",
        avgRate, maxRate_, avgSize, pctOn, maxQueueSize_);
      maxRate_ = 0;
      lastPrintTime_ += statisticsPrintInterval_;
      totalEvents_ = 0;
      totalTime_ = 0;
      totalMsgsSent_ = 0;
      totalEventsSent_ = 0;
      eventCount_[0] = 0;
      eventCount_[1] = 0;
      maxQueueSize_ = 0;
    }
  }

  void eventCallback(const EventCD * start, const EventCD * end)
  {
    const size_t n = end - start;
    if (n != 0) {
      updateStatistics(start, end);
      if (eventPublisher_.getNumSubscribers() > 0) {
        updateAndPublish(start, end);
      }
    }
  }

  void eventCallbackMultithreaded(const EventCD * start, const EventCD * end)
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

  void processingThread()
  {
    const std::chrono::microseconds timeout((int64_t)(1000000LL));
    while (ros::ok() && keepRunning_) {
      QueueElement qe(0, 0);
      size_t qs = 0;
      {  // critical section, no processing done here
        std::unique_lock<std::mutex> lock(mutex_);
        while (ros::ok() && keepRunning_ && queue_.empty()) {
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
        if (eventPublisher_.getNumSubscribers() > 0) {
          updateAndPublish(start, end);
        }
      }
    }
    ROS_INFO("processing thread exited!");
  }

  void updateAndPublish(const EventCD * start, const EventCD * end)
  {
    if (!msg_) {  // must allocate new message
      msg_.reset(new MsgType());
      msg_->header.frame_id = frameId_;
      msg_->header.seq = seq_++;
      msg_->width = width_;
      msg_->height = height_;
      // under full load a 50 Mev/s camera will
      // produce about 5000 events in a 100us
      // time slice.
      msg_->events.reserve(6000);
    }
    const size_t n = end - start;
    auto & events = msg_->events;
    const size_t old_size = events.size();
    // The resize should not trigger a
    // copy with proper reserved capacity.
    events.resize(events.size() + n);
    // copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is 320
    for (unsigned int i = 0; i < n; i++) {
      const auto & e_src = start[i];
      auto & e_trg = events[i + old_size];
      e_trg.x = e_src.x;
      e_trg.y = e_src.y;
      e_trg.polarity = e_src.p;
      e_trg.ts.fromNSec(t0_ + e_src.t * 1e3);
      eventCount_[e_src.p]++;
    }
    const ros::Time & t_msg = msg_->events.begin()->ts;
    const ros::Time & t_last = msg_->events.rbegin()->ts;
    if (t_last > t_msg + messageTimeThreshold_) {
      msg_->header.stamp = t_msg;
      eventPublisher_.publish(msg_);
      msg_.reset();  // no longer using this one
      totalEventsSent_ += events.size();
      totalMsgsSent_++;
    }
  }

  // ------------ variables
  ros::NodeHandle nh_;
  ros::Publisher eventPublisher_;
  Metavision::Camera cam_;
  Metavision::CallbackId statusChangeCallbackId_;
  bool statusChangeCallbackActive_{false};
  Metavision::CallbackId runtimeErrorCallbackId_;
  bool runtimeErrorCallbackActive_{false};
  Metavision::CallbackId contrastCallbackId_;
  bool contrastCallbackActive_{false};
  uint64_t t0_;  // base for time stamp calc
  // time span that will trigger a message to be send
  ros::Duration messageTimeThreshold_;
  std::string frameId_;  // ROS frame id
  int width_;            // image width
  int height_;           // image height
  uint32_t seq_;         // ROS sequence number
  boost::shared_ptr<MsgType> msg_;
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
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
  Config config_;
  std::string biasFile_;
  ros::ServiceServer saveService_;
  // related to multi threading
  bool useMultithreading_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::pair<size_t, const void *>> queue_;
  std::shared_ptr<std::thread> thread_;
  bool keepRunning_{true};
};
}  // namespace metavision_ros_driver
#endif
