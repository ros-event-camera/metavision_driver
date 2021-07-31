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
#include <metavision/sdk/driver/camera.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

#include <memory>
#include <string>

namespace metavision_ros_driver
{
namespace ph = std::placeholders;
template <class MsgType>

class Driver
{
public:
  using EventCD = Metavision::EventCD;

  Driver(const ros::NodeHandle & nh) : nh_(nh) {}
  ~Driver() { shutdown(); }

  bool initialize()
  {
    double print_interval;
    nh_.param<double>("statistics_print_interval", print_interval, 1.0);
    statisticsPrintInterval_ = (int)(print_interval * 1e6);
    messageTimeThreshold_ =
      ros::Duration(nh_.param<double>("message_time_threshold", 1e-9));
    int qs = nh_.param<int>("send_queue_size", 1000);
    eventPublisher_ = nh_.advertise<MsgType>("events", qs);
    if (!startCamera()) {
      ROS_ERROR_STREAM("could not start camera!");
      return (false);
    }
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
  }

  bool startCamera()
  {
    try {
      cam_ = Metavision::Camera::from_first_available();
      std::string biasFile = nh_.param<std::string>("bias_file", "");
      if (!biasFile.empty()) {
        cam_.biases().set_from_file(biasFile);
        ROS_INFO_STREAM("biases loaded from file: " << biasFile);
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
      contrastCallbackId_ = cam_.cd().add_callback(
        std::bind(&Driver::eventCallback, this, ph::_1, ph::_2));
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
      ROS_INFO(
        "rate[Mevs] avg: %7.3f, max: %7.3f, send msg sz: %7.2f ev", avgRate,
        maxRate_, avgSize);
      maxRate_ = 0;
      lastPrintTime_ += statisticsPrintInterval_;
      totalEvents_ = 0;
      totalTime_ = 0;
      totalMsgsSent_ = 0;
      totalEventsSent_ = 0;
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
    int width_;     // image width
    int height_;    // image height
    uint32_t seq_;  // ROS sequence number
    boost::shared_ptr<MsgType> msg_;
    // related to statistics
    int64_t statisticsPrintInterval_{1000000};
    float maxRate_{0};
    uint64_t totalEvents_{0};
    float totalTime_{0};
    int64_t lastPrintTime_{0};
    size_t totalMsgsSent_{0};
    size_t totalEventsSent_{0};
};
}  // namespace metavision_ros_driver
#endif
