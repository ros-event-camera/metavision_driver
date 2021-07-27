// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "metavision_ros_driver/driver.h"

#include <dvs_msgs/EventArray.h>

#include <functional>

namespace metavision_ros_driver
{
namespace ph = std::placeholders;

Driver::Driver(const ros::NodeHandle & nh) : nh_(nh) {}

Driver::~Driver() { shutdown(); }

void Driver::shutdown()
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

bool Driver::initialize()
{
  double print_interval;
  nh_.param<double>("statistics_print_interval", print_interval, 1.0);
  statisticsPrintInterval_ = (int)(print_interval * 1e6);
  messageTimeThreshold_ =
    ros::Duration(nh_.param<double>("message_time_threshold", 1e-9));
  eventPublisher_ = nh_.advertise<dvs_msgs::EventArray>("events", 1);
  if (!startCamera()) {
    ROS_ERROR_STREAM("could not start camera!");
    return (false);
  }
  ROS_INFO_STREAM("driver initialized successfully.");
  return (true);
}

bool Driver::startCamera()
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
    // default frame id to last 4 digits of serial number
    auto tail = sn.substr(sn.size() - 4);
    msg_.header.frame_id = nh_.param<std::string>("frame_id", tail);
    ROS_INFO_STREAM("camera serial number: " << sn);
    const auto & g = cam_.geometry();
    msg_.width = g.width();
    msg_.height = g.height();
    msg_.header.seq = 0;

    ROS_INFO_STREAM(
      "frame_id: " << msg_.header.frame_id << ", size: " << msg_.width << " x "
                   << msg_.height);

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

void Driver::runtimeErrorCallback(const Metavision::CameraException & e)
{
  ROS_WARN_STREAM("camera runtime error occured: " << e.what());
}

void Driver::statusChangeCallback(const Metavision::CameraStatus & s)
{
  ROS_INFO_STREAM(
    "camera "
    << (s == Metavision::CameraStatus::STARTED ? "started." : "stopped."));
}

void Driver::updateStatistics(const EventCD * start, const EventCD * end)
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
      "rate avg: %7.3f Mevs, max: %7.3f "
      "Mevs, send msg sz: %7.2f",
      avgRate, maxRate_, avgSize);
    maxRate_ = 0;
    lastPrintTime_ += statisticsPrintInterval_;
    totalEvents_ = 0;
    totalTime_ = 0;
    totalMsgsSent_ = 0;
    totalEventsSent_ = 0;
  }
}

void Driver::eventCallback(const EventCD * start, const EventCD * end)
{
  const size_t n = end - start;
  if (n != 0) {
    updateStatistics(start, end);
    if (eventPublisher_.getNumSubscribers() > 0) {
      auto & events = msg_.events;
      const size_t old_size = events.size();
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
      const ros::Time & t_msg = msg_.events.begin()->ts;
      const ros::Time & t_last = msg_.events.rbegin()->ts;
      if (t_last > t_msg + messageTimeThreshold_) {
        msg_.header.seq++;
        msg_.header.stamp = t_msg;
        eventPublisher_.publish(msg_);
        totalEventsSent_ += msg_.events.size();
        totalMsgsSent_++;
        msg_.events.clear();
      }
    }
  }
}

}  // namespace metavision_ros_driver
