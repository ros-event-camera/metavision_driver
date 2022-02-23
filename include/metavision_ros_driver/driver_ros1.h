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

#ifndef METAVISION_ROS_DRIVER__DRIVER_ROS1_H_
#define METAVISION_ROS_DRIVER__DRIVER_ROS1_H_

#include <camera_info_manager/camera_info_manager.h>
#include <dvs_msgs/EventArray.h>
#include <dynamic_reconfigure/server.h>
#include <event_array_msgs/EventArray.h>
#include <event_array_msgs/encode.h>
#include <metavision/sdk/driver/camera.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>

#include "metavision_ros_driver/MetaVisionDynConfig.h"
#include "metavision_ros_driver/metavision_wrapper.h"

namespace metavision_ros_driver
{
namespace ph = std::placeholders;
template <class MsgType>

class DriverROS1 : public CallbackHandler
{
public:
  using Config = MetaVisionDynConfig;
  explicit DriverROS1(const ros::NodeHandle & nh, const std::string & name) : nh_(nh)
  {
    wrapper_ = std::make_shared<MetavisionWrapper>(name);
    wrapper_->setSerialNumber(nh_.param<std::string>("serial", ""));
    cameraInfoURL_ = nh_.param<std::string>("camerainfo_url", "");
    frameId_ = nh_.param<std::string>("frame_id", "");
    syncMode_ = nh_.param<std::string>("sync_mode", "standalone");
    ROS_INFO_STREAM("sync mode: " << syncMode_);
    wrapper_->setSyncMode(syncMode_);
    if (syncMode_ == "primary") {  // defer starting until secondary is up
      const std::string topic = "ready";
      ROS_INFO_STREAM("waiting for ready message from secondary on topic \"" << topic << "\"");
      secondaryReadySub_ = nh_.subscribe(topic, 1, &DriverROS1::secondaryReady, this);
    } else {
      if (!start()) {
        ROS_ERROR("startup failed!");
        throw std::runtime_error("startup of CameraDriver node failed!");
      }
    }
  }
  ~DriverROS1()
  {
    stop();
    wrapper_.reset();  // invoke destructor
  }

  void setBias(int * current, const std::string & name)
  {
    wrapper_->setBias(name, *current);
    const int new_val = wrapper_->getBias(name);
    *current = new_val;  // feed back if not changed to desired value!
  }

  void configure(Config & config, int level)
  {
    if (level < 0) {  // initial call
      // initialize config from current settings
      config.bias_diff = wrapper_->getBias("bias_diff");
      config.bias_diff_off = wrapper_->getBias("bias_diff_off");
      config.bias_diff_on = wrapper_->getBias("bias_diff_on");
      config.bias_fo = wrapper_->getBias("bias_fo");
      config.bias_hpf = wrapper_->getBias("bias_hpf");
      config.bias_pr = wrapper_->getBias("bias_pr");
      config.bias_refr = wrapper_->getBias("bias_refr");
      ROS_INFO("initialized config to camera biases");
    } else {
      setBias(&config.bias_diff, "bias_diff");
      setBias(&config.bias_diff_off, "bias_diff_off");
      setBias(&config.bias_diff_on, "bias_diff_on");
      setBias(&config.bias_fo, "bias_fo");
      setBias(&config.bias_hpf, "bias_hpf");
      setBias(&config.bias_pr, "bias_pr");
      setBias(&config.bias_refr, "bias_refr");
    }
    config_ = config;  // remember current values
  }

  bool start()
  {
    const double mtt = nh_.param<double>("message_time_threshold", 100e-6);
    messageTimeThreshold_ = static_cast<uint64_t>(mtt * 1e9);
    reserveSize_ =
      static_cast<size_t>(nh_.param<double>("sensors_max_mevs", 50.0) / std::max(mtt, 1e-6));
    pub_ = nh_.advertise<MsgType>("events", nh_.param<int>("send_queue_size", 1000));
    if (syncMode_ == "secondary") {
      secondaryReadyPub_ = nh_.advertise<std_msgs::Header>("ready", 1);
    }

    if (!wrapper_->initialize(
          nh_.param<bool>("use_multithreading", false),
          nh_.param<double>("statistics_print_interval", 1.0),
          nh_.param<std::string>("bias_file", ""))) {
      ROS_ERROR("driver initialization failed!");
      return (false);
    }
    width_ = wrapper_->getWidth();
    height_ = wrapper_->getHeight();
    if (frameId_.empty()) {
      // default frame id to last 4 digits of serial number
      const auto sn = wrapper_->getSerialNumber();
      frameId_ = sn.substr(sn.size() - 4);
    }
    wrapper_->startCamera(this);
    ROS_INFO_STREAM("using frame id: " << frameId_);

    infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(nh_, cameraInfoURL_);
    cameraInfoMsg_ = infoManager_->getCameraInfo();
    cameraInfoMsg_.header.frame_id = frameId_;

    // hook up dynamic config server *after* the camera has
    // been initialized so we can read the bias values
    configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
    configServer_->setCallback(boost::bind(&DriverROS1::configure, this, _1, _2));

    saveBiasService_ = nh_.advertiseService("save_biases", &DriverROS1::saveBiases, this);

    ROS_INFO_STREAM("driver initialized successfully.");
    return (true);
  }

  void publish(const Metavision::EventCD * start, const Metavision::EventCD * end) override
  {
    const double sensorElapsedTime = start->t * 1e3;  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      return;
    }
    if (pub_.getNumSubscribers() == 0 || sensorElapsedTime < 2e9) {
      return;
    }
    if (!msg_) {  // must allocate new message
      // update the rosTimeOffset only when a new ROS message is started.
      // This reduces compute load but more crucially it ensures that
      // the time basis stays constant within each ROS message.
      rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
      msg_.reset(new MsgType());
      msg_->header.frame_id = frameId_;
      msg_->header.seq = seq_++;
      msg_->width = width_;
      msg_->height = height_;
      msg_->header.stamp.fromNSec(rosTimeOffset_ + static_cast<uint64_t>(sensorElapsedTime));
      msg_->events.reserve(reserveSize_);
    }
    const size_t n = end - start;
    auto & events = msg_->events;
    const size_t old_size = events.size();
    // With proper reserved capacity, the resize should not trigger a copy.
    events.resize(events.size() + n);
    // copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is 320
    int eventCount[2] = {0, 0};
    for (unsigned int i = 0; i < n; i++) {
      const auto & e_src = start[i];
      auto & e_trg = events[i + old_size];
      e_trg.x = e_src.x;
      e_trg.y = e_src.y;
      e_trg.polarity = e_src.p;
      e_trg.ts.fromNSec(rosTimeOffset_ + static_cast<uint64_t>(e_src.t * 1e3));
      eventCount[e_src.p]++;
    }
    wrapper_->updateEventCount(0, eventCount[0]);
    wrapper_->updateEventCount(1, eventCount[1]);
    lastROSTime_ = rosTimeOffset_ + static_cast<uint64_t>(start[n - 1].t * 1e3);
    if (lastROSTime_ > msg_->header.stamp.toNSec() + messageTimeThreshold_) {
      pub_.publish(msg_);
      wrapper_->updateEventsSent(events.size());
      wrapper_->updateMsgsSent(1);
      msg_.reset();  // no longer using this one
    }
  }

  bool keepRunning() override { return (ros::ok()); }

private:
  bool stop()
  {
    if (wrapper_) {
      return (wrapper_->stop());
    }
    return (false);
  }

  void secondaryReady(const std_msgs::Header::ConstPtr & msg)
  {
    (void)msg;
    secondaryReadySub_.shutdown();
    ROS_INFO("secondary is ready, starting primary!");
    if (!start()) {
      ROS_ERROR("startup failed!");
      throw std::runtime_error("startup of DriverROS1 node failed!");
    }
  }

  inline bool waitForGoodTimestamp(double sensorElapsedTime)
  {
    if (sensorElapsedTime == 0 && syncMode_ == "secondary") {
      const ros::Time t = ros::Time::now();
      if (lastReadyTime_ < t - readyIntervalTime_) {
        ROS_INFO_STREAM("secondary waiting for primary to come up");
        std_msgs::Header header;
        header.stamp = t;
        header.frame_id = frameId_;
        secondaryReadyPub_.publish(header);
        lastReadyTime_ = t;
      }
      return (true);
    }
    return (false);
  }

  bool saveBiases(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
  {
    (void)req;
    res.success = false;
    if (wrapper_) {
      res.success = wrapper_->saveBiases();
    }
    res.message += (res.success ? "succeeded" : "failed");
    return (res.success);
  }

  inline uint64_t updateROSTimeOffset(double dt_sensor)
  {
    const uint64_t rosT = ros::Time::now().toNSec();
    if (rosT0_ == 0) {
      rosT0_ = rosT;
      // initialize to dt_ros - dt_sensor because dt_ros == 0
      averageTimeDifference_ = -dt_sensor;
      lastROSTime_ = rosT;
      bufferingDelay_ = 0;
      prevSensorTime_ = dt_sensor;
    }
    // compute time in seconds elapsed since ROS startup
    const double dt_ros = static_cast<double>(rosT - rosT0_);
    // difference between elapsed ROS time and elapsed sensor Time
    const double dt = dt_ros - dt_sensor;
    // compute moving average of elapsed time difference
    // average over 10 seconds
    constexpr double f = 1.0 / (10e9);
    const double sensor_inc = dt_sensor - prevSensorTime_;
    const double alpha = std::min(sensor_inc * f, 0.1);
    averageTimeDifference_ = averageTimeDifference_ * (1.0 - alpha) + alpha * dt;
    prevSensorTime_ = dt_sensor;
    //
    // We want to use sensor time, but adjust it for the average clock
    // skew between sensor time and ros time, plus some unknown buffering delay dt_buf
    // (to be estimated)
    //
    // t_ros_adj
    //  = t_sensor + avg(t_ros - t_sensor) + dt_buf
    //  = t_sensor_0 + dt_sensor + avg(t_ros_0 + dt_ros - (t_sensor_0 + dt_sensor)) + dt_buf
    //          [now use t_sensor_0 and t_ros_0 == constant]
    //  = t_ros_0 + avg(dt_ros - dt_sensor) + dt_sensor + dt_buf
    //  =: ros_time_offset + dt_sensor;
    //
    // Meaning once ros_time_offset has been computed, the adjusted ros timestamp
    // is obtained by just adding the sensor elapsed time (dt_sensor) that is reported
    // by the SDK.

    const uint64_t dt_sensor_int = static_cast<uint64_t>(dt_sensor);
    const int64_t avg_timediff_int = static_cast<int64_t>(averageTimeDifference_);
    const uint64_t MIN_EVENT_DELTA_T = 0LL;  // minimum time gap between packets

    // First test if the new ros time stamp (trialTime) would be in future. If yes, then
    // the buffering delay has been underestimated and must be adjusted.

    const uint64_t trialTime = rosT0_ + avg_timediff_int + dt_sensor_int;

    if (rosT < trialTime + bufferingDelay_) {  // time stamp would be in the future
      bufferingDelay_ = -(trialTime - rosT);
    }

    // The buffering delay could make the time stamps go backwards.
    // Ensure that this does not happen. This safeguard may cause
    // time stamps to be (temporarily) in the future, there is no way around
    // that.
    if (trialTime + bufferingDelay_ < lastROSTime_ + MIN_EVENT_DELTA_T) {
      bufferingDelay_ = (int64_t)(lastROSTime_ + MIN_EVENT_DELTA_T) - (int64_t)trialTime;
    }

    const uint64_t rosTimeOffset = rosT0_ + avg_timediff_int + bufferingDelay_;

    return (rosTimeOffset);
  }

  // ------------ variables
  ros::NodeHandle nh_;
  std::shared_ptr<MetavisionWrapper> wrapper_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  ros::ServiceServer saveBiasService_;
  ros::Publisher pub_;
  ros::Publisher secondaryReadyPub_;
  ros::Subscriber secondaryReadySub_;
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
  Config config_;

  sensor_msgs::CameraInfo cameraInfoMsg_;
  boost::shared_ptr<MsgType> msg_;

  std::string cameraInfoURL_;
  uint64_t messageTimeThreshold_;    // duration (nsec) for triggering a ROS message
  size_t reserveSize_{0};            // how many events to preallocate per message
  int width_;                        // image width
  int height_;                       // image height
  std::string frameId_;              // ROS frame id
  std::string syncMode_;             // standalone, primary, secondary
  uint64_t seq_{0};                  // ROS sequence number
  uint64_t rosT0_{0};                // time when first callback happened
  double averageTimeDifference_{0};  // average of elapsed_ros_time - elapsed_sensor_time
  double prevSensorTime_{0};         // sensor time during previous update
  int64_t bufferingDelay_{0};        // estimate of buffering delay
  uint64_t rosTimeOffset_{0};        // roughly rosT0_ + averageTimeDifference_
  uint64_t lastROSTime_{0};          // the last event's ROS time stamp
  ros::Duration readyIntervalTime_{1.0};  // frequency of publishing ready messages
  ros::Time lastReadyTime_;               // last time ready message was published
};

template <>
void DriverROS1<event_array_msgs::EventArray>::publish(
  const Metavision::EventCD * start, const Metavision::EventCD * end)
{
  const double sensorElapsedTime = start->t * 1e3;  // nanosec
  if (waitForGoodTimestamp(sensorElapsedTime)) {
    return;
  }

  // skip the first 2 seconds of packets to work around bad initial time stamps
  // on SilkyEV
  if (pub_.getNumSubscribers() == 0 || sensorElapsedTime < 2e9) {
    return;
  }
  if (!msg_) {  // must allocate new message
    // update the rosTimeOffset only when a new ROS message is started.
    // This reduces compute load but more crucially it ensures that
    // the time basis stays constant within each ROS message.
    rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
    msg_.reset(new event_array_msgs::EventArray());
    msg_->header.frame_id = frameId_;
    msg_->header.seq = static_cast<uint32_t>(seq_++);
    msg_->width = width_;
    msg_->height = height_;
    msg_->encoding = "mono";
    msg_->time_base = static_cast<uint64_t>(sensorElapsedTime);
    msg_->header.stamp.fromNSec(rosTimeOffset_ + static_cast<uint64_t>(sensorElapsedTime));
    msg_->events.reserve(reserveSize_ * 8);
    msg_->seq = seq_;  // duplicate, but wanted symmetry with ROS2
  }
  const size_t n = end - start;
  const size_t old_size = msg_->events.size();
  // With proper reserved capacity the resize should not trigger a
  // copy
  msg_->events.resize(old_size + n * 8);
  // copy data into ROS message. For the SilkyEvCam
  // the full load packet size delivered by the SDK is n = 320
  int eventCount[2] = {0, 0};
  uint64_t * pyxt = reinterpret_cast<uint64_t *>(&(msg_->events[old_size]));
  const uint64_t headerStamp = msg_->header.stamp.toNSec();
  for (unsigned int i = 0; i < n; i++) {
    const auto & e = start[i];
    const uint64_t ts = rosTimeOffset_ + static_cast<uint64_t>(e.t * 1e3);
    const uint64_t dt = (ts - headerStamp) & 0xFFFFFFFFULL;  // limit to 32 bits
    event_array_msgs::mono::encode(pyxt + i, e.p, e.x, e.y, dt);
    eventCount[e.p]++;
  }
  wrapper_->updateEventCount(0, eventCount[0]);
  wrapper_->updateEventCount(1, eventCount[1]);
  lastROSTime_ = rosTimeOffset_ + static_cast<uint64_t>(start[n - 1].t * 1e3);
  if (lastROSTime_ > headerStamp + messageTimeThreshold_) {
    wrapper_->updateEventsSent(msg_->events.size() / 8);
    wrapper_->updateMsgsSent(1);
    pub_.publish(msg_);
    msg_.reset();  // start a new message
  }
}

}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__DRIVER_ROS1_H_
