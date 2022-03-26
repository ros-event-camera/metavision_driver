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
#include <tuple>
#include <utility>

#include "metavision_ros_driver/MetaVisionDynConfig.h"
#include "metavision_ros_driver/metavision_wrapper.h"

// skip first few packets that may have bad time stamps
// (time is in nanoseconds)
#define SKIP_TIME 2000000000

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
    auto roi = nh_.param<std::vector<int>>("roi", std::vector<int>());
    if (!roi.empty()) {
      ROS_INFO_STREAM("using ROI with " << (roi.size() / 4) << " rectangle(s)");
    }
    wrapper_->setROI(roi);

    // disabled, enabled, loopback
    const std::string triggerInMode = nh_.param<std::string>("trigger_in_mode", "disabled");
    // disabled, enabled
    const std::string triggerOutMode = nh_.param<std::string>("trigger_out_mode", "disabled");
    // trigger out period in usec
    const int triggerOutPeriod = nh_.param<int>("trigger_out_period", 100000);
    // fraction of cycle that trigger is HIGH
    const double triggerOutDutyCycle = nh_.param<double>("trigger_duty_cycle", 0.5);

    wrapper_->setExternalTriggerInMode(triggerInMode);
    wrapper_->setExternalTriggerOutMode(triggerOutMode, triggerOutPeriod, triggerOutDutyCycle);

    if (triggerInMode != "disabled") {
      triggerState_.pub =
        nh_.advertise<MsgType>("trigger", nh_.param<int>("send_queue_size", 1000));
    }

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
    // set up event message state
    const double ett = nh_.param<double>("event_message_time_threshold", 100e-6);
    LOG_INFO("event message time threshold: " << ett);
    eventState_.msgThreshold = static_cast<uint64_t>(ett * 1e9);
    eventState_.reserveSize =
      static_cast<size_t>(nh_.param<double>("sensors_max_mevs", 50.0) * 1.0e6 * ett);
    LOG_INFO("using event message reserve size: " << eventState_.reserveSize);
    eventState_.pub = nh_.advertise<MsgType>("events", nh_.param<int>("send_queue_size", 1000));

    // set up trigger message state
    const double ttt = nh_.param<double>("trigger_message_time_threshold", 100e-6);
    LOG_INFO("trigger message time threshold: " << ttt);
    triggerState_.msgThreshold = static_cast<uint64_t>(ttt * 1e9);
    triggerState_.reserveSize =
      static_cast<size_t>(nh_.param<double>("tigger_max_freq", 1000.0) * ttt);
    triggerState_.pub = nh_.advertise<MsgType>("trigger", nh_.param<int>("send_queue_size", 1000));

    if (syncMode_ == "secondary") {
      secondaryReadyPub_ = nh_.advertise<std_msgs::Header>("ready", 1);
    }

    // Get information on external pin configuration per hardware setup
    wrapper_->setHardwarePinConfig(getHardwarePinConfig());

    if (!wrapper_->initialize(
          nh_.param<bool>("use_multithreading", false),
          nh_.param<double>("statistics_print_interval", 1.0),
          nh_.param<std::string>("bias_file", ""))) {
      ROS_ERROR("driver initialization failed!");
      return (false);
    }
    width_ = wrapper_->getWidth();
    height_ = wrapper_->getHeight();
    const union {
      uint32_t i;
      char c[4];
    } combined_int = {0x01020304};  // from stackoverflow
    isBigEndian_ = (combined_int.c[0] == 1);
    lastReadyTime_ = ros::Time::now() - readyIntervalTime_;  // move to past

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

  MetavisionWrapper::HardwarePinConfig getHardwarePinConfig()
  {
    MetavisionWrapper::HardwarePinConfig config;
    XmlRpc::XmlRpcValue hardware_pin_config;
    nh_.getParam("prophesee_pin_config", hardware_pin_config);
    ROS_ASSERT(hardware_pin_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    auto iter = hardware_pin_config.begin();

    for (; iter != hardware_pin_config.end(); iter++) {
      config.emplace(iter->first, std::map<std::string, int>{});

      auto pin_map_iter = iter->second.begin();

      for (; pin_map_iter != iter->second.end(); pin_map_iter++) {
        config[iter->first][pin_map_iter->first] = static_cast<int>(pin_map_iter->second);
      }
    }
    return (config);
  }

  bool keepRunning() override { return (ros::ok()); }

private:
  // MsgState holds the message and all other
  // pieces that are needed to build and send the message
  struct MsgState
  {
    uint64_t seq{0};            // sequence number
    uint64_t rosTimeOffset{0};  // rosTimeOffset for current message
    size_t reserveSize{0};
    uint64_t msgThreshold;         // min duration (nsec) until msg is sent
    std::unique_ptr<MsgType> msg;  // pointer to message itself
    ros::Publisher pub;            // ros publisher
  };

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

  inline bool waitForGoodTimestamp(int64_t sensorElapsedTime)
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

  void allocateMessageIfNeeded(MsgState * state, int64_t sensorElapsedTime, int width, int height)
  {
    auto & msg = state->msg;
    if (!state->msg) {  // must allocate new message
                        // remember the current rostime offset such that it will be kept
                        // constant until this message is sent out.
      state->rosTimeOffset = rosTimeOffset_;
      msg.reset(new MsgType());
      msg->header.frame_id = frameId_;
      msg->header.seq = state->seq++;
      msg->width = width;
      msg->height = height;
      msg->header.stamp.fromNSec(state->rosTimeOffset + sensorElapsedTime);
      msg->events.reserve(state->reserveSize);
    }
  }

  void allocateMessageIfNeeded(
    MsgState * state, int64_t sensorElapsedTime, int width, int height,
    const std::string & encoding)
  {
    auto & msg = state->msg;
    if (!state->msg) {  // must allocate new message
      // remember the current rostime offset such that it will be kept
      // constant until this message is sent out.
      state->rosTimeOffset = rosTimeOffset_;
      msg.reset(new event_array_msgs::msg::EventArray());
      msg->header.frame_id = frameId_;
      msg->header.seq = state->seq++;
      msg->width = width;
      msg->height = height;
      msg->header.stamp.fromNSec(state->rosTimeOffset + sensorElapsedTime);
      msg->events.reserve(state->reserveSize * 8);  // 8 bytes per event
      msg->is_bigendian = isBigEndian_;
      msg->encoding = encoding;
      msg->time_base = sensorElapsedTime;  // to allow original time stamp reconstruction
      msg->seq = state->seq;               // duplicate, but keep for symmetry with ROS2
    }
  }

  inline bool sendMessageIfComplete(MsgState * state, int64_t last_event_time, size_t events_sent)
  {
    const uint64_t latestTime = state->rosTimeOffset + last_event_time;
    const ros::Time msgStartTime(state->msg->header.stamp);
    if (latestTime >= msgStartTime.NSec() + state->msgThreshold) {
      wrapper_->updateEventsSent(events_sent);
      wrapper_->updateMsgsSent(1);
      // the std::move should reset the message
      state->pub->publish(std::move(state->msg));
      return (true);
    }
    return (false);
  }

  void triggerCallback(
    const Metavision::EventExtTrigger * start, const Metavision::EventExtTrigger * end) override
  {
    const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      return;
    }
    int eventCount[2] = {0, 0};
    const size_t n = end - start;
    MsgState & state = triggerState_;
    if (state.pub->get_subscription_count() > 0) {
      allocateMessageIfNeeded(&state, sensorElapsedTime, 1 /* width */, 1 /*height */);
      auto & events = state.msg->events;
      const size_t old_size = events.size();
      // With proper reserved capacity, the resize should not trigger a copy.
      events.resize(events.size() + n);
      // copy data into ROS message.
      for (unsigned int i = 0; i < n; i++) {
        const auto & e_src = start[i];
        auto & e_trg = events[i + old_size];
        e_trg.x = 0;
        e_trg.y = 0;
        e_trg.polarity = e_src.p;
        e_trg.ts.fromNSec(state.rosTimeOffset + e_src.t * 1000);
        eventCount[e_src.p]++;
      }
      (void)sendMessageIfComplete(&state, start[n - 1].t * 1000, events.size());
    } else {
      // no subscribers: discard unfinished message and gather event statistics
      state.msg.reset();
      for (unsigned int i = 0; i < n; i++) {
        eventCount[start[i].p]++;
      }
    }

    wrapper_->updateEventCount(0, eventCount[0]);
    wrapper_->updateEventCount(1, eventCount[1]);
  }

  void eventCallback(const Metavision::EventCD * start, const Metavision::EventCD * end) override
  {
    const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
    if (waitForGoodTimestamp(sensorElapsedTime)) {
      // I'm the secondary and the primary is not running yet, so my time stamps
      // are bad (0)
      return;
    }
    MsgState & state = eventState_;
    const size_t n = end - start;
    int eventCount[2] = {0, 0};
    if (pub_.getNumSubscribers() != 0 && sensorElapsedTime > SKIP_TIME) {
      if (!state.msg) {
        // update the difference between ROS time and sensor time.
        // Only do so on message start to reduce compute time
        rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
      }
      allocateMessageIfNeeded(&state, sensorElapsedTime, width_, height_);
      const size_t old_size = events.size();
      // With proper reserved capacity, the resize should not trigger a copy.
      events.resize(events.size() + n);
      // copy data into ROS message. For the SilkyEvCam
      // the full load packet size delivered by the SDK is 320
      for (unsigned int i = 0; i < n; i++) {
        const auto & e_src = start[i];
        auto & e_trg = events[i + old_size];
        e_trg.x = e_src.x;
        e_trg.y = e_src.y;
        e_trg.polarity = e_src.p;
        e_trg.ts.fromNSec(state.rosTimeOffset + e_src.t * 1000);
        eventCount[e_src.p]++;
      }
      // must keep the rostime of the last event for maintaining
      // the offset
      const int64_t lastEventTime = start[n - 1].t * 1000;
      lastROSTime_ = rosTimeOffset_ + lastEventTime;
      (void)sendMessageIfComplete(&state, lastEventTime, events.size());
    } else {
      // no subscribers: discard unfinished message and gather event statistics
      state.msg.reset();
      for (unsigned int i = 0; i < n; i++) {
        eventCount[start[i].p]++;
      }
    }
    wrapper_->updateEventCount(0, eventCount[0]);
    wrapper_->updateEventCount(1, eventCount[1]);
  }

  // ------------ variables
  ros::NodeHandle nh_;
  std::shared_ptr<MetavisionWrapper> wrapper_;
  int width_;            // image width
  int height_;           // image height
  std::string frameId_;  // ROS frame id
  bool isBigEndian_{false};
  // ----- dynamic configuration for biases
  Config config_;
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
  ros::ServiceServer saveBiasService_;
  // -------- related to camerainfo
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  std::string cameraInfoURL_;
  sensor_msgs::CameraInfo cameraInfoMsg_;
  // ------- related to time keeping
  uint64_t rosT0_{0};                // time when first callback happened
  double averageTimeDifference_{0};  // average of elapsed_ros_time - elapsed_sensor_time
  double prevSensorTime_{0};         // sensor time during previous update
  int64_t bufferingDelay_{0};        // estimate of buffering delay
  uint64_t rosTimeOffset_{0};        // roughly rosT0_ + averageTimeDifference_
  uint64_t lastROSTime_{0};          // the last event's ROS time stamp
  // ------- state related to message publishing
  MsgState eventState_;    // state for sending event message
  MsgState triggerState_;  // state for sending trigger message
  // ------ related to sync
  rclcpp::Duration readyIntervalTime_;  // frequency of publishing ready messages
  rclcpp::Time lastReadyTime_;          // last time ready message was published
  std::string syncMode_;                // primary, secondary, standalone
  ros::Publisher secondaryReadyPub_;
  ros::Subscriber secondaryReadySub_;
  ros::Duration readyIntervalTime_{1.0};  // frequency of publishing ready messages
  ros::Time lastReadyTime_;               // last time ready message was published
};

inline size_t resize_message(event_array_msgs::msg::EventArray * msg, size_t n)
{
  const size_t oldSize = msg->events.size();
  const size_t newSize = oldSize + n * 8;
  msg->events.resize(newSize);
  return (oldSize);
}

template <>
void DriverROS1<event_array_msgs::EventArray>::eventCallback(
  const Metavision::EventCD * start, const Metavision::EventCD * end)
{
  const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
  if (waitForGoodTimestamp(sensorElapsedTime)) {
    return;
  }

  int eventCount[2] = {0, 0};
  const size_t n = end - start;
  MsgState & state = eventState_;

  // skip the first 2 seconds of packets to work around bad initial time stamps
  // on SilkyEV
  if (state.pub->get_subscription_count() > 0 && sensorElapsedTime > SKIP_TIME) {
    if (!state.msg) {
      // update the difference between ROS time and sensor time.
      // Only do so on message start
      rosTimeOffset_ = updateROSTimeOffset(sensorElapsedTime);
    }
    allocateMessageIfNeeded(&state, sensorElapsedTime, width_, height_, "mono");
    // If capacity is sufficient the resize should not trigger a copy
    const size_t old_size = resize_message(state.msg.get(), n);

    // copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is n = 320
    uint64_t * pyxt = reinterpret_cast<uint64_t *>(&(msg_->events[old_size]));
    const uint64_t headerStamp = msg_->header.stamp.toNSec();
    for (unsigned int i = 0; i < n; i++) {
      const auto & e = start[i];
      const uint64_t ts = rosTimeOffset_ + e.t * 1000;
      const uint64_t dt = (ts - headerStamp) & 0xFFFFFFFFULL;  // limit to 32 bits
      event_array_msgs::mono::encode(pyxt + i, e.p, e.x, e.y, dt);
      eventCount[e.p]++;
    }

    // update lastROSTime_ with latest event time stamp
    const int64_t lastEventTime = start[n - 1].t * 1000;
    lastROSTime_ = rosTimeOffset_ + lastEventTime;
    (void)sendMessageIfComplete(&state, lastEventTime, events.size() / 8);
  } else {
    // no subscribers: clear out unfinished message and gather event statistics
    state.msg.reset();
    for (unsigned int i = 0; i < n; i++) {
      eventCount[start[i].p]++;
    }
  }
  wrapper_->updateEventCount(0, eventCount[0]);
  wrapper_->updateEventCount(1, eventCount[1]);
}

template <>
void DriverROS1<event_array_msgs::EventArray>::triggerCallback(
  const Metavision::EventExtTrigger * start, const Metavision::EventExtTrigger * end)
{
  const int64_t sensorElapsedTime = start->t * 1000;  // nanosec
  if (waitForGoodTimestamp(sensorElapsedTime)) {
    // I'm the secondary and the primary is not running yet, so my time stamps
    // are bad (0)
    return;
  }

  int eventCount[2] = {0, 0};
  const size_t n = end - start;
  MsgState & state = triggerState_;

  // skip the first 2 seconds of packets to work around bad initial time stamps
  // on SilkyEV
  if (externalTriggerPub_.getNumSubscribers() > 0 && sensorElapsedTime > SKIP_TIME) {
    allocateMessageIfNeeded(&state, sensorElapsedTime, 1 /* width */, 1 /*height */, "trigger");
    const size_t old_size = state->msg.events.size();
    // With proper reserved capacity the resize should not trigger a
    // copy
    externalTriggerMsg_->events.resize(old_size + n * 8);
    // copy data into ROS message. For the SilkyEvCam
    // the full load packet size delivered by the SDK is n = 320

    uint64_t * pyxt = reinterpret_cast<uint64_t *>(&(externalTriggerMsg_->events[old_size]));
    const uint64_t headerStamp = state->msg.header.stamp.toNSec();
    for (unsigned int i = 0; i < n; i++) {
      const auto & e = start[i];
      const uint64_t ts = state.rosTimeOffset + e.t * 1000;
      const uint64_t dt = (ts - headerStamp) & 0xFFFFFFFFULL;  // limit to 32 bits
      event_array_msgs::special::encode(pyxt + i, e.p, dt);
      eventCount[e.p]++;
    }
    const int64_t lastEventTime = start[n - 1].t * 1000;
    (void)sendMessageIfComplete(&state, lastEventTime, events.size() / 8);
  } else {
    // no subscribers: clear out unfinished message and gather event statistics
    state.msg.reset();
    for (unsigned int i = 0; i < n; i++) {
      eventCount[start[i].p]++;
    }
  }
}

}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__DRIVER_ROS1_H_
