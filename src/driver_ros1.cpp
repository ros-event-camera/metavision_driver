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

#include "metavision_ros_driver/driver_ros1.h"

#include <event_array_msgs/EventArray.h>

#include "metavision_ros_driver/check_endian.h"
#include "metavision_ros_driver/metavision_wrapper.h"

namespace metavision_ros_driver
{
namespace ph = std::placeholders;
DriverROS1::DriverROS1(ros::NodeHandle & nh) : nh_(nh)
{
  configureWrapper(ros::this_node::getName());

  eventPub_ = nh_.advertise<std_msgs::Header>("events", nh_.param<int>("out_ros_queue_size", 1000));

  if (wrapper_->getSyncMode() == "primary") {
    // defer starting the primary until the secondary is up
    const std::string topic = "ready";
    ROS_INFO_STREAM("waiting for ready message from secondary on topic \"" << topic << "\"");
    secondaryReadySub_ = nh_.subscribe(topic, 1, &DriverROS1::secondaryReadyCallback, this);
  } else {
    if (wrapper_->getSyncMode() == "secondary") {
      secondaryReadyPub_ = nh_.advertise<std_msgs::Header>("ready", 1);
    }
    if (!start()) {
      ROS_ERROR("startup failed!");
      throw std::runtime_error("startup of CameraDriver node failed!");
    }
  }
}

DriverROS1::~DriverROS1()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

bool DriverROS1::sendReadyMessage()
{
  if (wrapper_->getSyncMode() == "secondary") {
    // secondary does not see sync signal from the master, so at given intervals
    // send a "ready" message to the primary so it knows it can start
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

bool DriverROS1::saveBiases(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
{
  (void)req;
  res.success = false;
  if (wrapper_) {
    res.success = wrapper_->saveBiases();
  }
  res.message += (res.success ? "succeeded" : "failed");
  return (res.success);
}

void DriverROS1::setBias(int * current, const std::string & name)
{
  wrapper_->setBias(name, *current);
  const int new_val = wrapper_->getBias(name);
  *current = new_val;  // feed back if not changed to desired value!
}

void DriverROS1::configure(Config & config, int level)
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

void DriverROS1::secondaryReadyCallback(const std_msgs::Header::ConstPtr &)
{
  secondaryReadySub_.shutdown();
  ROS_INFO("primary sees secondary is ready, starting camera!");
  if (!start()) {
    ROS_ERROR("startup failed!");
    throw std::runtime_error("startup of DriverROS1 node failed!");
  }
}

bool DriverROS1::start()
{
  if (!wrapper_->initialize(
        nh_.param<bool>("use_multithreading", false), nh_.param<std::string>("bias_file", ""))) {
    ROS_ERROR("driver initialization failed!");
    return (false);
  }
  lastReadyTime_ = ros::Time::now() - readyIntervalTime_;  // move to past

  if (frameId_.empty()) {
    // default frame id to last 4 digits of serial number
    const auto sn = wrapper_->getSerialNumber();
    frameId_ = sn.substr(sn.size() - 4);
  }
  ROS_INFO_STREAM("using frame id: " << frameId_);

  // ------ get other parameters from camera
  width_ = wrapper_->getWidth();
  height_ = wrapper_->getHeight();
  isBigEndian_ = check_endian::isBigEndian();

  // ------ start camera, may get callbacks from then on
  wrapper_->startCamera(this);

  // hook up dynamic config server *after* the camera has
  // been initialized so we can read the bias values
  configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
  configServer_->setCallback(boost::bind(&DriverROS1::configure, this, _1, _2));

  saveBiasService_ = nh_.advertiseService("save_biases", &DriverROS1::saveBiases, this);
  return (true);
}

bool DriverROS1::stop()
{
  if (wrapper_) {
    return (wrapper_->stop());
  }
  return (false);
}

static MetavisionWrapper::HardwarePinConfig get_hardware_pin_config(
  const ros::NodeHandle & nh, std::string nn)
{
  MetavisionWrapper::HardwarePinConfig config;
  XmlRpc::XmlRpcValue pin_config;
  // strip leading "/" from node name
  nn.erase(std::remove(nn.begin(), nn.end(), '/'), nn.end());
  std::string path = nn + "/ros__parameters/prophesee_pin_config";
  nh.getParam(path, pin_config);
  if (pin_config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR_STREAM("got invalid config, no trigger parameter with name " << path);
    ROS_ERROR_STREAM("node name " << nn << " must match pin config yaml file!");
    throw std::runtime_error("invalid hardware pin config!");
  }
  for (const auto & pin_map : pin_config) {
    config.emplace(pin_map.first, std::map<std::string, int>{});
    for (auto & pin_entry : pin_map.second) {
      config[pin_map.first][pin_entry.first] = static_cast<int>(pin_entry.second);
    }
  }
  return (config);
}

void DriverROS1::configureWrapper(const std::string & name)
{
  wrapper_ = std::make_shared<MetavisionWrapper>(name);
  wrapper_->setSerialNumber(nh_.param<std::string>("serial", ""));
  wrapper_->setFromFile(nh_.param<std::string>("from_file", ""));
  wrapper_->setSyncMode(nh_.param<std::string>("sync_mode", "standalone"));
  auto roi = nh_.param<std::vector<int>>("roi", std::vector<int>());
  if (!roi.empty()) {
    ROS_INFO_STREAM("using ROI with " << (roi.size() / 4) << " rectangle(s)");
  }
  wrapper_->setROI(roi);
  ROS_INFO_STREAM("sync mode: " << wrapper_->getSyncMode());
  // disabled, enabled, loopback
  wrapper_->setExternalTriggerInMode(nh_.param<std::string>("trigger_in_mode", "disabled"));
  // disabled, enabled
  wrapper_->setExternalTriggerOutMode(
    nh_.param<std::string>("trigger_out_mode", "disabled"),
    nh_.param<int>("trigger_out_period", 100000),   // trigger out period in usec
    nh_.param<double>("trigger_duty_cycle", 0.5));  // fraction of cycle that trigger is HIGH

  // disabled, enabled, na
  wrapper_->setEventRateController(
    nh_.param<std::string>("erc_mode", "na"),  // Event Rate Controller Mode
    nh_.param<int>("erc_rate", 100000000));    // Event Rate Controller Rate

  // Get information on external pin configuration per hardware setup
  if (wrapper_->triggerActive()) {
    wrapper_->setHardwarePinConfig(get_hardware_pin_config(nh_, ros::this_node::getName()));
  }
}

void DriverROS1::rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end)
{
  if (eventPub_.getNumSubscribers() != 0) {
    if (!msg_) {
      msg_.reset(new EventArrayMsg());
      msg_->header.frame_id = frameId_;
      msg_->header.seq = seq_++;
      msg_->time_base = 0;  // not used here
      msg_->encoding = encoding_;
      msg_->seq = msg_->header.seq;
      msg_->width = width_;
      msg_->height = height_;
      msg_->header.stamp = ros::Time().fromNSec(t);
      msg_->events.reserve(reserveSize_);
    }
    const size_t n = end - start;
    auto & events = msg_->events;
    const size_t oldSize = events.size();
    resize_hack(events, oldSize + n);
    memcpy(reinterpret_cast<void *>(events.data() + oldSize), start, n);

    if (t - lastMessageTime_ > messageThresholdTime_ || events.size() > messageThresholdSize_) {
      reserveSize_ = std::max(reserveSize_, events.size());
      eventPub_.publish(std::move(msg_));
      lastMessageTime_ = t;
      wrapper_->updateBytesSent(events.size());
      wrapper_->updateMsgsSent(1);
    }
  } else {
    if (msg_) {
      msg_.reset();
    }
  }
}

void DriverROS1::eventCDCallback(
  uint64_t, const Metavision::EventCD * start, const Metavision::EventCD * end)
{
  // check if there are valid timestamps then the primary is ready
  bool hasZeroTime(false);
  for (auto e = start; e != end; e++) {
    if (e->t == 0) {
      hasZeroTime = true;
      break;
    }
  }
  if (hasZeroTime) {
    // tell primary we are up so it can start the camera
    sendReadyMessage();
  } else {
    // alright, finaly the primary is up, no longer need the expensive
    // ddociding
    ROS_INFO("secondary sees primary up!");
    wrapper_->setDecodingEvents(false);
  }
}

}  // namespace metavision_ros_driver
