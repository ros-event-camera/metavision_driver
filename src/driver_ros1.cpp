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

#include "metavision_driver/driver_ros1.h"

#include <event_camera_msgs/EventPacket.h>

#include "metavision_driver/check_endian.h"
#include "metavision_driver/metavision_wrapper.h"

namespace metavision_driver
{
namespace ph = std::placeholders;
DriverROS1::DriverROS1(ros::NodeHandle & nh) : nh_(nh)
{
  configureWrapper(ros::this_node::getName());

  encoding_ = nh.param<std::string>("encoding", "evt3");
  if (encoding_ != "evt3") {
    ROS_ERROR_STREAM("invalid encoding: " << encoding_);
    throw std::runtime_error("invalid encoding!");
  }
  messageThresholdTime_ =
    uint64_t(std::abs(nh_.param<double>("event_message_time_threshold", 1e-3) * 1e9));
  messageThresholdSize_ =
    static_cast<size_t>(std::abs(nh_.param<int>("event_message_size_threshold", 1024 * 1024)));

  eventPub_ = nh_.advertise<EventPacketMsg>("events", nh_.param<int>("send_queue_size", 1000));

  if (wrapper_->getSyncMode() == "primary") {
    // defer starting the primary until the secondary is up
    ros::ServiceClient client = nh_.serviceClient<Trigger>("ready");
    Trigger trig;
    while (!client.call(trig)) {
      ROS_INFO_STREAM("waiting for secondary to come up...");
      ros::Duration(0, 1000000000).sleep();  // sleep for a second
    }
    ROS_INFO_STREAM("secondary is up: " << trig.response.message);
    start();  // only now can this be started
  } else if (wrapper_->getSyncMode() == "secondary") {
    // on the secondary first start the camera before bringing
    // up the server
    start();
    secondaryReadyServer_ =
      nh_.advertiseService("ready", &DriverROS1::secondaryReadyCallback, this);
  } else {  // standalone mode
    start();
  }
}

DriverROS1::~DriverROS1()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

bool DriverROS1::saveBiases(Trigger::Request & req, Trigger::Response & res)
{
  (void)req;
  res.success = false;
  if (wrapper_) {
    res.success = wrapper_->saveBiases();
  }
  res.message += (res.success ? "succeeded" : "failed");
  return (res.success);
}

int DriverROS1::getBias(const std::string & name) const
{
  if (biasParameters_.find(name) != biasParameters_.end()) {
    return (wrapper_->getBias(name));
  }
  return (0);
}

void DriverROS1::setBias(int * field, const std::string & name)
{
  auto it = biasParameters_.find(name);
  if (it != biasParameters_.end()) {
    auto & bp = it->second;
    int val = std::min(std::max(*field, bp.minVal), bp.maxVal);
    if (val != *field) {
      ROS_WARN_STREAM(name << " must be between " << bp.minVal << " and " << bp.maxVal);
    }
    wrapper_->setBias(name, val);
    const int new_val = wrapper_->getBias(name);  // read back
    *field = new_val;                             // feed back if not changed to desired value!
  }
}

void DriverROS1::configure(Config & config, int level)
{
  if (level < 0) {  // initial call
    // initialize config from current settings
    config.bias_diff_off = getBias("bias_diff_off");
    config.bias_diff_on = getBias("bias_diff_on");
    config.bias_fo = getBias("bias_fo");
    config.bias_hpf = getBias("bias_hpf");
    config.bias_pr = getBias("bias_pr");
    config.bias_refr = getBias("bias_refr");
    ROS_INFO("initialized config to camera biases");
  } else {
    setBias(&config.bias_diff_off, "bias_diff_off");
    setBias(&config.bias_diff_on, "bias_diff_on");
    setBias(&config.bias_fo, "bias_fo");
    setBias(&config.bias_hpf, "bias_hpf");
    setBias(&config.bias_pr, "bias_pr");
    setBias(&config.bias_refr, "bias_refr");
  }
  config_ = config;  // remember current values
}

bool DriverROS1::secondaryReadyCallback(Trigger::Request &, Trigger::Response & res)
{
  res.success = true;
  res.message += "succeeded";
  return (true);
}

void DriverROS1::start()
{
  wrapper_->setStatisticsInterval(nh_.param<double>("statistics_print_interval", 1.0));
  if (!wrapper_->initialize(
        nh_.param<bool>("use_multithreading", false), nh_.param<std::string>("bias_file", ""))) {
    ROS_ERROR("driver initialization failed!");
    throw std::runtime_error("driver init failed!");
  }

  if (wrapper_->getSyncMode() == "secondary") {
    ROS_INFO("secondary is decoding events...");
    wrapper_->setDecodingEvents(true);
  }

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

  initializeBiasParameters(wrapper_->getSensorVersion());
  // hook up dynamic config server *after* the camera has
  // been initialized so we can read the bias values
  configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
  configServer_->setCallback(boost::bind(&DriverROS1::configure, this, _1, _2));

  saveBiasService_ = nh_.advertiseService("save_biases", &DriverROS1::saveBiases, this);
}

void DriverROS1::initializeBiasParameters(const std::string & sensorVersion)
{
  biasParameters_ = BiasParameter::getAll(sensorVersion);
  if (biasParameters_.empty()) {
    ROS_WARN_STREAM("unknown sensor version " << sensorVersion << ", disabling tunable biases");
  }
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
      msg_.reset(new EventPacketMsg());
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
      wrapper_->updateBytesSent(events.size());
      wrapper_->updateMsgsSent(1);
      eventPub_.publish(std::move(msg_));
      lastMessageTime_ = t;
      msg_.reset();
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
  // This callback will only be exercised during startup on the
  // secondary until good data is available. The moment good time stamps
  // are available we disable decoding and use the raw interface.
  bool hasZeroTime(false);
  for (auto e = start; e != end; e++) {
    if (e->t == 0) {
      hasZeroTime = true;
      break;
    }
  }
  if (!hasZeroTime) {
    // finally the primary is up, no longer need the expensive decoding
    ROS_INFO("secondary sees primary up!");
    wrapper_->setDecodingEvents(false);
  }
}

}  // namespace metavision_driver
