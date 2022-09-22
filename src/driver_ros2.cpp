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

#include "metavision_ros_driver/driver_ros2.h"

#include <event_array_msgs/msg/event_array.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "metavision_ros_driver/check_endian.h"
#include "metavision_ros_driver/logging.h"
#include "metavision_ros_driver/metavision_wrapper.h"

namespace metavision_ros_driver
{
DriverROS2::DriverROS2(const rclcpp::NodeOptions & options)
: Node(
    "metavision_ros_driver",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)),
  readyIntervalTime_(rclcpp::Duration::from_seconds(1.0))

{
  configureWrapper(get_name());

  this->get_parameter_or("encoding", encoding_, std::string("evt3"));
  if (encoding_ != "evt3") {
    LOG_ERROR("invalid encoding: " << encoding_);
    throw std::runtime_error("invalid encoding!");
  }
  double mtt;
  this->get_parameter_or("event_message_time_threshold", mtt, 1e-3);
  messageThresholdTime_ = uint64_t(std::abs(mtt) * 1e9);
  int64_t mts;
  this->get_parameter_or("event_message_size_threshold", mts, int64_t(1000000000));
  messageThresholdSize_ = static_cast<size_t>(std::abs(mts));

  int qs;
  this->get_parameter_or("out_ros_queue_size", qs, 1000);
  eventPub_ = this->create_publisher<EventArrayMsg>(
    "~/events", rclcpp::QoS(rclcpp::KeepLast(qs)).best_effort().durability_volatile());

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  if (wrapper_->getSyncMode() == "primary") {
    // defer starting the primary until secondary is up
    const std::string topic = "~/ready";
    LOG_INFO("waiting for secondary to publish ready message on topic \"" << topic << "\"");
    secondaryReadySub_ = this->create_subscription<std_msgs::msg::Header>(
      topic, qos, std::bind(&DriverROS2::secondaryReadyCallback, this, std::placeholders::_1));
  } else {
    if (wrapper_->getSyncMode() == "secondary") {
      secondaryReadyPub_ = this->create_publisher<std_msgs::msg::Header>("~/ready", qos);
    }
    if (!start()) {
      LOG_ERROR("startup failed!");
      throw std::runtime_error("startup of DriverROS2 node failed!");
    }
  }
}

DriverROS2::~DriverROS2()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

bool DriverROS2::sendReadyMessage()
{
  if (wrapper_->getSyncMode() == "secondary") {
    // secondary does not see sync signal from the master, so at given intervals
    // send a "ready" message to the primary so it knows it can start
    const rclcpp::Time t = this->now();
    if (lastReadyTime_ < t - readyIntervalTime_) {
      LOG_INFO("secondary waiting for primary to come up");
      std_msgs::msg::Header header;
      header.stamp = t;
      header.frame_id = frameId_;
      secondaryReadyPub_->publish(header);
      lastReadyTime_ = t;
    }
    return (true);
  }
  return (false);
}

void DriverROS2::saveBiases(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  response->success = false;
  response->message = "bias file write ";
  if (wrapper_) {
    response->success = wrapper_->saveBiases();
  }
  response->message += (response->success ? "succeeded" : "failed");
}

rcl_interfaces::msg::SetParametersResult DriverROS2::parameterChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = false;
  res.reason = "not set";
  for (const auto & p : params) {
    const auto it = biasParameters_.find(p.get_name());
    if (it != biasParameters_.end()) {
      if (wrapper_) {
        // TODO(Bernd): check value if possible and reject if out of bounds
        res.successful = true;
        res.reason = "successfully set";
      }
    } else {
      res.successful = true;
      res.reason = "ignored unknown bias";
    }
  }
  return (res);
}

void DriverROS2::onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
{
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }
  std::vector<std::string> validEvents;
  for (auto it = biasParameters_.begin(); it != biasParameters_.end(); ++it) {
    validEvents.push_back(it->first);
  }
  rclcpp::ParameterEventsFilter filter(
    event, validEvents, {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (auto & it : filter.get_events()) {
    const std::string & name = it.second->name;
    const auto bp_it = biasParameters_.find(name);
    if (bp_it != biasParameters_.end()) {
      if (wrapper_) {
        // apply bias to SDK. The driver may adjust the parameter value!
        const int oldVal = wrapper_->getBias(name);
        const int val = it.second->value.integer_value;
        if (oldVal != val) {
          const int newVal = wrapper_->setBias(name, val);
          if (val != newVal) {
            // communicate adjusted value to ROS world
            this->set_parameter(rclcpp::Parameter(name, newVal));
          }
        }
      }
    }
  }
}

std::vector<std::string> split_string(const std::string & s)
{
  std::stringstream ss(s);
  std::string tmp;
  std::vector<std::string> words;
  while (getline(ss, tmp, '.')) {
    words.push_back(tmp);
  }
  return (words);
}

void DriverROS2::addBiasParameter(
  const std::string & name, int min_val, int max_val, const std::string & desc)
{
  rcl_interfaces::msg::IntegerRange rg;
  rg.from_value = min_val;
  rg.to_value = max_val;
  rg.step = 1;
  rcl_interfaces::msg::ParameterDescriptor pd;
  pd.name = name;
  pd.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  pd.integer_range.push_back(rg);
  pd.description = desc;
  biasParameters_.insert(ParameterMap::value_type(name, pd));
}

void DriverROS2::initializeBiasParameters()
{
  addBiasParameter("bias_diff", 0, 1800, "reference level for diff_off and diff_on");
  addBiasParameter("bias_diff_off", 0, 298, "off threshold level");
  addBiasParameter("bias_diff_on", 300, 1800, "on threshold level");
  addBiasParameter("bias_fo", 0, 1800, "source follower low pass filter");
  addBiasParameter("bias_hpf", 0, 1800, "differentiator high pass filter");
  addBiasParameter("bias_pr", 0, 1800, "photoreceptor (frontend) bias");
  addBiasParameter("bias_refr", 0, 1800, "refractory time bias");
}

void DriverROS2::declareBiasParameters()
{
  initializeBiasParameters();
  for (const auto & p : biasParameters_) {
    const auto & name = p.first;
    try {
      const int defBias = wrapper_->getBias(name);
      try {
        this->declare_parameter(name, rclcpp::ParameterValue(defBias), p.second, false);
        LOG_INFO_FMT("%-20s value: %4d", name.c_str(), defBias);
      } catch (rclcpp::exceptions::InvalidParameterTypeException & e) {
        LOG_WARN("cannot declare parameter " << name << ": " << e.what());
      } catch (const std::exception & e) {
        LOG_WARN("error thrown " << e.what());
      }
    } catch (const std::runtime_error & e) {
      LOG_WARN("cannot get default bias for " << name << ", skipping it!");
    }
  }
}

void DriverROS2::secondaryReadyCallback(std_msgs::msg::Header::ConstSharedPtr msg)
{
  (void)msg;
  if (secondaryReadySub_) {
    secondaryReadySub_.reset();  // unsubscribe
  }
  LOG_INFO("secondary is ready, starting primary!");
  if (!start()) {
    LOG_ERROR("startup failed!");
    throw std::runtime_error("startup of DriverROS2 node failed!");
  }
}

bool DriverROS2::start()
{
  // must wait with initialize() until all trigger params have been set
  bool useMT;
  this->get_parameter_or("use_multithreading", useMT, true);
  double printInterval;
  this->get_parameter_or("statistics_print_interval", printInterval, 1.0);
  wrapper_->setStatisticsInterval(printInterval);
  std::string biasFile;
  this->get_parameter_or("bias_file", biasFile, std::string(""));
  if (!wrapper_->initialize(useMT, biasFile)) {
    LOG_ERROR("driver initialization failed!");
    return (false);
  }
  if (wrapper_->getSyncMode() == "secondary") {
    LOG_INFO("secondary is decoding events...");
    wrapper_->setDecodingEvents(true);
  }

  lastReadyTime_ = this->now() - readyIntervalTime_;  // move to past

  if (frameId_.empty()) {
    // default frame id to last 4 digits of serial number
    const auto sn = wrapper_->getSerialNumber();
    frameId_ = sn.substr(sn.size() - 4);
  }
  LOG_INFO("using frame id: " << frameId_);

  // ------ get other parameters from camera
  width_ = wrapper_->getWidth();
  height_ = wrapper_->getHeight();
  isBigEndian_ = check_endian::isBigEndian();

  // ------ start camera, may get callbacks from then on
  wrapper_->startCamera(this);

  declareBiasParameters();
  callbackHandle_ = this->add_on_set_parameters_callback(
    std::bind(&DriverROS2::parameterChanged, this, std::placeholders::_1));
  parameterSubscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    std::bind(&DriverROS2::onParameterEvent, this, std::placeholders::_1));

  saveBiasesService_ = this->create_service<std_srvs::srv::Trigger>(
    "save_biases",
    std::bind(&DriverROS2::saveBiases, this, std::placeholders::_1, std::placeholders::_2));
  return (true);
}

bool DriverROS2::stop()
{
  if (wrapper_) {
    return (wrapper_->stop());
  }
  return false;
}

static MetavisionWrapper::HardwarePinConfig get_hardware_pin_config(rclcpp::Node * node)
{
  MetavisionWrapper::HardwarePinConfig config;
  // builds map, e.g:
  // config["evc3a_plugin_gen31"]["external"] = 0
  // config["evc3a_plugin_gen31"]["loopback"] = 6
  const auto params = node->list_parameters({"prophesee_pin_config"}, 10 /* 10 deep */);
  for (const auto name : params.names) {
    auto a = split_string(name);
    if (a.size() != 3) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "invalid pin config found: " << name);
    } else {
      int64_t pin;
      node->get_parameter(name, pin);
      auto it_bool = config.insert({a[1], std::map<std::string, int>()});
      it_bool.first->second.insert({a[2], static_cast<int>(pin)});
    }
  }
  return (config);
}

void DriverROS2::configureWrapper(const std::string & name)
{
  wrapper_ = std::make_shared<MetavisionWrapper>(name);
  std::string sn;
  this->get_parameter_or("serial", sn, std::string(""));
  wrapper_->setSerialNumber(sn);
  std::string fromFile;
  this->get_parameter_or("from_file", fromFile, std::string(""));
  wrapper_->setFromFile(fromFile);
  std::string syncMode;
  this->get_parameter_or("sync_mode", syncMode, std::string("standalone"));
  wrapper_->setSyncMode(syncMode);
  LOG_INFO("sync mode: " << syncMode);
  std::vector<int64_t> roi_long;
  this->get_parameter_or("roi", roi_long, std::vector<int64_t>());
  std::vector<int> r(roi_long.begin(), roi_long.end());
  if (!r.empty()) {
    LOG_INFO("using ROI with " << (r.size() / 4) << " rectangle(s)");
    for (size_t i = 0; i < r.size() / 4; i += 4) {
      LOG_INFO(r[4 * i] << " " << r[4 * i + 1] << " " << r[4 * i + 2] << " " << r[4 * i + 3]);
    }
  }
  wrapper_->setROI(r);
  std::string tInMode;
  this->get_parameter_or("trigger_in_mode", tInMode, std::string("disabled"));
  if (tInMode != "disabled") {
    LOG_INFO("trigger in mode:        " << tInMode);
  }
  // disabled, enabled, loopback
  wrapper_->setExternalTriggerInMode(tInMode);
  // disabled, enabled
  std::string tOutMode;
  this->get_parameter_or("trigger_out_mode", tOutMode, std::string("disabled"));
  int64_t tOutPeriod;
  this->get_parameter_or("trigger_out_period", tOutPeriod, 100000L);  // trigger out period in usec
  double tOutCycle;
  this->get_parameter_or("trigger_duty_cycle", tOutCycle, 0.5);  // fraction of cycle trigger HIGH
  if (tOutMode != "disabled") {
    LOG_INFO("trigger out mode:       " << tOutMode);
    LOG_INFO("trigger out period:     " << tOutPeriod);
    LOG_INFO("trigger out duty cycle: " << tOutCycle);
  }
  wrapper_->setExternalTriggerOutMode(tOutMode, tOutPeriod, tOutCycle);

  // disabled, enabled, na
  std::string ercMode;  // Event Rate Controller Mode
  this->get_parameter_or("erc_mode", ercMode, std::string("na"));
  int ercRate;  // Event Rate Controller Rate
  this->get_parameter_or("erc_rate", ercRate, 100000000);
  wrapper_->setEventRateController(ercMode, ercRate);

  if (wrapper_->triggerActive()) {
    wrapper_->setHardwarePinConfig(get_hardware_pin_config(this));
  }
}

void DriverROS2::rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end)
{
  if (eventPub_->get_subscription_count() > 0) {
    if (!msg_) {
      msg_.reset(new EventArrayMsg());
      msg_->header.frame_id = frameId_;
      msg_->time_base = 0;  // not used here
      msg_->encoding = encoding_;
      msg_->seq = seq_++;
      msg_->width = width_;
      msg_->height = height_;
      msg_->header.stamp = rclcpp::Time(t, RCL_SYSTEM_TIME);
      msg_->events.reserve(reserveSize_);
    }
    const size_t n = end - start;
    auto & events = msg_->events;
    const size_t oldSize = events.size();
    resize_hack(events, oldSize + n);
    memcpy(reinterpret_cast<void *>(events.data() + oldSize), start, n);

    if (t - lastMessageTime_ > messageThresholdTime_ || events.size() > messageThresholdSize_) {
      reserveSize_ = std::max(reserveSize_, events.size());
      eventPub_->publish(std::move(msg_));
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

void DriverROS2::eventCDCallback(
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
    // decoding
    LOG_INFO("secondary sees primary up!");
    wrapper_->setDecodingEvents(false);
  }
}

}  // namespace metavision_ros_driver

RCLCPP_COMPONENTS_REGISTER_NODE(metavision_ros_driver::DriverROS2)
