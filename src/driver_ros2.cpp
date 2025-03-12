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

#include "metavision_driver/driver_ros2.h"

#include <chrono>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <map>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

#include "metavision_driver/check_endian.h"
#include "metavision_driver/logging.h"
#include "metavision_driver/metavision_wrapper.h"

namespace metavision_driver
{
DriverROS2::DriverROS2(const rclcpp::NodeOptions & options)
: Node(
    "metavision_driver",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
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
  this->get_parameter_or("send_queue_size", qs, 1000);
  eventPub_ = this->create_publisher<EventPacketMsg>(
    "~/events", rclcpp::QoS(rclcpp::KeepLast(qs)).best_effort().durability_volatile());

  if (wrapper_->getSyncMode() == "primary") {
    // delay primary until secondary is up and running
    // need to delay this to finish the constructor and release the thread
    oneOffTimer_ = this->create_wall_timer(std::chrono::seconds(1), [=]() {
      oneOffTimer_->cancel();
      auto client = this->create_client<Trigger>("~/ready");
      while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted!");
          throw std::runtime_error("interrupted!");
        }
        RCLCPP_INFO(this->get_logger(), "primary waiting for secondary...");
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "secondary is up!");
      start();  // only now can this be started
    });
  } else if (wrapper_->getSyncMode() == "secondary") {
    start();
    // creation of server signals to primary that we are ready.
    // We don't handle the callback
    secondaryReadyServer_ = this->create_service<Trigger>(
      "~/ready",
      [=](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response>) {});
  } else {
    // standalone mode
    start();
  }
}

DriverROS2::~DriverROS2()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

void DriverROS2::saveBiases(
  const std::shared_ptr<Trigger::Request> request,
  const std::shared_ptr<Trigger::Response> response)
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
  // need to make copy to work around Foxy API
  auto ev = std::make_shared<rcl_interfaces::msg::ParameterEvent>(*event);
  rclcpp::ParameterEventsFilter filter(
    ev, validEvents, {rclcpp::ParameterEventsFilter::EventType::CHANGED});
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

void DriverROS2::addBiasParameter(const std::string & name, const BiasParameter & bp)
{
  if (wrapper_->hasBias(name)) {
    rcl_interfaces::msg::IntegerRange rg;
    rg.from_value = bp.minVal;
    rg.to_value = bp.maxVal;
    rg.step = 1;
    rcl_interfaces::msg::ParameterDescriptor pd;
    pd.name = name;
    pd.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    pd.integer_range.push_back(rg);
    pd.description = bp.info;
    biasParameters_.insert(ParameterMap::value_type(name, pd));
  }
}

void DriverROS2::initializeBiasParameters(const std::string & sensorVersion)
{
  const auto map = BiasParameter::getAll(sensorVersion);
  if (map.empty()) {
    LOG_WARN("unknown sensor version " << sensorVersion << ", disabling tunable biases");
  } else {
    for (const auto & p : map) {  // loop through params
      addBiasParameter(p.first, p.second);
    }
  }
}

void DriverROS2::declareBiasParameters(const std::string & sensorVersion)
{
  initializeBiasParameters(sensorVersion);
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

void DriverROS2::start()
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
    throw std::runtime_error("driver initialization failed!");
  }
  if (wrapper_->getSyncMode() == "secondary") {
    // For Gen3 the secondary will produce data with time stamps == 0
    // until it sees a clock signal. To filter out those events we
    // decode the events and wait until the time stamps are good.
    // Only then do we switch to "raw" mode.
    LOG_INFO("secondary is decoding events...");
    wrapper_->setDecodingEvents(true);
  }

  if (frameId_.empty()) {
    // default frame id to last 4 digits of serial number
    const auto sn = wrapper_->getSerialNumber();
    frameId_ = (sn.size() > 4) ? sn.substr(sn.size() - 4) : std::string("event_cam");
  }
  LOG_INFO("using frame id: " << frameId_);

  if (wrapper_->getEncodingFormat() != encoding_) {
    LOG_ERROR(
      "encoding mismatch, camera has: " << wrapper_->getEncodingFormat() << ", but expecting "
                                        << encoding_);
    throw std::runtime_error("encoding mismatch!");
  }

  // ------ get other parameters from camera
  width_ = wrapper_->getWidth();
  height_ = wrapper_->getHeight();
  isBigEndian_ = check_endian::isBigEndian();

  // ------ start camera, may get callbacks from then on
  wrapper_->startCamera(this);

  if (wrapper_->getFromFile().empty()) {
    declareBiasParameters(wrapper_->getSensorVersion());
    callbackHandle_ = this->add_on_set_parameters_callback(
      std::bind(&DriverROS2::parameterChanged, this, std::placeholders::_1));
    parameterSubscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
      this->get_node_topics_interface(),
      std::bind(&DriverROS2::onParameterEvent, this, std::placeholders::_1));

    saveBiasesService_ = this->create_service<Trigger>(
      "save_biases",
      std::bind(&DriverROS2::saveBiases, this, std::placeholders::_1, std::placeholders::_2));
  }
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
  for (const auto & name : params.names) {
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
  bool trailFilter;
  this->get_parameter_or("trail_filter", trailFilter, false);
  std::string trailFilterType;
  this->get_parameter_or("trail_filter_type", trailFilterType, std::string("trail"));
  int trailFilterThreshold;
  this->get_parameter_or("trail_filter_threshold", trailFilterThreshold, 0);
  if (trailFilter) {
    LOG_INFO(
      "Using tail filter in " << trailFilterType << " mode with threshold "
                              << trailFilterThreshold);
    wrapper_->setTrailFilter(
      trailFilterType, static_cast<uint32_t>(trailFilterThreshold), trailFilter);
  }
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
  int mipiFramePeriod{-1};
  this->get_parameter_or("mipi_frame_period", mipiFramePeriod, -1);
  wrapper_->setMIPIFramePeriod(mipiFramePeriod);
}

void DriverROS2::rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end)
{
  if (eventPub_->get_subscription_count() > 0) {
    if (!msg_) {
      msg_.reset(new EventPacketMsg());
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
      wrapper_->updateBytesSent(events.size());
      eventPub_->publish(std::move(msg_));
      lastMessageTime_ = t;
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
    LOG_INFO("secondary sees primary up!");
    wrapper_->setDecodingEvents(false);
  }
}

}  // namespace metavision_driver

RCLCPP_COMPONENTS_REGISTER_NODE(metavision_driver::DriverROS2)
