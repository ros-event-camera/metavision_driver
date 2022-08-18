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

#include <dvs_msgs/msg/event_array.hpp>
#include <event_array_msgs/msg/event_array.hpp>
#include <prophesee_event_msgs/msg/event_array.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include "metavision_ros_driver/check_endian.h"
#include "metavision_ros_driver/event_publisher.h"
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
  this->get_parameter_or("frame_id", frameId_, std::string(""));
  this->get_parameter_or("fps", fps_, 25.0);

  configureWrapper(get_name());

  const rmw_qos_profile_t qosProf = rmw_qos_profile_default;

  imagePub_ = image_transport::create_publisher(this, "~/image_raw", qosProf);

  cameraInfoPub_ = this->create_publisher<CameraInfo>("~/camera_info", rclcpp::QoS(1));

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
  // Since ROS2 image transport  does not call back when subscribers come and go
  // must check by polling
  subscriptionCheckTimer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration(1, 0),
    std::bind(&DriverROS2::subscriptionCheckTimerExpired, this));
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

void DriverROS2::subscriptionCheckTimerExpired()
{
  // this silly dance is only necessary because ROS2 at this time does not support
  // callbacks when subscribers come and go
  if ((imagePub_.getNumSubscribers() || cameraInfoPub_->get_subscription_count()) && !frameTimer_) {
    // start publishing frames if there is interest in either camerainfo or image
    frameTimer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / fps_),
      std::bind(&DriverROS2::frameTimerExpired, this));
  } else if (
    (!imagePub_.getNumSubscribers() && !cameraInfoPub_->get_subscription_count()) && frameTimer_) {
    // if nobody is listening, stop publishing frames if this is currently happening
    frameTimer_->cancel();
    frameTimer_.reset();
  }

  if (imagePub_.getNumSubscribers() == 0) {
    // all subscribers are gone
    if (imageUpdater_.hasImage()) {
      // tell image updater to deallocate image
      imageUpdater_.resetImagePtr();
    }
    wrapper_->setCallbackHandler2(0);
  } else if (!imageUpdater_.hasImage()) {
    // we have subscribers but no image is being updated yet, so start doing so
    startNewImage();
    wrapper_->setCallbackHandler2(&imageUpdater_);
  }
}

void DriverROS2::frameTimerExpired()
{
  const rclcpp::Time t = this->get_clock()->now();
  // publish camerainfo if somebody is listening
  if (cameraInfoPub_->get_subscription_count() != 0) {
    cameraInfoMsg_.header.stamp = t;
    cameraInfoPub_->publish(cameraInfoMsg_);
  }
  // publish frame if available and somebody listening
  if (imagePub_.getNumSubscribers() != 0 && imageUpdater_.hasImage()) {
    // take memory managent from image updater
    sensor_msgs::msg::Image::UniquePtr updated_img = imageUpdater_.getImage();
    updated_img->header.stamp = t;
    // give memory management to imagePub_
    imagePub_.publish(std::move(updated_img));
    // start a new image
    startNewImage();
  }
}

void DriverROS2::startNewImage()
{
  sensor_msgs::msg::Image::UniquePtr img(new sensor_msgs::msg::Image(imageMsgTemplate_));
  img->data.resize(img->height * img->step, 0);  // allocate memory and set all bytes to zero
  imageUpdater_.setImage(&img);                  // event publisher will also render image now
}

bool DriverROS2::start()
{
  // must wait with initialize() until all trigger params have been set
  bool useMT;
  this->get_parameter_or("use_multithreading", useMT, true);
  double printInterval;
  this->get_parameter_or("statistics_print_interval", printInterval, 1.0);
  std::string biasFile;
  this->get_parameter_or("bias_file", biasFile, std::string(""));
  if (!wrapper_->initialize(useMT, printInterval, biasFile)) {
    LOG_ERROR("driver initialization failed!");
    return (false);
  }
  lastReadyTime_ = this->now() - readyIntervalTime_;  // move to past

  if (frameId_.empty()) {
    // default frame id to last 4 digits of serial number
    const auto sn = wrapper_->getSerialNumber();
    frameId_ = sn.substr(sn.size() - 4);
  }
  LOG_INFO("using frame id: " << frameId_);

  std::string cameraInfoURL;
  this->get_parameter_or("camerainfo_url", cameraInfoURL, std::string(""));

  infoManager_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(this, get_name(), cameraInfoURL);
  cameraInfoMsg_ = infoManager_->getCameraInfo();
  cameraInfoMsg_.header.frame_id = frameId_;
  cameraInfoMsg_.height = wrapper_->getHeight();
  cameraInfoMsg_.width = wrapper_->getWidth();

  imageMsgTemplate_.header.frame_id = frameId_;
  imageMsgTemplate_.height = wrapper_->getHeight();
  imageMsgTemplate_.width = wrapper_->getWidth();
  imageMsgTemplate_.encoding = "bgr8";
  imageMsgTemplate_.is_bigendian = check_endian::isBigEndian();
  imageMsgTemplate_.step = 3 * imageMsgTemplate_.width;

  // ------ start camera, may get callbacks from then on
  makeEventPublisher();
  wrapper_->startCamera(eventPub_.get());

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

template <typename MsgType>
static std::shared_ptr<EventPublisher<MsgType>> make_publisher(
  rclcpp::Node * node, Synchronizer * sync, const std::shared_ptr<MetavisionWrapper> & wrapper,
  const std::string & frameId, bool pubTrig, size_t eventReserveSize, double eventTimeThreshold,
  size_t triggerReserveSize, double triggerTimeThreshold)
{
  const int qs = 1000;
  typedef EventPublisher<MsgType> EventPub;
  auto pub = std::make_shared<EventPub>(node, sync, wrapper, frameId);
  pub->setupEventState(eventReserveSize, eventTimeThreshold, "events", qs);
  if (pubTrig) {
    pub->setupTriggerState(triggerReserveSize, triggerTimeThreshold, "trigger", qs);
  }
  return (pub);
}

void DriverROS2::makeEventPublisher()
{
  const bool pubTrig = wrapper_->triggerInActive();
  std::string msgType;
  this->get_parameter_or("message_type", msgType, std::string("event_array"));
  double eventTimeThreshold;
  this->get_parameter_or("event_message_time_threshold", eventTimeThreshold, 100e-6);
  LOG_INFO("event message time threshold: " << eventTimeThreshold << "s");
  double mmevs;
  this->get_parameter_or("sensor_max_mevs", mmevs, 50.0);
  const size_t eventReserveSize = static_cast<size_t>(mmevs * 1.0e6 * eventTimeThreshold);
  LOG_INFO("using event reserve size: " << eventReserveSize);
  double triggerTimeThreshold;
  this->get_parameter_or("trigger_message_time_threshold", triggerTimeThreshold, 100e-6);
  LOG_INFO("trigger message time threshold: " << triggerTimeThreshold << "s");
  double ttmf;
  this->get_parameter_or("trigger_max_freq", ttmf, 1000.0);
  const size_t triggerReserveSize = static_cast<size_t>(ttmf * triggerTimeThreshold);

  // different types of publishers depending on message type
  if (msgType == "prophesee") {
    eventPub_ = make_publisher<prophesee_event_msgs::msg::EventArray>(
      this, this, wrapper_, frameId_, pubTrig, eventReserveSize, eventTimeThreshold,
      triggerReserveSize, triggerTimeThreshold);
  } else if (msgType == "dvs") {
    eventPub_ = make_publisher<dvs_msgs::msg::EventArray>(
      this, this, wrapper_, frameId_, pubTrig, eventReserveSize, eventTimeThreshold,
      triggerReserveSize, triggerTimeThreshold);
  } else if (msgType == "event_array") {
    eventPub_ = make_publisher<event_array_msgs::msg::EventArray>(
      this, this, wrapper_, frameId_, pubTrig, eventReserveSize, eventTimeThreshold,
      triggerReserveSize, triggerTimeThreshold);
  } else {
    LOG_ERROR("invalid msg type: " << msgType);
    throw(std::runtime_error("invalid message type!"));
  }
  LOG_INFO("started driver with msg type: " << msgType);
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
  if (wrapper_->triggerActive()) {
    wrapper_->setHardwarePinConfig(get_hardware_pin_config(this));
  }
}

}  // namespace metavision_ros_driver

RCLCPP_COMPONENTS_REGISTER_NODE(metavision_ros_driver::DriverROS2)
