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

#include <dvs_msgs/EventArray.h>
#include <event_array_msgs/EventArray.h>
#include <prophesee_event_msgs/EventArray.h>

#include "metavision_ros_driver/check_endian.h"
#include "metavision_ros_driver/event_publisher.h"
#include "metavision_ros_driver/metavision_wrapper.h"

namespace metavision_ros_driver
{
namespace ph = std::placeholders;
DriverROS1::DriverROS1(ros::NodeHandle & nh) : nh_(nh)
{
  frameId_ = nh_.param<std::string>("frame_id", "");
  fps_ = nh_.param<double>("fps", 25.0);

  configureWrapper(ros::this_node::getName());

  image_transport::ImageTransport it(nh_);
  imagePub_ = it.advertise(
    "image_raw", 1, boost::bind(&DriverROS1::imageConnectCallback, this, boost::placeholders::_1),
    boost::bind(&DriverROS1::imageConnectCallback, this, boost::placeholders::_1));

  cameraInfoPub_ = nh_.advertise<CameraInfo>(
    std::string("camera_info"), (uint32_t)1,
    boost::bind(&DriverROS1::cameraInfoConnectCallback, this, boost::placeholders::_1),
    boost::bind(&DriverROS1::cameraInfoConnectCallback, this, boost::placeholders::_1));

  // create timer and stop it right away
  frameTimer_ = nh_.createTimer(ros::Duration(1.0 / fps_), &DriverROS1::frameTimerExpired, this);
  frameTimer_.stop();

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

void DriverROS1::cameraInfoConnectCallback(const ros::SingleSubscriberPublisher &)
{
  updateFrameTimer();
}

void DriverROS1::imageConnectCallback(const image_transport::SingleSubscriberPublisher &)
{
  updateFrameTimer();
}

void DriverROS1::frameTimerExpired(const ros::TimerEvent &)
{
  if (cameraInfoPub_.getNumSubscribers() == 0 && imagePub_.getNumSubscribers() == 0) {
    // shouldn't get called in this case, but just for good measure...
    updateFrameTimer();
    return;
  }
  const auto t = ros::Time::now();

  if (cameraInfoPub_.getNumSubscribers() != 0) {
    cameraInfoMsg_.header.stamp = t;
    cameraInfoPub_.publish(cameraInfoMsg_);
  }

  if (imagePub_.getNumSubscribers() != 0 && imageUpdater_.hasImage()) {
    // take memory managent from image updater
    std::unique_ptr<sensor_msgs::Image> updated_img = imageUpdater_.getImage();
    updated_img->header.stamp = t;
    // give memory management to imagePub_
    imagePub_.publish(std::move(updated_img));
    // start a new image
    startNewImage();
  }
}

void DriverROS1::updateFrameTimer()
{
  if (cameraInfoPub_.getNumSubscribers() != 0 || imagePub_.getNumSubscribers() != 0) {
    frameTimer_.start();  // no-op if already running
  } else {
    frameTimer_.stop();  // no-op if already stopped
  }

  if (imagePub_.getNumSubscribers() != 0) {
    if (!imageUpdater_.hasImage()) {
      // we have subscribers but no image is being updated yet, so start doing so
      startNewImage();
    }
    wrapper_->setCallbackHandler2(&imageUpdater_);
  } else {
    if (imageUpdater_.hasImage()) {
      imageUpdater_.resetImagePtr();  // tell image updater to deallocate image
    }
    wrapper_->setCallbackHandler2(0);
  }
}

void DriverROS1::startNewImage()
{
  std::unique_ptr<sensor_msgs::Image> img(new sensor_msgs::Image(imageMsgTemplate_));
  img->data.resize(img->height * img->step, 0);  // allocate memory and set all bytes to zero
  imageUpdater_.setImage(&img);                  // event publisher will also render image now
}

bool DriverROS1::start()
{
  if (!wrapper_->initialize(
        nh_.param<bool>("use_multithreading", false),
        nh_.param<double>("statistics_print_interval", 1.0),
        nh_.param<std::string>("bias_file", ""))) {
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

  infoManager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    nh_, nh_.param<std::string>("camerainfo_url", ros::this_node::getName()));
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

// -- static helper to avoid templating DriverROS1 class
template <typename MsgType>
static std::shared_ptr<EventPublisher<MsgType>> make_publisher(
  ros::NodeHandle & node, Synchronizer * sync, const std::shared_ptr<MetavisionWrapper> & wrapper,
  const std::string & frameId, bool pubTrig, size_t eventReserveSize, double eventTimeThreshold,
  size_t triggerReserveSize, double triggerTimeThreshold)
{
  const int qs = 1000;
  auto pub = std::make_shared<EventPublisher<MsgType>>(&node, sync, wrapper, frameId);
  pub->setupEventState(eventReserveSize, eventTimeThreshold, "events", qs);
  pub->setupRawState(eventReserveSize, eventTimeThreshold, "packets", qs);
  if (pubTrig) {
    pub->setupTriggerState(triggerReserveSize, triggerTimeThreshold, "trigger", qs);
  }
  return (pub);
}

void DriverROS1::makeEventPublisher()
{
  const bool pubTrig = wrapper_->triggerInActive();
  const std::string msgType = nh_.param<std::string>("message_type", "event_array");

  const double eventTimeThreshold = nh_.param<double>("event_message_time_threshold", 100e-6);
  ROS_INFO_STREAM("event message time threshold: " << eventTimeThreshold);
  const size_t eventReserveSize =
    static_cast<size_t>(nh_.param<double>("sensor_max_mevs", 50.0) * 1.0e6 * eventTimeThreshold);
  ROS_INFO_STREAM("event message reserve size: " << eventReserveSize);

  const double triggerTimeThreshold = nh_.param<double>("trigger_message_time_threshold", 100e-6);
  const size_t triggerReserveSize =
    static_cast<size_t>(nh_.param<double>("trigger_max_freq", 1000.0) * triggerTimeThreshold);

  if (pubTrig) {
    ROS_INFO_STREAM("trigger message time threshold: " << triggerTimeThreshold);
    ROS_INFO_STREAM("trigger message reserve size: " << triggerReserveSize);
  }

  // different types of publishers depending on message type
  if (msgType == "prophesee") {
    eventPub_ = make_publisher<prophesee_event_msgs::EventArray>(
      nh_, this, wrapper_, frameId_, pubTrig, eventReserveSize, eventTimeThreshold,
      triggerReserveSize, triggerTimeThreshold);
  } else if (msgType == "dvs") {
    eventPub_ = make_publisher<dvs_msgs::EventArray>(
      nh_, this, wrapper_, frameId_, pubTrig, eventReserveSize, eventTimeThreshold,
      triggerReserveSize, triggerTimeThreshold);
  } else if (msgType == "event_array") {
    eventPub_ = make_publisher<event_array_msgs::EventArray>(
      nh_, this, wrapper_, frameId_, pubTrig, eventReserveSize, eventTimeThreshold,
      triggerReserveSize, triggerTimeThreshold);
  } else {
    ROS_ERROR_STREAM("invalid msg type: " << msgType);
    throw(std::runtime_error("invalid message type!"));
  }
  ROS_INFO_STREAM("started driver with msg type: " << msgType);
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
    nh_.param<std::string>("erc_mode", "na"), // Event Rate Controller Mode
    nh_.param<int>("erc_rate", 100000000));  // Event Rate Controller Rate

  // Get information on external pin configuration per hardware setup
  if (wrapper_->triggerActive()) {
    wrapper_->setHardwarePinConfig(get_hardware_pin_config(nh_, ros::this_node::getName()));
  }
}

}  // namespace metavision_ros_driver
