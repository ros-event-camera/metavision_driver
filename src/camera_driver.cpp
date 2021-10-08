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

#include "metavision_ros_driver/camera_driver.h"

#include <functional>
#include <iostream>
#include <rclcpp_components/register_node_macro.hpp>

#include "metavision_ros_driver/logging.h"
#include "metavision_ros_driver/metavision_wrapper.h"

namespace metavision_ros_driver
{
CameraDriver::CameraDriver(const rclcpp::NodeOptions & options)
: Node("metavision_ros_driver", options), messageTimeThreshold_(0, 0)
{
  bool status = start();
  if (!status) {
    LOG_ERROR("startup failed!");
    throw std::runtime_error("startup of CameraDriver node failed!");
  }
}

CameraDriver::~CameraDriver()
{
  stop();
  wrapper_.reset();  // invoke destructor
}

void CameraDriver::initializeBiasParameters()
{
  addBiasParameter("bias_diff", 0, 1800, "reference level for diff_off and diff_on");
  addBiasParameter("bias_diff_off", 0, 298, "off threshold level");
  addBiasParameter("bias_diff_on", 300, 1800, "on threshold level");
  addBiasParameter("bias_fo", 0, 1800, "source follower low pass filter");
  addBiasParameter("bias_hpf", 0, 1800, "differentiator high pass filter");
  addBiasParameter("bias_pr", 0, 1800, "photoreceptor (frontend) bias");
  addBiasParameter("bias_refr", 0, 1800, "refractory time bias");
}

void CameraDriver::addBiasParameter(
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

void CameraDriver::declareBiasParameters()
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

bool CameraDriver::stop()
{
  if (wrapper_) {
    return (wrapper_->stop());
  }
  return false;
}

rcl_interfaces::msg::SetParametersResult CameraDriver::parameterChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = false;
  res.reason = "not set";
  for (const auto & p : params) {
    const auto it = biasParameters_.find(p.get_name());
    if (it != biasParameters_.end()) {
      if (wrapper_) {
        const int newVal = wrapper_->setBias(p.get_name(), p.as_int());
        if (newVal == p.as_int()) {
          res.successful = true;
          res.reason = "successfully set";
        } else {
          LOG_INFO(p.get_name() << " was adjusted from  " << p.as_int() << " to " << newVal);
          changeTimer_ = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration::from_nanoseconds(1000000), [=]() {
              // LOG_INFO("setting back: " << p.get_name() << " to " << newVal);
              changeTimer_->cancel();
              this->set_parameter(rclcpp::Parameter(p.get_name(), newVal));
            });
        }
      }
    } else {
      res.successful = true;
      res.reason = "ignored";
    }
  }
  return (res);
}

bool CameraDriver::start()
{
  cameraInfoURL_ = this->declare_parameter<std::string>("camerainfo_url", "");
  frameId_ = this->declare_parameter<std::string>("frame_id", "");
  messageTimeThreshold_ = rclcpp::Duration::from_nanoseconds(
    (uint64_t)(1e9 * this->declare_parameter<double>("message_time_threshold", 10e-6)));
  callbackHandle_ = this->add_on_set_parameters_callback(
    std::bind(&CameraDriver::parameterChanged, this, std::placeholders::_1));

  pub_ = create_publisher<prophesee_event_msgs::msg::EventArray>(
    "~/events", this->declare_parameter<int>("send_queue_size", 1000));

  wrapper_ = std::make_shared<MetavisionWrapper>(this);

  if (!wrapper_->initialize(
        this->declare_parameter<bool>("use_multithreading", true),
        this->declare_parameter<double>("statistics_print_interval", 1.0),
        this->declare_parameter<std::string>("bias_file", ""))) {
    LOG_ERROR("driver initialization failed!");
    return (false);
  }
  width_ = wrapper_->getWidth();
  height_ = wrapper_->getHeight();
  if (frameId_.empty()) {
    // default frame id to last 4 digits of serial number
    const auto sn = wrapper_->getSerialNumber();
    frameId_ = sn.substr(sn.size() - 4);
  }
  LOG_INFO("using frame id: " << frameId_);

  infoManager_ =
    std::make_shared<camera_info_manager::CameraInfoManager>(this, get_name(), cameraInfoURL_);
  cameraInfoMsg_ = infoManager_->getCameraInfo();
  cameraInfoMsg_.header.frame_id = frameId_;
  declareBiasParameters();
  saveBiasesService_ = this->create_service<std_srvs::srv::Trigger>(
    "save_biases",
    std::bind(&CameraDriver::saveBiases, this, std::placeholders::_1, std::placeholders::_2));
  return (true);
}

void CameraDriver::saveBiases(
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

void CameraDriver::publish(const Metavision::EventCD * start, const Metavision::EventCD * end)
{
  if (t0_ == 0) {
    t0_ = this->now().nanoseconds();
  }
  if (!msg_) {  // must allocate new message
    msg_.reset(new prophesee_event_msgs::msg::EventArray());
    msg_->header.frame_id = frameId_;
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
  int eventCount[2] = {0, 0};
  for (unsigned int i = 0; i < n; i++) {
    const auto & e_src = start[i];
    auto & e_trg = events[i + old_size];
    e_trg.x = e_src.x;
    e_trg.y = e_src.y;
    e_trg.polarity = e_src.p;
    e_trg.ts = rclcpp::Time(t0_ + (uint64_t)(e_src.t * 1e3), RCL_SYSTEM_TIME);
    eventCount[e_src.p]++;
  }
  wrapper_->updateEventCount(0, eventCount[0]);
  wrapper_->updateEventCount(1, eventCount[1]);
  const rclcpp::Time t_msg(msg_->events.begin()->ts);
  const rclcpp::Time t_last(msg_->events.rbegin()->ts);
  if (t_last > t_msg + messageTimeThreshold_) {
    msg_->header.stamp = t_msg;
    // the move() will reset msg_ and transfer ownership
    pub_->publish(std::move(msg_));
    wrapper_->updateEventsSent(events.size());
    wrapper_->updateMsgsSent(1);
  }
}

}  // namespace metavision_ros_driver

RCLCPP_COMPONENTS_REGISTER_NODE(metavision_ros_driver::CameraDriver)
