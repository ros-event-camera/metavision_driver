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

#ifndef METAVISION_ROS_DRIVER__DRIVER_ROS2_H_
#define METAVISION_ROS_DRIVER__DRIVER_ROS2_H_

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "metavision_ros_driver/event_publisher_base.h"
#include "metavision_ros_driver/image_updater.h"
#include "metavision_ros_driver/synchronizer.h"

namespace metavision_ros_driver
{
class MetavisionWrapper;  // forward decl

class DriverROS2 : public rclcpp::Node, public Synchronizer
{
public:
  explicit DriverROS2(const rclcpp::NodeOptions & options);
  ~DriverROS2();
  // ----------- from synchronizer ---------
  bool sendReadyMessage() override;

private:
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  // service call to dump biases
  void saveBiases(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // related to dynanmic config (runtime parameter update)
  rcl_interfaces::msg::SetParametersResult parameterChanged(
    const std::vector<rclcpp::Parameter> & params);
  void onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);
  void addBiasParameter(const std::string & n, int min_v, int max_v, const std::string & desc);
  void initializeBiasParameters();
  void declareBiasParameters();

  // for primary sync
  void secondaryReadyCallback(std_msgs::msg::Header::ConstSharedPtr msg);

  // functions related to frame and camerainfo publishing
  void subscriptionCheckTimerExpired();
  void frameTimerExpired();
  void startNewImage();

  // misc helper functions
  bool start();
  bool stop();
  void makeEventPublisher();
  void configureWrapper(const std::string & name);

  // ------------------------  variables ------------------------------
  typedef std::map<std::string, rcl_interfaces::msg::ParameterDescriptor> ParameterMap;

  std::shared_ptr<MetavisionWrapper> wrapper_;
  std::shared_ptr<EventPublisherBase> eventPub_;
  std::string frameId_;

  // ------ related to sync
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr secondaryReadyPub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr secondaryReadySub_;
  rclcpp::Duration readyIntervalTime_;  // frequency of publishing ready messages
  rclcpp::Time lastReadyTime_;          // last time ready message was published

  // ------ related to dynamic config and services
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callbackHandle_;
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent, std::allocator<void>>>
    parameterSubscription_;
  ParameterMap biasParameters_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr saveBiasesService_;

  // ------ related to rendered image publishing
  ImageUpdater imageUpdater_;
  double fps_;
  rclcpp::TimerBase::SharedPtr frameTimer_;
  image_transport::Publisher imagePub_;
  sensor_msgs::msg::Image imageMsgTemplate_;
  rclcpp::TimerBase::SharedPtr subscriptionCheckTimer_;

  // -------- related to camerainfo
  std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  rclcpp::Publisher<CameraInfo>::SharedPtr cameraInfoPub_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
};
}  // namespace metavision_ros_driver
#endif  // METAVISION_ROS_DRIVER__DRIVER_ROS2_H_
