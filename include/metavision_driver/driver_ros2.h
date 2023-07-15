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

#ifndef METAVISION_DRIVER__DRIVER_ROS2_H_
#define METAVISION_DRIVER__DRIVER_ROS2_H_

#include <event_camera_msgs/msg/event_packet.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "metavision_driver/bias_parameter.h"
#include "metavision_driver/callback_handler.h"
#include "metavision_driver/resize_hack.h"

namespace metavision_driver
{
class MetavisionWrapper;  // forward decl

class DriverROS2 : public rclcpp::Node, public CallbackHandler
{
  using EventPacketMsg = event_camera_msgs::msg::EventPacket;
  using Trigger = std_srvs::srv::Trigger;

public:
  explicit DriverROS2(const rclcpp::NodeOptions & options);
  ~DriverROS2();

  // ---------------- inherited from CallbackHandler -----------
  void rawDataCallback(uint64_t t, const uint8_t * start, const uint8_t * end) override;
  void eventCDCallback(
    uint64_t t, const Metavision::EventCD * begin, const Metavision::EventCD * end) override;
  // ---------------- end of inherited  -----------

private:
  // service call to dump biases
  void saveBiases(
    const std::shared_ptr<Trigger::Request> request,
    const std::shared_ptr<Trigger::Response> response);

  // related to dynanmic config (runtime parameter update)
  rcl_interfaces::msg::SetParametersResult parameterChanged(
    const std::vector<rclcpp::Parameter> & params);
  void onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);
  void addBiasParameter(const std::string & n, const BiasParameter & bp);
  void initializeBiasParameters(const std::string & sensorVersion);
  void declareBiasParameters(const std::string & sensorVersion);

  // misc helper functions
  void start();
  bool stop();
  void configureWrapper(const std::string & name);

  // ------------------------  variables ------------------------------
  std::shared_ptr<MetavisionWrapper> wrapper_;
  int width_;   // image width
  int height_;  // image height
  bool isBigEndian_;
  std::string frameId_;
  std::string encoding_;
  uint64_t seq_{0};        // sequence number
  size_t reserveSize_{0};  // recommended reserve size
  uint64_t lastMessageTime_{0};
  uint64_t messageThresholdTime_{0};  // threshold time for sending message
  size_t messageThresholdSize_{0};    // threshold size for sending message
  EventPacketMsg::UniquePtr msg_;
  rclcpp::Publisher<EventPacketMsg>::SharedPtr eventPub_;
  // ------ related to sync
  rclcpp::Service<Trigger>::SharedPtr secondaryReadyServer_;
  rclcpp::TimerBase::SharedPtr oneOffTimer_;
  // ------ related to dynamic config and services
  typedef std::map<std::string, rcl_interfaces::msg::ParameterDescriptor> ParameterMap;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callbackHandle_;
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent, std::allocator<void>>>
    parameterSubscription_;
  ParameterMap biasParameters_;
  rclcpp::Service<Trigger>::SharedPtr saveBiasesService_;
};
}  // namespace metavision_driver
#endif  // METAVISION_DRIVER__DRIVER_ROS2_H_
