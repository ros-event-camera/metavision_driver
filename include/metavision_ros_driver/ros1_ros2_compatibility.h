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

#ifndef METAVISION_ROS_DRIVER__ROS1_ROS2_COMPATIBILITY_H_
#define METAVISION_ROS_DRIVER__ROS1_ROS2_COMPATIBILITY_H_

#ifdef USING_ROS_1

#include <ros/ros.h>

#define GENERIC_ROS_NODE_TYPE ros::NodeHandle

#define GENERIC_ROS_MESSAGE_TYPE(X, Y) X ::Y

#define GENERIC_ROS_GET_NODE_NAME(NODE) (ros::this_node::getName())

#define GENERIC_ROS_NSEC_NOW(NODE) (ros::Time::now().toNSec())

#define GENERIC_ROS_CREATE_PUBLISHER(NODE, TYPE, TOPIC, QSIZE) (NODE)->advertise<TYPE>(TOPIC, QSIZE)

#define GENERIC_ROS_NSEC_FROM_STAMP(STAMP) (STAMP).toNSec();

#define GENERIC_ROS_SYSTEM_TIME_FROM_NSEC(NSEC) (ros::Time().fromNSec(NSEC))

#define GENERIC_ROS_OK() (ros::ok())

#define GENERIC_ROS_PUBLISHER(TYPE) ros::Publisher

#define GENERIC_ROS_SUBSCRIPTION_COUNT(PUB) ((PUB).getNumSubscribers())

#define GENERIC_ROS_PUBLISH(PUB, MSG) ((PUB).publish(std::move((MSG))))

#define GENERIC_ROS_MSG_PTR(MSG) MSG ::Ptr

namespace metavision_ros_driver
{
typedef ros::Time GenericRosTime;
}  // namespace metavision_ros_driver

#else  // end of ROS1 definitions

//
// ------------------------ ROS2 definitions -------------------------
//
#include <rclcpp/rclcpp.hpp>

#define GENERIC_ROS_NODE_TYPE rclcpp::Node

#define GENERIC_ROS_MESSAGE_TYPE(X, Y) X ::msg::Y

#define GENERIC_ROS_GET_NODE_NAME(NODE) ((NODE)->get_name())

#define GENERIC_ROS_NSEC_NOW(NODE) (node_->now().nanoseconds())

#define GENERIC_ROS_CREATE_PUBLISHER(NODE, TYPE, TOPIC, QSIZE) \
  (NODE)->create_publisher<TYPE>(                              \
    std::string("~/") + TOPIC,                                 \
    rclcpp::QoS(rclcpp::KeepLast(QSIZE)).best_effort().durability_volatile())

#define GENERIC_ROS_NSEC_FROM_STAMP(STAMP) (rclcpp::Time(STAMP).nanoseconds())

#define GENERIC_ROS_SYSTEM_TIME_FROM_NSEC(NSEC) (rclcpp::Time(NSEC, RCL_SYSTEM_TIME))

#define GENERIC_ROS_OK() (rclcpp::ok())

#define GENERIC_ROS_PUBLISHER(TYPE) typename rclcpp::Publisher<MsgType>::SharedPtr

#define GENERIC_ROS_SUBSCRIPTION_COUNT(PUB) ((PUB)->get_subscription_count())

#define GENERIC_ROS_PUBLISH(PUB, MSG) ((PUB)->publish(std::move((MSG))))

#define GENERIC_ROS_MSG_PTR(MSG) MSG ::UniquePtr

namespace metavision_ros_driver
{
typedef rclcpp::Time GenericRosTime;
}  // namespace metavision_ros_driver

#endif  // end of ROS2 definitions

#endif  // METAVISION_ROS_DRIVER__ROS1_ROS2_COMPATIBILITY_H_
