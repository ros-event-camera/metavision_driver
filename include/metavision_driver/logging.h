// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef METAVISION_DRIVER__LOGGING_H_
#define METAVISION_DRIVER__LOGGING_H_
#ifdef USING_ROS_1
#include <ros/ros.h>
#define LOG_INFO_NAMED(...)       \
  {                               \
    ROS_INFO_STREAM(__VA_ARGS__); \
  }
#define LOG_WARN_NAMED(...)       \
  {                               \
    ROS_WARN_STREAM(__VA_ARGS__); \
  }
#define LOG_ERROR_NAMED(...)       \
  {                                \
    ROS_ERROR_STREAM(__VA_ARGS__); \
  }

#define LOG_INFO_NAMED_FMT(...) \
  {                             \
    ROS_INFO(__VA_ARGS__);      \
  }
#define LOG_WARN_NAMED_FMT(...) \
  {                             \
    ROS_WARN(__VA_ARGS__);      \
  }
#define LOG_ERROR_NAMED_FMT(...) \
  {                              \
    ROS_ERROR(__VA_ARGS__);      \
  }

#define LOG_INFO_NAMED_FMT_THROTTLE(...) \
  {                                      \
    ROS_INFO_THROTTLE(__VA_ARGS__);      \
  }
#define LOG_WARN_NAMED_FMT_THROTTLE(...) \
  {                                      \
    ROS_WARN_THROTTLE(__VA_ARGS__);      \
  }
#define LOG_ERROR_NAMED_FMT_THROTTLE(...) \
  {                                       \
    ROS_ERROR_THROTTLE(__VA_ARGS__);      \
  }

#else

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#define BOMB_OUT(...)                               \
  {                                                 \
    RCLCPP_ERROR_STREAM(get_logger(), __VA_ARGS__); \
    std::stringstream SS;                           \
    SS << __VA_ARGS__;                              \
    throw(std::runtime_error(SS.str()));            \
  }
#define BOMB_OUT_NODE(...)                                 \
  {                                                        \
    RCLCPP_ERROR_STREAM(node_->get_logger(), __VA_ARGS__); \
    std::stringstream SS;                                  \
    SS << __VA_ARGS__;                                     \
    throw(std::runtime_error(SS.str()));                   \
  }
#define BOMB_OUT_CERR(...)                 \
  {                                        \
    std::cerr << __VA_ARGS__ << std::endl; \
    std::stringstream SS;                  \
    SS << __VA_ARGS__;                     \
    throw(std::runtime_error(SS.str()));   \
  }
#define LOG_INFO_NAMED(...)                                           \
  {                                                                   \
    RCLCPP_INFO_STREAM(rclcpp::get_logger(loggerName_), __VA_ARGS__); \
  }
#define LOG_WARN_NAMED(...)                                           \
  {                                                                   \
    RCLCPP_WARN_STREAM(rclcpp::get_logger(loggerName_), __VA_ARGS__); \
  }
#define LOG_ERROR_NAMED(...)                                           \
  {                                                                    \
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(loggerName_), __VA_ARGS__); \
  }

#define LOG_INFO(...)                              \
  {                                                \
    RCLCPP_INFO_STREAM(get_logger(), __VA_ARGS__); \
  }
#define LOG_WARN(...)                              \
  {                                                \
    RCLCPP_WARN_STREAM(get_logger(), __VA_ARGS__); \
  }
#define LOG_ERROR(...)                              \
  {                                                 \
    RCLCPP_ERROR_STREAM(get_logger(), __VA_ARGS__); \
  }

#define LOG_INFO_NODE(...)                                \
  {                                                       \
    RCLCPP_INFO_STREAM(node_->get_logger(), __VA_ARGS__); \
  }
#define LOG_WARN_NODE(...)                                \
  {                                                       \
    RCLCPP_WARN_STREAM(node_->get_logger(), __VA_ARGS__); \
  }
#define LOG_ERROR_NODE(...)                                \
  {                                                        \
    RCLCPP_ERROR_STREAM(node_->get_logger(), __VA_ARGS__); \
  }

#define LOG_INFO_FMT(...)                   \
  {                                         \
    RCLCPP_INFO(get_logger(), __VA_ARGS__); \
  }
#define LOG_WARN_FMT(...)                   \
  {                                         \
    RCLCPP_WARN(get_logger(), __VA_ARGS__); \
  }
#define LOG_ERROR_FMT(...)                   \
  {                                          \
    RCLCPP_ERROR(get_logger(), __VA_ARGS__); \
  }

#define LOG_INFO_NODE_FMT(...)                     \
  {                                                \
    RCLCPP_INFO(node_->get_logger(), __VA_ARGS__); \
  }
#define LOG_WARN_NODE_FMT(...)                     \
  {                                                \
    RCLCPP_WARN(node_->get_logger(), __VA_ARGS__); \
  }
#define LOG_ERROR_NODE_FMT(...)                     \
  {                                                 \
    RCLCPP_ERROR(node_->get_logger(), __VA_ARGS__); \
  }
#define LOG_INFO_NAMED_FMT(...)                                \
  {                                                            \
    RCLCPP_INFO(rclcpp::get_logger(loggerName_), __VA_ARGS__); \
  }
#define LOG_WARN_NAMED_FMT(...)                                \
  {                                                            \
    RCLCPP_WARN(rclcpp::get_logger(loggerName_), __VA_ARGS__); \
  }
#define LOG_ERROR_NAMED_FMT(...)                                \
  {                                                             \
    RCLCPP_ERROR(rclcpp::get_logger(loggerName_), __VA_ARGS__); \
  }

#define LOG_INFO_NODE_FMT_THROTTLE(...)                                          \
  {                                                                              \
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), __VA_ARGS__); \
  }
#define LOG_WARN_NODE_FMT_THROTTLE(...)                                          \
  {                                                                              \
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), __VA_ARGS__); \
  }
#define LOG_ERROR_NODE_FMT_THROTTLE(...)                                          \
  {                                                                               \
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), __VA_ARGS__); \
  }

#endif  // USING_ROS_1

#endif  // METAVISION_DRIVER__LOGGING_H_
