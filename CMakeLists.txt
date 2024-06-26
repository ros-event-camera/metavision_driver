#
# Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(metavision_driver)

include(FetchContent)

find_package(MetavisionSDK COMPONENTS driver QUIET)

# if metavision sdk is not present, download it into the build directory
set(MUST_INSTALL_METAVISION FALSE)

if(NOT MetavisionSDK_FOUND)
  message(STATUS "metavision SDK is not installed, must build it")
  # must set various variables for OpenEB *before* fetching openEB.
  # CMAKE_ARGS seems to not work for FetchContent_Declare()
  # set(COMPILE_3DVIEW OFF CACHE INTERNAL "Build 3d viewer")
  set(COMPILE_PLAYER OFF CACHE INTERNAL "Build player")
  set(COMPILE_PYTHON3_BINDINGS OFF CACHE INTERNAL "build python3 bindings")
  set(UDEV_RULES_SYSTEM_INSTALL OFF CACHE INTERNAL "install udev rules")

# The following line must have zero indent. It disables the cmake linter
# lint_cmake: -readability/wonkycase
  FetchContent_Declare(
    metavision
    GIT_REPOSITORY https://github.com/ros-event-camera/openeb.git
    GIT_TAG   4.2.0-ros)

  FetchContent_MakeAvailable(metavision)
  message(STATUS "metavision SDK fetched and made available")
  # do this to avoid the "install" target being run on the metavision sdk
  if(IS_DIRECTORY "${metavision_SOURCE_DIR}")
    set_property(DIRECTORY ${metavision_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL YES)
  endif()

  set(MUST_INSTALL_METAVISION TRUE)
else()
  message(STATUS "metavision SDK is installed, not building it")
endif()

#
# figure out ROS1 vs ROS2
#
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION "$ENV{ROS_VERSION}")
  message(STATUS "using env variable ROS_VERSION: ${ROS_VERSION}")
else()
  find_package(catkin QUIET)
  if(catkin_FOUND)
    message(STATUS "found catkin package, must be ROS1!")
    set(ROS_VERSION "1")
  else()
    message(STATUS "catkin package not found, must be ROS2!")
    set(ROS_VERSION "2")
  endif()
endif()

if(ROS_VERSION EQUAL "2")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
elseif(ROS_VERSION EQUAL "1")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
else()
  message(SEND_ERROR "invalid ROS_VERSION: ${ROS_VERSION}")
endif()
