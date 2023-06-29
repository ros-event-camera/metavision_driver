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

cmake_minimum_required(VERSION 3.5)
project(metavision_driver)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL "2")
    # to avoid cmake warning: "Manually-specified variables were not used by the project"
    unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
    unset(CATKIN_SYMLINK_INSTALL)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
  elseif($ENV{ROS_VERSION} EQUAL "1")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
  else()
    message(SEND_ERROR "invalid ROS_VERSION environment variable: $ENV{ROS_VERSION}")
  endif()
else()
  message(SEND_ERROR "ROS_VERSION environment variable is not set!")
endif()
