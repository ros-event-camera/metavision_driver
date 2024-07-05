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


#add_compile_options(-Wall -Wextra -pedantic -Werror)
add_compile_options(-Wall -Wextra -Wpedantic)
#add_compile_definitions(USING_ROS_1)
add_definitions(-DUSING_ROS_1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  dynamic_reconfigure
  event_camera_msgs
  std_srvs)

# MetavisionSDK is now found otherwise
# find_package(MetavisionSDK COMPONENTS driver REQUIRED)

if(MetavisionSDK_VERSION_MAJOR LESS 4)
  add_definitions(-DUSING_METAVISION_3)
endif()

generate_dynamic_reconfigure_options(
  cfg/MetaVisionDyn.cfg)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

#
# --------- driver -------------

# code common to nodelet and node
add_library(driver_common
  src/driver_ros1.cpp src/bias_parameter.cpp src/metavision_wrapper.cpp)
target_link_libraries(driver_common MetavisionSDK::driver ${catkin_LIBRARIES})
# to ensure messages get built before executable
add_dependencies(driver_common ${metavision_driver_EXPORTED_TARGETS})

# nodelet
add_library(driver_nodelet src/driver_nodelet_ros1.cpp)
target_link_libraries(driver_nodelet driver_common MetavisionSDK::driver ${catkin_LIBRARIES})
# to ensure messages get built before executable
add_dependencies(driver_nodelet ${metavision_driver_EXPORTED_TARGETS})

# node
add_executable(driver_node src/driver_node_ros1.cpp)
target_link_libraries(driver_node driver_common MetavisionSDK::driver ${catkin_LIBRARIES})
# to ensure messages get built before executable
add_dependencies(driver_node ${metavision_driver_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS driver_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch")

# install some example bias files
install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(MUST_INSTALL_METAVISION)
  install(DIRECTORY  "${CMAKE_CURRENT_BINARY_DIR}/_deps/metavision-build/lib"
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()


#############
## Testing ##
#############

# To be done...
