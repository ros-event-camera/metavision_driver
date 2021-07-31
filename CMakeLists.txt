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

cmake_minimum_required(VERSION 3.0.2)
project(metavision_ros_driver)

add_compile_options(-Wall -Wextra -pedantic -Werror)

find_package(catkin REQUIRED COMPONENTS
  dvs_msgs
  prophesee_event_msgs
  roscpp
  nodelet
  )

find_package(MetavisionSDK COMPONENTS driver REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#
# --------- driver -------------

# nodelet
add_library(driver_nodelet src/driver_nodelet.cpp)
target_link_libraries(driver_nodelet MetavisionSDK::driver ${catkin_LIBRARIES})
# node
add_executable(driver_node src/driver_node.cpp)
target_link_libraries(driver_node MetavisionSDK::driver ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS driver_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
 
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  )


#############
## Testing ##
#############

# To be done...
