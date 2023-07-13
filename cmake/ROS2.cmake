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

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # metavision SDK header files produce warnings, switch off for now
  #add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# find dependencies

# MetavisionSDK is now found otherwise
# find_package(MetavisionSDK COMPONENTS driver REQUIRED)

if(MetavisionSDK_VERSION_MAJOR LESS 4)
  add_definitions(-DUSING_METAVISION_3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)


set(ROS2_DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "event_camera_msgs"
  "std_srvs"
)

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})

#
# --------- driver (composable component) -------------

ament_auto_add_library(driver_ros2 SHARED
  src/metavision_wrapper.cpp
  src/bias_parameter.cpp
  src/driver_ros2.cpp)

target_include_directories(driver_ros2 PRIVATE include)
target_link_libraries(driver_ros2 MetavisionSDK::driver)

rclcpp_components_register_nodes(driver_ros2 "metavision_driver::DriverROS2")

# --------- driver (plain old node) -------------

ament_auto_add_executable(driver_node
  src/driver_node_ros2.cpp)


# the node must go into the project specific lib directory or else
# the launch file will not find it
install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}/)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  driver_ros2
  DESTINATION lib)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

# install some example bias files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

if(MUST_INSTALL_METAVISION)
  install(DIRECTORY  "${CMAKE_CURRENT_BINARY_DIR}/_deps/metavision-build/lib"
    DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_flake8()
  #ament_lint_cmake(--filter=-readability/wonkycase)
  ament_lint_cmake()
  ament_pep257()
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_xmllint()
endif()

ament_package()
