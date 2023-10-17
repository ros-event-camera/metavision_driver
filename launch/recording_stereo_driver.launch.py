# -----------------------------------------------------------------------------
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
#
#

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    cam_0_name = LaunchConfig("camera_0_name")
    cam_0_str = cam_0_name.perform(context)
    cam_1_name = LaunchConfig("camera_1_name")
    cam_1_str = cam_1_name.perform(context)
    pkg_name = "metavision_driver"
    share_dir = get_package_share_directory(pkg_name)
    bias_config = os.path.join(share_dir, "config", "silky_ev_cam.bias")  # noqa F841
    #
    # camera 0
    #
    cam_0 = ComposableNode(
        package="metavision_driver",
        plugin="metavision_driver::DriverROS2",
        name=cam_0_name,
        parameters=[
            {
                "use_multithreading": False,
                # 'bias_file': bias_config,
                "camerainfo_url": "",
                "frame_id": "cam_0",
                "serial": "CenturyArks:evc3a_plugin_gen31:00000198",
                "sync_mode": "primary",
                "event_message_time_threshold": 1.0e-3,
            }
        ],
        remappings=[
            ("~/events", cam_0_str + "/events"),
            # must remap so primary listens to secondary's ready message
            ("~/ready", cam_1_str + "/ready"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    #
    # camera 1
    #
    cam_1 = ComposableNode(
        package="metavision_driver",
        plugin="metavision_driver::DriverROS2",
        name=cam_1_name,
        parameters=[
            {
                "use_multithreading": False,
                # 'bias_file': bias_config,
                "camerainfo_url": "",
                "frame_id": "cam_1",
                "serial": "CenturyArks:evc3a_plugin_gen31:00000293",
                "sync_mode": "secondary",
                "event_message_time_threshold": 1.0e-3,
            }
        ],
        remappings=[("~/events", cam_1_str + "/events")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    #
    # recorder
    #
    recorder = ComposableNode(
        package="rosbag2_composable_recorder",
        plugin="rosbag2_composable_recorder::ComposableRecorder",
        name="recorder",
        parameters=[
            {
                "topics": ["/" + cam_0_str + "/events", "/" + cam_1_str + "/events"],
                "bag_name": LaunchConfig("bag"),
                "bag_prefix": LaunchConfig("bag_prefix"),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    #
    # the container for all 3 executables, avoid network messages
    #
    container = ComposableNodeContainer(
        name="metavision_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        # prefix=['xterm -e gdb -ex run --args'],
        composable_node_descriptions=[cam_0, cam_1, recorder],
        output="screen",
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg(
                "camera_0_name",
                default_value=["event_cam_0"],
                description="camera name of camera 0",
            ),
            LaunchArg(
                "camera_1_name",
                default_value=["event_cam_1"],
                description="camera name of camera 1",
            ),
            LaunchArg("bag", default_value=[""], description="name of output bag"),
            LaunchArg("bag_prefix", default_value=["events_"], description="prefix of output bag"),
            OpaqueFunction(function=launch_setup),
        ]
    )
