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

import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    cam_name = LaunchConfig("camera_name")
    cam_str = cam_name.perform(context)
    container = ComposableNodeContainer(
        name="metavision_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        # prefix=['xterm -e gdb -ex run --args'],
        composable_node_descriptions=[
            ComposableNode(
                package="metavision_driver",
                plugin="metavision_driver::DriverROS2",
                name=cam_name,
                parameters=[
                    {
                        "use_multithreading": False,
                        "statistics_print_interval": 2.0,
                        "camerainfo_url": "",
                        "frame_id": "",
                        "event_message_time_threshold": 1.0e-3,
                    }
                ],
                remappings=[("~/events", cam_str + "/events")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="rosbag2_composable_recorder",
                plugin="rosbag2_composable_recorder::ComposableRecorder",
                name="recorder",
                parameters=[
                    {
                        "topics": ["/event_camera/events"],
                        "bag_name": LaunchConfig("bag"),
                        "bag_prefix": LaunchConfig("bag_prefix"),
                    }
                ],
                remappings=[("~/events", cam_str + "/events")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )
    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg("camera_name", default_value=["event_camera"], description="camera name"),
            LaunchArg("bag", default_value=[""], description="name of output bag"),
            LaunchArg("bag_prefix", default_value=["events_"], description="prefix of output bag"),
            OpaqueFunction(function=launch_setup),
        ]
    )
