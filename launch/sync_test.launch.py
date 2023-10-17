# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    topic_0 = LaunchConfig("cam_0_topic")
    topic_1 = LaunchConfig("cam_1_topic")
    node = Node(
        package="metavision_driver",
        executable="sync_test",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        name="sync_test",
        parameters=[{"use_ros_time": True}],
        remappings=[("~/events_cam_0", topic_0), ("~/events_cam_1", topic_1)],
    )
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg(
                "cam_0_topic",
                default_value=["/event_cam_0/events"],
                description="event topic cam 0",
            ),
            LaunchArg(
                "cam_1_topic",
                default_value=["/event_cam_1/events"],
                description="event topic cam 1",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
