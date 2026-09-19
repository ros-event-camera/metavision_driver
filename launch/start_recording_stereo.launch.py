# -----------------------------------------------------------------------------
# Copyright 2026 Bernd Pfrommer <bernd@eventvisionresearch.com>
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

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def make_name(prefix, context):
    now = datetime.now()
    return prefix.perform(context) + now.strftime("%Y_%m_%d-%H_%M_%S")


def launch_setup(context, *args, **kwargs):
    topics = [
        "/" + LaunchConfig(cam + "_name").perform(context) + "/camera/events"
        for cam in ("camera_0", "camera_1")
    ]
    launch_action = LoadComposableNodes(
        target_container=LaunchConfig("container_name"),
        composable_node_descriptions=[
            ComposableNode(
                package="rosbag2_transport",
                plugin="rosbag2_transport::Recorder",
                name="recorder",
                parameters=[
                    {
                        "record.topics": topics,
                        "record.start_paused": False,
                        "storage.uri": make_name(LaunchConfig("bag_prefix"), context),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )
    return [launch_action]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    per_camera_args = []
    for cam in ["camera_0", "camera_1"]:
        per_camera_args += [
            LaunchArg(
                cam + "_name",
                default_value=["event_cam_0" if cam == "camera_0" else "event_cam_1"],
                description="name of " + cam,
            )
        ]
    return LaunchDescription(
        per_camera_args
        + [
            LaunchArg(
                "bag_prefix",
                default_value=["events_"],
                description="prefix of rosbag",
            ),
            LaunchArg(
                "container_name",
                default_value=["metavision_driver_container"],
                description="name of the container for the camera driver nodes",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
