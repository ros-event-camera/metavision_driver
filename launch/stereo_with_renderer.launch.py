# -----------------------------------------------------------------------------
# Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


common_params = {
    "use_multithreading": False,
    "bias_file": "",
    "camerainfo_url": "",
    "event_message_time_threshold": 1.0e-3,
}


def make_renderers(cameras):
    nodes = [
        ComposableNode(
            package="event_camera_renderer",
            plugin="event_camera_renderer::Renderer",
            name=cam + "_renderer",
            parameters=[{"fps": 25.0}],
            remappings=[("~/events", cam + "/events")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam in cameras
    ]
    return nodes


def make_cameras(cameras, params):
    nodes = [
        ComposableNode(
            package="metavision_driver",
            plugin="metavision_driver::DriverROS2",
            name=cam,
            parameters=[
                common_params,
                {"serial": params[cam]["serial"]},
                {"sync_mode": params[cam]["sync_mode"]},
            ],
            remappings=params[cam]["remappings"],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam in cameras
    ]
    return nodes


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    cam_0_name = LaunchConfig("camera_0_name")
    cam_0 = cam_0_name.perform(context)
    cam_1_name = LaunchConfig("camera_1_name")
    cam_1 = cam_1_name.perform(context)
    cameras = (cam_0, cam_1)
    specific_params = {
        cam_0: {
            "serial": LaunchConfig("camera_0_serial").perform(context),
            "sync_mode": "primary",
            "remappings": [
                ("~/events", cam_0 + "/events"),
                ("~/ready", cam_1 + "/ready"),
            ],
        },
        cam_1: {
            "serial": LaunchConfig("camera_1_serial").perform(context),
            "sync_mode": "secondary",
            "remappings": [("~/events", cam_1 + "/events")],
        },
    }
    container = ComposableNodeContainer(
        name="metavision_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=make_cameras(cameras, specific_params)
        + make_renderers(cameras),
        output="screen",
    )
    preload = SetEnvironmentVariable(
        name="LD_PRELOAD", value="/usr/lib/gcc/x86_64-linux-gnu/13/libasan.so"
    )
    asan_options = SetEnvironmentVariable(
        name="ASAN_OPTIONS", value="new_delete_type_mismatch=0"
    )

    # return [preload, asan_options, container]
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
            LaunchArg(
                "camera_0_serial",
                default_value=["4110030785"],
                description="serial number of camera 0",
            ),
            LaunchArg(
                "camera_1_serial",
                default_value=["4110030791"],
                description="serial number of camera 1",
            ),
            LaunchArg(
                "fps",
                default_value=["fps"],
                description="renderer frame rate in Hz",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
