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

import launch
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

common_params = {
    "use_multithreading": False,
    "bias_file": "",
    "camerainfo_url": "",
    "event_message_time_threshold": 1.0e-3,
}


def make_renderer(camera, fps):
    return ComposableNode(
        package="event_camera_renderer",
        plugin="event_camera_renderer::Renderer",
        namespace=camera,
        name="renderer",
        parameters=[{"fps": fps}],
        # map one level up
        remappings=[("~/events", "camera/events")],
        # remappings=[],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_camera(camera, params, remappings):
    return ComposableNode(
        package="metavision_driver",
        plugin="metavision_driver::DriverROS2",
        namespace=camera,
        name="camera",
        parameters=params,
        remappings=remappings,
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    nodes = []
    cam_names = {}
    for cam in ("camera_0", "camera_1"):
        cam_names[cam] = LaunchConfig(cam + "_name").perform(context)

    for cam in ("camera_0", "camera_1"):
        camera = cam_names[cam]
        params = [
            {
                "serial": LaunchConfig(cam + "_serial").perform(context),
                "settings": LaunchConfig(cam + "_settings").perform(context),
                "statistics_print_interval": float(
                    LaunchConfig("statistics_print_interval").perform(context)
                ),
            }
        ]
        remappings = []
        sync_mode = LaunchConfig(cam + "_sync_mode").perform(context)
        if sync_mode == "primary":
            other_cam = "camera_1" if cam == "camera_0" else "camera_0"
            if LaunchConfig(other_cam + "_sync_mode").perform(context) == "secondary":
                # If the other camera is in secondary sync mode, rempa the ready signal.
                other_camera = cam_names[other_cam]
                remappings += [("~/ready", "/" + other_camera + "/camera/ready")]
            params[0]["sync_mode"] = "primary"
        else:
            params[0]["sync_mode"] = "secondary"
        nodes += [make_camera(camera, params, remappings)]

        if IfCondition(LaunchConfig("with_renderer")).evaluate(context):
            nodes += [
                make_renderer(camera, float(LaunchConfig("fps").perform(context)))
            ]

    container = ComposableNodeContainer(
        name="metavision_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=nodes,
        output="screen",
    )

    debug_with_libasan = False
    if debug_with_libasan:
        preload = SetEnvironmentVariable(
            name="LD_PRELOAD", value="/usr/lib/gcc/x86_64-linux-gnu/13/libasan.so"
        )
        asan_options = SetEnvironmentVariable(
            name="ASAN_OPTIONS", value="new_delete_type_mismatch=0"
        )
        return [preload, asan_options, container]

    return [container]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    per_camera_args = []
    for cam in ["camera_0", "camera_1"]:
        per_camera_args += [
            LaunchArg(
                cam + "_name",
                default_value=["event_cam_0" if cam == "camera_0" else "event_cam_1"],
                description="name of " + cam,
            ),
            LaunchArg(
                cam + "_serial",
                # serial numbers of my cameras, replace with your own!
                default_value=["4110030785" if cam == "camera_0" else "4110030791"],
                description="serial number of " + cam,
            ),
            LaunchArg(
                cam + "_settings",
                default_value=[""],
                description="settings file for " + cam,
            ),
            LaunchArg(
                cam + "_sync_mode",
                default_value=["primary"] if cam == "camera_0" else ["secondary"],
                description="settings file for " + cam,
            ),
        ]

    return launch.LaunchDescription(
        per_camera_args
        + [
            LaunchArg(
                "fps",
                default_value=["25.0"],
                description="renderer frame rate in Hz",
            ),
            LaunchArg(
                "statistics_print_interval",
                default_value=["2.0"],
                description="time in seconds between statistics printing",
            ),
            LaunchArg(
                "with_renderer",
                default_value="false",
                description="if renderers should be started as well",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
