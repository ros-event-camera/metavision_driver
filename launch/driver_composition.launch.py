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
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


fixed_params = {
    "use_multithreading": False,
    "bias_file": "",
    "camerainfo_url": "",
    "frame_id": "",
    "event_message_time_threshold": 1.0e-3,
}


def make_renderer(camera):
    return ComposableNode(
        package="event_camera_renderer",
        plugin="event_camera_renderer::Renderer",
        name=camera + "_renderer",
        parameters=[{"fps": 25.0}],
        remappings=[("~/events", camera + "/events")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_fibar(camera):
    return ComposableNode(
        package="event_image_reconstruction_fibar",
        plugin="event_image_reconstruction_fibar::Fibar",
        name=camera + "_fibar",
        parameters=[{"fps": 25.0}],
        remappings=[("~/events", camera + "/events")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def make_camera(camera, params, remappings):
    return ComposableNode(
        package="metavision_driver",
        plugin="metavision_driver::DriverROS2",
        name=camera,
        parameters=params,
        remappings=remappings,
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    camera = LaunchConfig("camera_name").perform(context)
    params = [
        {
            "serial": LaunchConfig("serial").perform(context),
            "settings": LaunchConfig("settings").perform(context),
        }
    ]
    remappings = []
    nodes = [make_camera(camera, params, remappings)]

    if IfCondition(LaunchConfig("with_renderer")).evaluate(context):
        nodes += [make_renderer(camera)]
    if IfCondition(LaunchConfig("with_fibar")).evaluate(context):
        nodes += [make_fibar(camera)]
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
    return launch.LaunchDescription(
        [
            LaunchArg(
                "camera_name",
                default_value=["event_camera"],
                description="name of camera",
            ),
            LaunchArg(
                "serial",
                default_value=["4110030785"],
                description="serial number of camera",
            ),
            LaunchArg(
                "settings",
                default_value=[""],
                description="settings file for camera",
            ),
            LaunchArg(
                "fps",
                default_value=["fps"],
                description="renderer and fibar frame rate in Hz",
            ),
            LaunchArg(
                "with_renderer",
                default_value="false",
                description="if renderers should be started as well",
            ),
            LaunchArg(
                "with_fibar",
                default_value="false",
                description="if fibar reconstruction should be started as well",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
