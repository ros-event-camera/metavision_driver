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
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    cam_name = LaunchConfig("camera_name")
    node = Node(
        package="metavision_driver",
        executable="driver_node",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        name=cam_name,
        parameters=[
            # trigger_config,  # loads the whole file
            {
                "use_multithreading": False,
                "send_queue_size": 1,
                "statistics_print_interval": 2.0,
                "bias_file": "",  # bias_config,
                "camerainfo_url": "",
                "frame_id": "",
                "serial": LaunchConfig("serial"),
                "erc_mode": "disabled",
                # "erc_rate": 100000000,
                "trail_filter": False,
                "trail_filter_type": "stc_cut_trail",
                "trail_filter_threshold": 5000,
                # "roi": [0, 0, 320, 320],
                # "roni": False,
                # valid: 'external', 'loopback', 'disabled'
                "trigger_in_mode": "external",
                # valid: 'enabled', 'disabled'
                # "trigger_out_mode": "enabled",
                # "trigger_out_period": 100000,  # in usec
                # "trigger_duty_cycle": 0.5,  # fraction high/low
                "event_message_time_threshold": 1.0e-3,
                # "bias_diff_off": 0,
                # "bias_diff_on": 0,
                # "bias_hpf": 0,
                # "bias_fo": 0,
                # "bias_refr": 0,
            },
        ],
        remappings=[],
    )
    preload = SetEnvironmentVariable(
        name="LD_PRELOAD", value="/usr/lib/gcc/x86_64-linux-gnu/13/libasan.so"
    )
    asan_options = SetEnvironmentVariable(
        name="ASAN_OPTIONS", value="new_delete_type_mismatch=0"
    )
    return [preload, asan_options, node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription(
        [
            LaunchArg(
                "camera_name", default_value=["event_camera"], description="camera name"
            ),
            LaunchArg(
                "serial", default_value=[""], description="serial number of camera"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
