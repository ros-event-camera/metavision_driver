#!/usr/bin/env python3
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
"""
Test code for event packet time stamp debugging.

Some code snippets for rosbag reading were taken from
https://github.com/ros2/rosbag2/blob/master/rosbag2_py/test/test_sequential_reader.py
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def read_bag(args):
    bag_path = str(args.bag)
    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    print(f"opening bag {args.bag}")
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=[args.topic])
    reader.set_filter(storage_filter)

    print("iterating through messages...")
    t0_ros, t0_sensor = None, None
    t_last_evt = None
    ros_times, sensor_times, rec_times = [], [], []
    num_ts_future = 0

    while reader.has_next():
        (topic, data, t_rec) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        # unpack time stamps for all events in the message
        packed = np.frombuffer(msg.events, dtype=np.uint64)
        dt = np.bitwise_and(packed, 0xFFFFFFFF)
        t_sensor = dt + msg.time_base

        t_ros = dt + Time.from_msg(msg.header.stamp).nanoseconds
        t_rec_nsec = t_rec
        if not t_last_evt:
            t_last_evt = t_ros[0] - 1
        if not t0_ros:
            t0_ros = t_ros[0]
            t0_sensor = t_sensor[0]
            t0_rec = t_rec_nsec

        dt_msg = t_ros[0] - t_last_evt
        if dt_msg < 0:
            print("ERROR: timestamp going backward at time: ", t_ros[0])
        if t_ros[-1] > t_rec_nsec:
            num_ts_future += 1
            # print('WARN: timestamp from the future (can happen): ',
            # f'{t_ros[0]} diff: {(t_ros[-1] - t_rec_nsec) * 1e-9}')
        t_last_evt = t_ros[-1]
        ros_times.append(t_ros[0] - t0_ros)
        sensor_times.append(t_sensor[0] - t0_sensor)
        rec_times.append(t_rec_nsec - t0_rec)

    print(
        "fraction of messages with header stamp > recording stamp:",
        f"{num_ts_future * 100 / len(ros_times)}%",
    )
    return (
        np.array(ros_times).astype(np.float),
        np.array(sensor_times).astype(np.float),
        np.array(rec_times).astype(np.float),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="examine ROS time stamps for event packet bag.")
    parser.add_argument(
        "--bag",
        "-b",
        action="store",
        default=None,
        required=True,
        help="bag file to read events from",
    )
    parser.add_argument(
        "--topic", help="Event topic to read", default="/event_camera/events", type=str
    )
    ros_times, sensor_times, rec_times = read_bag(parser.parse_args())
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(
        ros_times * 1e-9, (ros_times - sensor_times) * 1e-9, "g", label="ros stamp - sensor time"
    )
    ax.plot(
        ros_times * 1e-9,
        (rec_times - ros_times) * 1e-9,
        "r.",
        label="rec time - ros stamp",
        markersize=0.2,
    )
    ax.set_xlabel("time [sec]")
    ax.set_ylabel("time differences [sec]")
    ax.legend()
    ax.set_title("time offsets to ROS message header stamps")
    plt.show()
