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
"""test code for event packet time stamp debugging."""

import rosbag
import argparse
import numpy as np
import matplotlib.pyplot as plt


def read_bag(args):
    print(f'opening bag {args.bag} ...')
    bag = rosbag.Bag(args.bag)
    print('iterating through messages...')
    t0_ros, t0_sensor = None, None
    t_last_evt = None
    ros_times, sensor_times, rec_times = [], [], []

    for topic, msg, t_rec in bag.read_messages(topics=[args.topic]):
        # unpack time stamps for all events in the message
        packed = np.frombuffer(msg.events, dtype=np.uint64)
        dt = np.bitwise_and(packed, 0xFFFFFFFF)
        t_sensor = dt + msg.time_base
        t_ros = dt + msg.header.stamp.to_nsec()
        t_rec_nsec = t_rec.to_nsec()
        if not t_last_evt:
            t_last_evt = t_ros[0] - 1
        if not t0_ros:
            t0_ros = t_ros[0]
            t0_sensor = t_sensor[0]
            t0_rec = t_rec_nsec

        dt_msg = t_ros[0] - t_last_evt
        if dt_msg < 0:
            print('ERROR: timestamp going backward at time: ', t_ros[0])
        if t_ros[-1] > t_rec_nsec:
            print('WARN: timestamp from the future (can happen): ',
                  f'{t_ros[0]} diff: {(t_ros[-1] - t_rec_nsec) * 1e-9}')
        t_last_evt = t_ros[-1]
        ros_times.append(t_ros[0] - t0_ros)
        sensor_times.append(t_sensor[0] - t0_sensor)
        rec_times.append(t_rec_nsec - t0_rec)

    return np.array(ros_times).astype(np.float), \
        np.array(sensor_times).astype(np.float), \
        np.array(rec_times).astype(np.float)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='examine ROS time stamps for event packet bag.')
    parser.add_argument('--bag', '-b', action='store', default=None,
                        required=True, help='bag file to read events from')
    parser.add_argument('--topic', help='Event topic to read',
                        default='/event_camera/events', type=str)
    ros_times, sensor_times, rec_times = read_bag(parser.parse_args())
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(ros_times * 1e-9, (ros_times - sensor_times) * 1e-9,
            'g', label='ros stamp - sensor time')
    ax.plot(ros_times * 1e-9, (rec_times - ros_times) * 1e-9,
            'r.', label='rec time - ros stamp', markersize=0.2)
    ax.set_xlabel('time [sec]')
    ax.set_ylabel('time differences [sec]')
    ax.legend()
    ax.set_ylim([-0.004, 0.005])
    ax.set_title('time offsets to ROS message header stamps')
    plt.show()
