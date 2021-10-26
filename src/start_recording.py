#! /usr/bin/env python3
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

import rospy
import actionlib

from nodelet_rosbag.msg import StartAction, StartGoal

if __name__ == '__main__':
    rospy.init_node('start_recording_client')
    print('sending command to start recording!')
    client = actionlib.SimpleActionClient('start', StartAction)
    client.wait_for_server()
    goal = StartGoal()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print('recording should be started now...')
