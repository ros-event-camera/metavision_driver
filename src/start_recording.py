#! /usr/bin/env python3

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
