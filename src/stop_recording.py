#! /usr/bin/env python3

import rospy
import actionlib

from nodelet_rosbag.msg import StopAction, StopGoal

if __name__ == '__main__':
    rospy.init_node('stop_recording_client')
    print('sending command to stop recording!')
    client = actionlib.SimpleActionClient('stop', StopAction)
    client.wait_for_server()
    goal = StopGoal()
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print('recording should be stopped now...')
