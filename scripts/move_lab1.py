#!/usr/bin/env python2

import roslib
roslib.load_manifest('lab2')

import rospy

from geometry_msgs.msg import Twist
from turtlebot_actions.msg import *
from actionlib_msgs.msg import *

import actionlib

class Demo:
    def __init__(self, distance):

        # Construct the action client
        rospy.loginfo("Starting action client...")
        action_client = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
        action_client.wait_for_server()
        rospy.loginfo("Action client connected to action server.")

        self.distance = distance
        self.sub = rospy.Subscriber('base_scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist)

    def move_forward(self, distance):
        rospy.loginfo("Calling the action server...")
        action_goal = TurtlebotMoveGoal()
        action_goal.forward_distance = distance

        if action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0), rospy.Duration(50.0)) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to action server succeeded')
        else:
            rospy.logerr('Call to action server failed')


    def laser_callback(self, scan):
        # Drive forward at a given speed.  The robot points up the x-axis.
        command = Twist()
        command.linear.x = 0.1
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
        rospy.loginfo('range0:  {0}, range1:  {1}'.format(scan.ranges[0], scan.ranges[1]))
        self.pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('move')
    demo = Demo(0.5)
    rospy.spin()

