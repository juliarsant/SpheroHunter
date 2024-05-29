#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys

'''
    Publishes the twist commands so the locobot can turn.
    The commands are published to /mobile_base/cmd_vel to interact
    directly with the motor
'''
def publish_velocity(linear_vel, angular_vel, duration=0.5):
    # Create a publisher for the /mobile_base/cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)
    
    # Construct the Twist message
    twist_msg = Twist()
    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel
    
    # Publish the message for the specified duration
    rate = rospy.Rate(10)  # 10Hz
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        cmd_vel_pub.publish(twist_msg)
        rate.sleep()