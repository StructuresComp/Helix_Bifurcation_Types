#!/usr/bin/env python2
import message_filters
import rospy
from sensor_msgs.msg import JointState

"""
    source: https://groups.google.com/g/moveit-users/c/jKV9QuHeFXU
"""

pub = rospy.Publisher('joint_states', JointState, queue_size=10)


def remap_callback(msg):
    pub.publish(msg)


def control():
    rospy.init_node('joint_control', anonymous=True)
    rospy.Subscriber("/robot/joint_states", JointState, remap_callback)
    rospy.spin()


control()
