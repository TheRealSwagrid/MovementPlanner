#!/usr/bin/env python

import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_about_axis
from AbstractVirtualCapability import VirtualCapabilityServer
from visualization_msgs.msg import Marker
from copy import copy
from MovementPlanner import MovementPlanner
from time import sleep


class movementplanner:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('rosnode')
    rate = rospy.Rate(10)
    rospy.logwarn("HEYO IM HERE")
    server = VirtualCapabilityServer(int(rospy.get_param('~semantix_port')))
    place_robot = MovementPlanner(server)
    place_robot.start()
    robot = movementplanner()


    while not rospy.is_shutdown():
        robot.publish_visual()
        rate.sleep()
