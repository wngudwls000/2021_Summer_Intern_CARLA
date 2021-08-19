#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class TestInput():
    def __init__(self):

        self.pub_goallane = rospy.Publisher('/car_odometry',Odometry,queue_size=1)

    def getKey(self):
        
        odo = Odometry()
        # lat = Float64()
        # lon = Float64()
        x = input("type current x: ")
        y = input("type current y: ")
        odo.pose.pose.position.x = x
        odo.pose.pose.position.y = y
        while True:
            self.pub_goallane.publish(odo)


if __name__ == '__main__':
    rospy.init_node('TestInput')
    while not rospy.is_shutdown():
        node = TestInput()
        node.getKey()

    # # node.main()
    # rospy.spin()