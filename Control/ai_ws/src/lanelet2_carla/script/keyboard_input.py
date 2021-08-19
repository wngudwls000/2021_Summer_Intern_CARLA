#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


class KeyboardInput():
    def __init__(self):

        # self.pub_goallane = rospy.Publisher('/goal_node',Float64,queue_size=1)
        self.pub_goallane = rospy.Publisher('/goal_lat_lon',Float64MultiArray,queue_size=1)

    def getKey(self):
        
        lat_lon = Float64MultiArray()
        # lat = Float64()
        # lon = Float64()
        lat = input("type goal x: ")
        lon = input("type goal y: ")
        lat_lon.data = (lat,lon)

        self.pub_goallane.publish(lat_lon)


if __name__ == '__main__':
    rospy.init_node('KeyboardInput')
    while not rospy.is_shutdown():
        node = KeyboardInput()
        node.getKey()

    # # node.main()
    # rospy.spin()
