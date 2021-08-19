#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from darknet_ros_msgs.msg   import BoundingBox, BoundingBoxes

def callback(data):
    rospy.get_caller_id() + "%s", data.bounding_boxes
    msg = data.bounding_boxes[0].Class
    print(msg)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
