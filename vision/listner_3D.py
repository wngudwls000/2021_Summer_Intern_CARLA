#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from geometry_msgs.msg import Pose

def callback(data):
    rospy.get_caller_id() + "%s", data.objects
    msg = data.objects[0].pose
    Point = msg.position
    Quaternion = msg.orientation
    print("Point: ", Point)
    print("----------------------")
    print("Quaternion: ", Quaternion)
    print("----------------------")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/detected_objects", DetectedObjectArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
