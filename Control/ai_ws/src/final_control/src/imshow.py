#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

import numpy as np

# Initialize the CvBridge class
bridge = CvBridge()
# pub = rospy.Publisher('myimg', Image, queue_size=10)

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def image_callback(img_msg):
    # rospy.loginfo(img_msg.encoding)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgra8")
    except :
        rospy.logerr("CvBridge Error")

     # Show the converted image
    show_image(cv_image)

def main():

    rospy.init_node('imshow', anonymous=True)

    rospy.Subscriber("/carla/ego_vehicle/rgb_view/image", Image, image_callback)
   
    rospy.spin()


    # spin() simply keeps python from exiting until this node is stopped

cv2.namedWindow("Image Window", 1)

if __name__ == '__main__':
    main()

    if rospy.is_shutdown():
        cv2.destroyAllWindows()
