#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point

from collections import deque
import numpy as np


class Waypoint:
    def __init__(self):

        self.waypoints_ = []
        self.position_ = []
        self.waypoints_essence = []

    def path_callback(self,path_msg):

        waypoints=[]

        for i in range (len(path_msg.poses)):
            x = path_msg.poses[i].pose.position.x
            y = path_msg.poses[i].pose.position.y

            (roll,pitch,yaw) = euler_from_quaternion([path_msg.poses[i].pose.orientation.x,path_msg.poses[i].pose.orientation.y,path_msg.poses[i].pose.orientation.z,path_msg.poses[i].pose.orientation.w])
            yaw = np.degrees(yaw)
            # print(x,y,yaw)

            waypoint = [x,y,yaw]
            waypoints.append(waypoint)
        
        try:
            self.waypoints_ = deque(waypoints)
            self.waypoints_essence = deque(path_msg.poses)
            # print(waypoints)
            # print(deque(path_msg.poses))
        except:
            print("NO_WAYPOINTS_RECEIVED")

    def odometry_callback(self,odo_msg):

        x = odo_msg.pose.pose.position.x
        y = odo_msg.pose.pose.position.y
        # z = odo_msg.pose.pose.position.z

        (roll,pitch,yaw) = euler_from_quaternion([  odo_msg.pose.pose.orientation.x,
                                                    odo_msg.pose.pose.orientation.y,
                                                    odo_msg.pose.pose.orientation.z,
                                                    odo_msg.pose.pose.orientation.w])
        yaw = np.degrees(yaw)
        # print(round(x),round(y), round(z),round(yaw))
        self.position_ = [x,y,yaw]


    def waypoint_next(self,look_ahead = 1):

        position = self.position_
        waypoint = self.waypoints_[0]
    
        distance = ((position[0] - waypoint[0])**2 + (position[1] - waypoint[1])**2)**0.5 

        if distance < look_ahead:
            self.waypoints_.popleft()
            self.waypoints_essence.popleft()

        # print(self.waypoints_[0])
        return self.waypoints_essence



def main():

    waypoint = Waypoint()
    rospy.init_node('waypoint_local', anonymous=True)

    # rospy.Subscriber("/carla/ego_vehicle/rgb_view/image", Image, image_callback)
    rospy.Subscriber("/carla/ego_vehicle/waypoints",Path,waypoint.path_callback)
    rospy.Subscriber("/carla/ego_vehicle/odometry",Odometry,waypoint.odometry_callback)
    
    pub = rospy.Publisher('/carla/ego_vehicle/waypoints_essence', Path, queue_size=1)
    


    while not rospy.is_shutdown():


        waypoints_front = Path()
        try:
            waypoints_front.poses = waypoint.waypoint_next()
            print(waypoints_front.poses[0])
        except:
            print("EMPTY_PUB")
        pub.publish(waypoints_front)



if __name__ == '__main__':
    main()

