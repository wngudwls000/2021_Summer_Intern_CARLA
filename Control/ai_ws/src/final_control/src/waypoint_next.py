#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
import numpy as np
import math

position = [0.,0.,0.]

class WaypointNext:
    def __init__(self):
        pass

    def 


def odometry_callback(odo_msg):
    global position
    x = odo_msg.pose.pose.position.x
    y = odo_msg.pose.pose.position.y
    # z = odo_msg.pose.pose.position.z

    (roll,pitch,yaw) = euler_from_quaternion([odo_msg.pose.pose.orientation.x,odo_msg.pose.pose.orientation.y,odo_msg.pose.pose.orientation.z,odo_msg.pose.pose.orientation.w])
    yaw = np.degrees(yaw)
    # print(round(x),round(y), round(z),round(yaw))
    position = [x,y,yaw]


def main():


    rospy.Subscriber("/carla/ego_vehicle/waypoints_essence",Path,waypoint_callback)
    rospy.Subscriber("/carla/ego_vehicle/odometry",Odometry,odometry_callback)



    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
    

    control_vehicle = VehiclePIDController(vehicle, args_Lateral={'wheel_base':wheelbase, 'max_steer':max_steer},args_LongitudnaL={'K_P':2,'K_D':0.0,'K_I':0.0})
    look_ahead = LookAhead(world,vehicle)

    while not rospy.is_shutdown():

        target_speed,waypoint = look_ahead.set_target_speed(waypoints)

        control = CarlaEgoVehicleControl()
        control_signal = control_vehicle.run_step(control,target_speed,waypoint)
        pub.publish(control_signal)
        

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()

