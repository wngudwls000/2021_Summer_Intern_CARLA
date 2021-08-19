#!/usr/bin/env python

import rospy

from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float32

import numpy as np
import math
import Queue as queue

import test
import time



speed = 0.0


def speed_callback(speed_):
    global speed
 
    vel = 3.6*abs(speed_.data)
    # print(round(vel,1))
    speed = vel

   
class Waypoint:
    def __init__(self, max_speed = 45):

        self.waypoints_ = []
        self.position_ = [0.,0.,0.]
        self.way_ind = 0

        self.max_speed = max_speed
        # pps = purepursuit , ts = target_speed
        self.look_ahead_for_pps = 20
        self.look_ahead_for_ts = 30
        self.object_positions = []
        self.light_state = "Off"
        self.intersection_list = [[336.32825041127336, -329.608680016030],[90.52338005072826, -329.57531672166886],[90.29606064172627, -197.92127247778305],[336.3936525220271, -197.69605319953516],[90.28596119784183, -131.23077835436845],[336.43315578795006, -132.0076260318536],[336.47787156722825, -57.652273237553906],[156.00887950691, -57.66697875489183],[90.27477068052197, -57.326643113004344],[90.0490868464229, 0.04542633770564263],[155.88480774831368, 0.04298432621406783],[336.3462432111822, -0.0015181974685102803]]




    def cost_function(self,paths):
        path_valid = True
        paths_cost = [200,200,200,200,200]
        # paths_cost = np.zeros(len(paths))
        object_distance = []
        
        if paths: # path in lattice
            
            for i in range(len(paths)): # set default cost in lattice
                paths_cost[i] = abs(i - (len(paths)-1)//2)

                # print("path_ind: ",i)
                for j in range(len(paths[i][0])):
                    for object in self.object_positions:

                        # print("point ",paths[i][0][j]," ",paths[i][1][j])
                        distance = ((paths[i][0][j]-object[0])**2 +(paths[i][1][j]-object[1])**2)**0.5
                        if distance <2.5:
                            # print("point ",paths[i][0][j]," ",paths[i][1][j])
                            # print(distance)
                            paths_cost[i] = 100
                            object_distance.append((object[0]**2 + object[1]**2)**0.5)
                            break
            if object_distance:
                object_distance_min = min(object_distance)
            else:
                object_distance_min = None   
            print(paths_cost)
            best_index = paths_cost.index(min(paths_cost))


            if paths_cost[best_index]>=100: # when best is collistion
                best_index = int((len(paths)-1)/2)
                path_valid = False # No path without collision
                # if valid_index == -1:  # to fix valid index == -1
                #     valid_index =  None

        else:   # No path in lattice
            print("No path in lattice")
            path_valid = False
            best_index = None
            object_distance_min = None
        
        return best_index,path_valid,object_distance_min
                        
    def cruise_control(self,object_distance): #####
        global speed

        timegap = 0.3
        target_distance = 10+timegap*speed
        distance_error = target_distance - object_distance
        cruise_speed = speed - (distance_error)/timegap

        return cruise_speed

    def intersection_conrol(self):

        red_light = False
        if self.light_state == "Red" or self.light_state == "RedRight" or self.light_state == "RedLeft" :
            red_light = True
        waypoints = self.waypoints_
        intersection_speed = self.max_speed

        waypoint_x = waypoints[self.way_ind][0]
        waypoint_y = waypoints[self.way_ind][1]
        target_point = [waypoint_x,waypoint_y]

        for intersection_point in self.intersection_list:
            if ((intersection_point[0]-target_point[0])**2 + (intersection_point[1]-target_point[0])**2)**0.5 < 10 and red_light:
                intersection_speed = 0

        return intersection_speed

    def traffic_callback(self,traffic_msg):
        
        traffic_prob = []
        
        for i in range(len(traffic_msg.bounding_boxes)):
            traffic_prob.append(traffic_msg.bounding_boxes.probablity)

        traffic_ind = traffic_prob.index(max(traffic_prob))
        self.light_state = traffic_msg.bounding_boxes[traffic_ind].Class

    def object_callback(self,obj_msg):

        for detected_object in obj_msg.objects:
            x = detected_object.pose.position.x
            y = detected_object.pose.position.y
            self.object_positions.append([x,y])


    def path_callback(self,path_msg):

        waypoints=[]
    
        for i in range (len(path_msg.poses)):

            x = path_msg.poses[i].pose.position.x
            y = path_msg.poses[i].pose.position.y

            (roll,pitch,yaw) = euler_from_quaternion([path_msg.poses[i].pose.orientation.x,path_msg.poses[i].pose.orientation.y,path_msg.poses[i].pose.orientation.z,path_msg.poses[i].pose.orientation.w])
            # yaw = np.degrees(yaw)
            # print(x,y,yaw)

            waypoint = [x,y,yaw]
            waypoints.append(waypoint)
        start = 0
        for i in range (len(waypoints)):
            
            distance = ((self.position_[0] - waypoints[i][0])**2 + (self.position_[1] - waypoints[i][1])**2)**0.5 
            if distance < 1.0:
                start = i

        waypoints = waypoints[start:]
        print("WAYPOINT SAVED")
        self.way_ind = 0 
        print("WAYPOINT INIT")

        self.waypoints_ = waypoints


    def odometry_callback(self,odo_msg):
        x = odo_msg.pose.pose.position.x
        y = odo_msg.pose.pose.position.y
        # z = odo_msg.pose.pose.position.z

        (roll,pitch,yaw) = euler_from_quaternion([  odo_msg.pose.pose.orientation.x,
                                                    odo_msg.pose.pose.orientation.y,
                                                    odo_msg.pose.pose.orientation.z,
                                                    odo_msg.pose.pose.orientation.w])
        # yaw = np.degrees(yaw)
        # print(round(x),round(y),round(yaw))
        self.position_ = [x,y,yaw]


    def waypoint_index(self):

        position = self.position_
        waypoints = self.waypoints_
        # print(waypoints[self.way_ind])
        for i in range (self.way_ind,len(waypoints)-1):
            distance = ((position[0] - waypoints[i][0])**2 + (position[1] - waypoints[i][1])**2)**0.5 
            if distance < 1.0:
                self.way_ind += 1
    

    def waypoint_next(self,look_ahead = 1):
        waypoints = self.waypoints_

        try:
            waypoint_x = waypoints[look_ahead+self.way_ind][0]
            waypoint_y = waypoints[look_ahead+self.way_ind][1]
            waypoin_yaw = waypoints[look_ahead+self.way_ind][2]
            target_point = [waypoint_x,waypoint_y, waypoin_yaw]
        except:
            target_point = [None,None,None]
            
        return target_point

    def set_target_speed(self):

        global speed

        position = self.position_
        look_ahead_waypoints_ts = []
        try:
            for i in range(int(self.look_ahead_for_ts)):

                dx = self.waypoint_next(i)[0] - position[0]
                dy = self.waypoint_next(i)[1] - position[1]

                y = dx*math.cos(position[2])+dy*math.sin(position[2])
                x = -(-dx*math.sin(position[2])+dy*math.cos(position[2]))


                look_ahead_waypoint = [x,y]
                look_ahead_waypoints_ts.append(look_ahead_waypoint)
            # print(int(position[0]),int(self.waypoint_next(0)[0]),int(position[1]),int(self.waypoint_next(0)[1]))
            len_look_ts = len(look_ahead_waypoints_ts)
            # look_ahead_waypoints_ts = (np.array(look_ahead_waypoints_ts)).T # [[x,x,x,x,x,x],[y,y,y,y,y,y]]
            curvature_list = []
            if len_look_ts > 3:
                for i in range(len_look_ts-2):
                    # curvature = 4*triangleArea/(sideLength1*sideLength2*sideLength3)
                    menger_curve = look_ahead_waypoints_ts[i:i+3]

                    ax = menger_curve[0][0]
                    bx = menger_curve[1][0]
                    cx = menger_curve[2][0]

                    ay = menger_curve[0][1]
                    by = menger_curve[1][1]
                    cy = menger_curve[2][1]

                    triangleArea = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)
                    sideLength1 = ((bx-ax)**2 +(by-ay)**2)**0.5
                    sideLength2 = ((bx-cx)**2 +(by-cy)**2)**0.5
                    sideLength3 = ((cx-ax)**2 +(cy-ay)**2)**0.5
                    
                    try:
                        curvature = abs(4*triangleArea/(sideLength1*sideLength2*sideLength3))
                    except: curvature = 0
                    curvature_list.append(curvature)
                    

            max_curvature = np.max(np.array(curvature_list))
            # print(max_curvature)

            if max_curvature > 0.3: 
                max_curvature = 0.3
            # if max_curvature>0.001:
            target_speed = self.max_speed*(1-max_curvature/0.5)
            target_speed = np.clip(target_speed,0.0,self.max_speed)
            
            # print(round(max_curvature,4) , round(target_speed) , round(speed)) 

            # if speed>target_speed:
            #     if target_speed>self.target:
            #         self.target += 0.005
            #     else:
            #         self.target -= 0.1
            #     speed = self.target
            # else:
            #     if speed>self.target:
            #         self.target += 0.005
            #     else:
            #         self.target = speed
            # speed = self.target

            ###########todo :: +=1 LD
            
            self.look_ahead_for_pps = int(2.0 + (speed*0.237))
            self.look_ahead_for_ts = int(10 + speed / 5)
            waypoint_pps = self.waypoint_next(self.look_ahead_for_pps)


            dx = waypoint_pps[0] - position[0]
            dy = waypoint_pps[1] - position[1]

            y = dx*math.cos(position[2])+dy*math.sin(position[2])
            x = (-dx*math.sin(position[2])+dy*math.cos(position[2]))

            local_waypoint = [y,x]
            # print("pps:",self.look_ahead_for_pps)
        except Exception as e: 
            print(e)            
            target_speed = 0
            local_waypoint = [None,None]
        return target_speed, local_waypoint



class VehiclePIDController():

    def __init__(self,args_Lateral , args_LongitudnaL , max_throttle = 1.0, max_break=1.0, max_steering=1.0):
        self.max_break = max_break
        self.max_steering = max_steering
        self.max_throttle = max_throttle
        
        self.long_controller = PIDLongitudnalControl( **args_LongitudnaL)
        self.lat_controller = PurePursuitLateralControl( **args_Lateral)

    def run_step(self, control, target_speed, waypoint):
        acceleration = self.long_controller.run_step(target_speed)
        current_steering = self.lat_controller.run_step(waypoint)

        if acceleration >=0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0

        else :
            control.throttle = 0.0
            control.brake = min(abs(acceleration),self.max_break)

        control.steer = current_steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = current_steering

        return control

    def run_step_accel(self, control, target_speed):
        acceleration = self.long_controller.run_step(target_speed)

        if acceleration >=0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0

        else :
            control.throttle = 0.0
            control.brake = min(abs(acceleration),self.max_break)

        control.hand_brake = False
        control.manual_gear_shift = False

        return control


class PIDLongitudnalControl():

    def __init__(self, K_P=1.0, K_I=0.0, K_D=0.0, dt = 0.03):

        self.K_D = K_D
        self.K_P = K_P
        self.K_I = K_I
        self.dt = dt
        self.errorBuffer = queue.deque(maxlen = 10)
        
    def pid_controller(self,target_speed, current_speed):
        error = target_speed-current_speed
        self.errorBuffer.append(error)

        if len(self.errorBuffer)>=2:
            de = (self.errorBuffer[-1] -self.errorBuffer[-2])/self.dt
            ie = sum(self.errorBuffer)*self.dt
        else:
            de=0.0
            ie=0.0

        return np.clip(self.K_P*error+self.K_D*de+self.K_I*ie, -1.0,1.0)

    def run_step(self, target_speed):
        global speed 
        # print(speed)
        current_speed = speed
        return self.pid_controller(target_speed,current_speed)


class PurePursuitLateralControl():
    def __init__(self, wheel_base = 2.8, max_steer = 70.0):

        self.wheel_base = wheel_base
        self.max_steer = max_steer
    
    def run_step(self,waypoint):
        return self.pure_pursuit_controller(waypoint)
        #https://gist.github.com/chardorn/aeeafa5b5e357d9690361701804b432b#file-custom_waypoints_control-py

    def pure_pursuit_controller(self,waypoint):

        try:
            y = waypoint[0]
            x = waypoint[1]

            # dist = ((position[0] - waypoint[0])**2 + (position[1] - waypoint[1])**2)**0.5
            # print(round(position[0]),round(waypoint[0]), round(position[1]),round(waypoint[1]), (math.degrees(position[2])))
        
            Ld2 = x**2 + y**2

            steer_rad = math.atan(2 * self.wheel_base * x / Ld2)
            # print("LD:",Ld2**0.5)
        except:
            steer_rad = 0.0

        steer_deg = math.degrees(steer_rad)
        steer_deg = np.clip(steer_deg, -self.max_steer, self.max_steer)
        
        # steer_deg = 0
        return -steer_deg / self.max_steer

   

def main():

    local_planner = test.LocalPlanner(5,1)


    waypoint = Waypoint()

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber("/carla/ego_vehicle/speedometer",Float32,speed_callback)
    rospy.Subscriber("/carla/ego_vehicle/odometry",Odometry,waypoint.odometry_callback)
    rospy.Subscriber("/carla/ego_vehicle/waypoints",Path,waypoint.path_callback)
    rospy.Subscriber("detected_objects",DetectedObjectArray,waypoint.object_callback)
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,waypoint.traffic_callback)
    # rospy.Subscriber("/")




    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
    pub_lattice = rospy.Publisher('/carla/ego_vehicle/lattice', Path, queue_size=10)

    control_vehicle = VehiclePIDController(args_Lateral={'wheel_base':2.8, 'max_steer':70},args_LongitudnaL={'K_P':2,'K_D':0.0,'K_I':0.0})


    control = CarlaEgoVehicleControl()
    paths_msg = Path()
    paths_msg.header.frame_id = "ego_vehicle"

    while not rospy.is_shutdown():
        target_speed = 0

        if len(waypoint.waypoints_)>0:
            waypoint.waypoint_index()
            target_speed, waypoint_pps = waypoint.set_target_speed()

            try:
                # start_time = time.time()
                # print(waypoint.waypoints_[waypoint.way_ind])
                paths_ = local_planner.get_goal_state_set( waypoint.way_ind+waypoint.look_ahead_for_ts,waypoint.waypoints_[waypoint.way_ind+waypoint.look_ahead_for_ts], waypoint.waypoints_,waypoint.position_)
                paths, path_validity = local_planner.plan_paths(paths_)

                path_point =  (19 * waypoint.look_ahead_for_pps) // waypoint.look_ahead_for_ts
                print(len(paths))
                valid_ind, path_valid, object_distance_min = waypoint.cost_function(paths)

                if path_valid == False:     # lattice paths are not valid. Conduct cruise control
                    if valid_ind == None:   # no lattice path
                        print("no lattice path")
                        cruise_speed = 0
                    else:                   #lattice paths are in collision
                        cruise_speed = waypoint.cruise_control(object_distance_min)
                        print("lattice collision")
                    
                    target_speed = min(cruise_speed,target_speed)
                intersection_speed = waypoint.intersection_conrol()

                target_speed = min(intersection_speed,target_speed)


                waypoint_pps = [paths[valid_ind][0][path_point],paths[valid_ind][1][path_point]]
                # if path_valid == False:
                #     target_speed = 0
                paths_msg.poses = []


                for i in range(len(paths)):
                    # print(len(paths[i][0]))

                    for j in range(len(paths[i][0])):
                        path_msg = PoseStamped()

                        path_msg.pose.position.x = (paths[i][0][j])
                        path_msg.pose.position.y = (paths[i][1][j])
                        path_msg.pose.position.z = 0
                        paths_msg.poses.append(path_msg)

                # print((time.time() - start_time))
            except Exception as e: 
                print(e)            
            
            control_vehicle.run_step(control,target_speed,waypoint_pps)

        # # if measured_distance < 30:
            
        # #     target_distance = 10+timegap*(get_speed(vehicle))
        # #     distance_error = target_distance - measured_distance
        # #     target_speed =get_speed(vehicle) - (distance_error)/timegap;


        # #     if get_speed(dynamic_obs)- get_speed(vehicle) < -8.5 :
        # #         target_speed = 0


        if (waypoint.waypoint_next(10))[0] == None: 
            target_speed = 0
            print("NO WAYPOINT",waypoint.position_)
            # waypoint.way_ind = 0
            control_vehicle.run_step_accel(control,target_speed)

        
        pub.publish(control)
        pub_lattice.publish(paths_msg)

        #     # print(target_speed,waypoint)



        

        

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    main()
