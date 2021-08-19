#!/usr/bin/env python
# -*- coding: utf-8 -*-
from genpy import rostime
import rospy
import os
import numpy as np
import lanelet2
import tf
# import lanelet2.core as lncore
# from carla_msgs.msg import Waypoint
# from carla_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

count = 0

class GlobalPlanning():
    def __init__(self):
        self.sub_odometry = rospy.Subscriber('/car_odometry',Odometry,self.car_odometry,queue_size = 1)
        self.sub_goallane = rospy.Subscriber('/goal_lat_lon',Float64MultiArray,self.pipeline,queue_size = 1)

        self.pub_waypoint = rospy.Publisher('/carla/ego_vehicle/waypoints',Path,queue_size=1)
        self.mapfile_path = "/home/labdog/ai_ws/src/lanelet2_carla/script/Town01.osm"

        self.osm_path = os.path.join(os.path.dirname(os.path.abspath('')), self.mapfile_path)
        self.lorigin = lanelet2.io.Origin(38.0, 125.0, 0.0) #node 7550 37.9970032767 125.001162728768
        self.lanelet_map = lanelet2.io.load(self.osm_path, self.lorigin)

        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules)

        self.car_x = None
        self.car_y = None

        print("__init__")
    
    def plane2gps(self,x,y):
        # y =  (-1.0)*y
        earth_radius = 6378137.0
        lat_theta = y/earth_radius
        lon_theta = x/(earth_radius*np.cos(lat_theta))
        lat = lat_theta*180.0/np.pi
        lon = lon_theta*180.0/np.pi
        
        return lat,lon

        

    def car_odometry(self,odometry_msg):

        self.car_x = odometry_msg.pose.pose.position.x
        self.car_y = odometry_msg.pose.pose.position.y
        

    # def find_closest_node(self,find_lat,find_lon):
    #     map_file = open(self.mapfile_path)
    #     lines = map_file.readlines()
        
    #     for i in range(len(lines)): 
    #         new_line=lines[i].replace("\""," ")
    #         line_split=new_line.split()

    #         if len(line_split)>9:
    #             lat = float(line_split[8])
    #             lon = float(line_split[10])
    #             if abs(lat-find_lat)<0.00005 and abs(lon-find_lon)<0.00005:
    #                 return int(line_split[2])

    def find_closest_node(self,find_lat,find_lon):
        map_file = open(self.mapfile_path)
        lines = map_file.readlines()
        node = None
        way_id = None
        for i in range(len(lines)): 
            new_line=lines[i].replace("\""," ")
            line_split=new_line.split()

            if len(line_split)>9:
                lat = float(line_split[8])
                lon = float(line_split[10])
                if abs(lat-find_lat)<0.00003 and abs(lon-find_lon)<0.00003:
                    node = line_split[2]
            if node and len(line_split)>=3 and line_split[0] != "<node":
                if line_split[0] =="<way":
                    way_id_temp = line_split[2]
                if line_split[2] == node:
                    way_id = way_id_temp
            if way_id and len(line_split)>6 and line_split[6] == "right":
                return int(node)

    def node2lane(self,find_lat,find_lon):

        node_id = self.find_closest_node(find_lat,find_lon)
        print("node id: ",node_id)
        way_id = 0
        lane_id = 0
        map_file = open(self.mapfile_path)

        lines = map_file.readlines()
        
        for i in range(len(lines)): 
            line_split=lines[i].split()
            if line_split[0] == "<way":
                new_line=lines[i].replace("\""," ")
                way_id = new_line.split()[2] 
            if len(line_split)>1 and line_split[1] == "ref=\"" + str(node_id) + "\"/>":
                print("way_id: ",way_id)
                break
        
        for i in range(len(lines)): 
            line_split=lines[i].split()
            if line_split[0] == "<relation":
                new_line=lines[i].replace("\""," ")
                lane_id = new_line.split()[2] 

            if len(line_split)>2 and line_split[2] == "ref=\"" + str(way_id) + "\"":
                print("lane_id: ",lane_id)
                break 

        map_file.close()
        return int(lane_id),node_id



    def gps2plane(self,lat,lon,lat_r,lon_r):
        earth_radius = 6378137.0
        lon_theta = (lon-lon_r)*np.pi/180.0
        lat_theta = (lat-lat_r)*np.pi/180.0
        x = earth_radius*np.cos(lat*np.pi/180.0)*(lon_theta)
        y = earth_radius*(lat_theta)
        return x,y
    def gps2plane2(self,lat,lon):
        earth_radius = 6378137.0
        lon_theta = (lon)*np.pi/180.0
        lat_theta = (lat)*np.pi/180.0
        x = earth_radius*np.cos(lat_theta)*(lon_theta)
        y = earth_radius*lat_theta
        return x,y



    def pipeline(self,goal_node):
        global count
        test_lat,test_lon = self.plane2gps(208,-199)
        print("test gps:",38.0+test_lat , 125.0+test_lon)
        # test_x,test_y = self.gps2plane(37.9982123525846,125.00186849579188,38.0,125.0)
        test_x,test_y = self.gps2plane2(37.9982123525846-38.0,125.00186849579188-125.0)
        print(test_x,test_y)

        Waypoints = Path()
        Waypoints.header.frame_id = "map"
        Waypoints.header.stamp = rospy.Time.now()
        # broadcaster = tf.TransformBroadcaster() 

        if self.car_x:
            start_lat, start_lon = self.plane2gps(self.car_x,self.car_y)
            end_lat, end_lon = self.plane2gps(goal_node.data[0],goal_node.data[1])

            print("start_lat: ",start_lat+38.0, " start_lon: ",start_lon+125.0)
            print("goal_lat: ",end_lat+38.0,"goal_lon: ",end_lon+125.0)


            start_lane_id ,start_node_id= self.node2lane(start_lat + 38.0,start_lon + 125.0)
            goal_lane_id ,end_node_id= self.node2lane(end_lat + 38.0,end_lon +125.0)

            print("start lane_id: ",start_lane_id)
            print("goal lane_id: ",goal_lane_id)
            startLane = self.lanelet_map.laneletLayer[start_lane_id] # lanelet IDs 8000     13663
            endLane = self.lanelet_map.laneletLayer[goal_lane_id] #13525       14011

            route = self.graph.getRoute(startLane, endLane)

            path_list=[]
            if route is None:
                print("error: no route was calculated")
            else:
                shortest_path = route.shortestPath()

                if shortest_path is None:
                    print ("error: no shortest path was calculated")
                else:
                    # print [lanelet.id for lanelet in shortest_path.getRemainingLane(startLane)] if sp else None

                    for lanelet in shortest_path.getRemainingLane(startLane):
                        path_list.append(lanelet.id)
                print(path_list)
            # ll=lanelet_map.laneletLayer[13525]

            # making waypoint
            new_file = open("waypoint_right.txt",'w')
            point_num_list=[]
            point_pair_list=[]
            for path in path_list:

                map_file = open(self.mapfile_path)

                lines = map_file.readlines()
                for i in range(len(lines)): #lanelet에 대한 우측 line id  저장
                    line_split=lines[i].split()
                    for j in range(len(line_split)):
                        if line_split[j] == "id=\"" + str(path) + "\"":
                            new_line=lines[i+1].replace("\""," ")
                            line_id=new_line.split()[4]

                map_file.close()

                map_file = open(self.mapfile_path)
                lines = map_file.readlines()
                line_num = None
                for i in range(len(lines)): # line id 에 대한 point 번호 위치 추출
                    line_split=lines[i].split()
                    for j in range(len(line_split)):
                        if line_split[j] == "id=\"" + str(line_id) + "\"":
                            # print("line_id: ",line_id)
                            line_num = i+1
                    
                    if line_split[0] == "</way>" and line_num:
                        line_end_num = i-1
                        break

                for i in range(line_num,line_end_num + 1):

                    new_line = lines[i].replace("\""," ")
                    point_num = int(new_line.split()[2])
                    # print("point_num: ", point_num)
                    # new_file.write(point_num + "\n")
                    point_num_list.append(point_num)
                    #### new
                    if i == line_num+1: #첫포인트 포기하고 속도 챙기기
                        start_point_num = point_num
                    elif i == line_end_num-1:
                        end_point_num = point_num
                

                ####new
                point_pair = (start_point_num,end_point_num)
                point_pair_list.append(point_pair)
                map_file.close()
            
            print(point_pair_list)
            map_file = open(self.mapfile_path)

            lines = map_file.readlines()

            x_prev = None
            y_prev = None
            x_real_prev = None
            y_real_prev = None

            # point pair 에 대해서 시작포인트 끝포인트 위치 저장
            start_end_list=[]
            for list_index in range(len(point_pair_list)):
                for point_index in range(len(lines)):
                    new_line=lines[point_index].replace("\""," ")
                    line_split = new_line.split()

                    if len(line_split)>9:
                        if  line_split[2]==str(point_pair_list[list_index][0]):
                            start_line = point_index
                        elif  line_split[2]==str(point_pair_list[list_index][1]):
                            end_line = point_index              
                start_end = (start_line,end_line)
                start_end_list.append(start_end)
            print(start_end_list)

            # line 단위로 시작포인트부터 끝포인트까지 write
            send_start = False
            send_end = False
            print("end_node: ",end_node_id)
            for list_index in range(len(start_end_list)):
                for line_index in range(start_end_list[list_index][0],start_end_list[list_index][1]+1):
                    new_line=lines[line_index].replace("\""," ")
                    line_split = new_line.split()
                    lat = float(line_split[8])
                    lon = float(line_split[10])  

                    x,y = self.gps2plane(lat,lon,38.0,125.0)
                    # x,y = self.gps2plane2(lat-38.0,lon-125.0)
                    # y *=-1.0

                    if x_prev and y_prev and x_prev != x and y_prev != y :

                        dx = x - x_prev
                        dy = y - y_prev
                        
                        theta = np.arctan2(dy,dx)
                        offset = 2.0
                        
                        x_offset =  -offset * np.sin(theta)
                        y_offset =  offset * np.cos(theta)

                        x_real = x + x_offset
                        y_real = y + y_offset
                        
                        x_prev = x
                        y_prev = y
                        if x_real_prev and y_real_prev:
                            distance = ((x_real-x_real_prev)**2+(y_real-y_real_prev)**2)**0.5
                        else:
                            distance  = 0
                        x_real_prev = x_real
                        y_real_prev = y_real
                        
                        new_file.write(str(x_real)+" "+str(y_real)+" "+str(distance)+"\n")  
                        
                        if line_split[2] == str(start_node_id):
                            send_start = True
                            print("#######################send Start")
                        if line_split[2] == str(end_node_id):
                            send_end = True
                            print("#######################send END!")
                        
                        if send_start and send_end is False:
                            # new_file.write("                                    "+str(distance)+"\n")  
                            Waypoint = PoseStamped()
                            Waypoint.pose.position.x = x_real
                            Waypoint.pose.position.y = y_real
                            
                            quaternion_x,quaternion_y,quaternion_z,quaternion_w = quaternion_from_euler(0,0,theta)
                            Waypoint.pose.orientation.x = quaternion_x
                            Waypoint.pose.orientation.y = quaternion_y
                            Waypoint.pose.orientation.z = quaternion_z
                            Waypoint.pose.orientation.w = quaternion_w
                            Waypoints.poses.append(Waypoint)

                    x_prev = x
                    y_prev = y
                    
            print("end")
            
            new_file.close()
            map_file.close()
            
            self.pub_waypoint.publish(Waypoints)
            print(count)
            count+=1

        # transform = tf.Transform()
        # transform.setOrigin(tf.Vector3(0.0, 0.0, 0.0))
        # q = tf.Quaternion()
        # q.setX(0)
        # q.setY(0)
        # q.setZ(0)
        # q.setW(0)
        # transform.setRotation(q)
        # broadcaster.sendTransform(tf.StampedTransform(transform, rospy.Time.now(),"world", "base_link"))
        # broadcaster.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"base_link","map")
        

if __name__ == '__main__':
    rospy.init_node('GlobalPlanning')
    
    node = GlobalPlanning()
    # node.main()
    rospy.spin()
