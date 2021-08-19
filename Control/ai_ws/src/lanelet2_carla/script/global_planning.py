#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import numpy as np
import lanelet2
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
from std_msgs.msg import Int32

class GlobalPlanning():
    def __init__(self):
        self.pub_waypoint = rospy.Publisher('/carla/ego_vehicle/waypoints',Path,queue_size=1)
        self.mapfile_path = "/home/jaeyoung/catkin_ws/Town01.osm"
        # self.osm_path = os.path.join(os.path.dirname(os.path.abspath('')), "Town01.osm")
        # self.lorigin = lanelet2.io.Origin(113.99336193330966, 333.595688874, 0.0) #node 7550 37.9970032767 125.001162728768
        # self.lanelet_map = lanelet2.io.load(self.osm_path, self.lorigin)

        # # remove non-compliant matches (such as driving in the wrong direction)
        # traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        # self.graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules)
        print("hello")
    
    def info(self):

        osm_path = os.path.join(os.path.dirname(os.path.abspath('')), self.mapfile_path)
        print("######")
        print(osm_path)
        lorigin = lanelet2.io.Origin(113.99336193330966, 333.595688874, 0.0) #node 7550 37.9970032767 125.001162728768
        lanelet_map = lanelet2.io.load(osm_path, lorigin)

        # remove non-compliant matches (such as driving in the wrong direction)
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
        graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules)
        
        return lanelet_map,graph

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



    def pipeline(self,lanelet_map,graph):
        Waypoints = Path()
        # end_lane_num = input("type end lane: ")
        # start_lane_num = current_lanelet
        startLane = lanelet_map.laneletLayer[8000] # lanelet IDs 8000     13663
        endLane = lanelet_map.laneletLayer[13525] #13525       14011
        route = graph.getRoute(startLane, endLane)

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
        for list_index in range(len(start_end_list)):
            for line_index in range(start_end_list[list_index][0],start_end_list[list_index][1]+1):
                new_line=lines[line_index].replace("\""," ")
                line_split = new_line.split()
                lat = float(line_split[8])
                lon = float(line_split[10])  

                x,y = self.gps2plane(lat,lon,38.0,125.0)
                # x,y = self.gps2plane2(lat-38.0,lon-125.0)
                y *=-1.0

                if x_prev and y_prev and x_prev != x and y_prev != y:

                    dx = x - x_prev
                    dy = y - y_prev
                    # distance = (dx**2 + dy**2)**0.5
                    theta = np.arctan2(dy,dx)
                    offset = 1.8
                    
                    x_offset =  offset * np.sin(theta)
                    y_offset =  -offset * np.cos(theta)

                    x_real = x + x_offset
                    y_real = y + y_offset

                    x_prev = x
                    y_prev = y
                    
                    new_file.write(str(x_real)+" "+str(y_real)+" 50.0\n")  
                    # new_file.write("                                    "+str(distance)+"\n")  
                    Waypoint = PoseStamped()
                    Waypoint.pose.position.x = x_real
                    Waypoint.pose.position.y = y_real
                    
                    quaternion_x,quaternion_y,quaternion_z,quaternion_w = quaternion_from_euler(0,0,theta)
                    Waypoint.pose.orientation.w = quaternion_w
                    Waypoints.poses.append(Waypoint)

                x_prev = x
                y_prev = y
                
        print("end")
          
        new_file.close()
        map_file.close()
        self.pub_waypoint.publish(Waypoints)
        

if __name__ == '__main__':
    rospy.init_node('GlobalPlanning')
    
    node = GlobalPlanning()
    lanelet_map, graph = node.info()
    node.pipeline(lanelet_map,graph)
    # node.main()
    rospy.spin()