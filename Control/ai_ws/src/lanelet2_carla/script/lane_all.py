# -*- coding: utf-8 -*-
import os
import math
import numpy as np
import lanelet2
import lanelet2.core as lncore

def gps2plane(lat,lon,lat_r,lon_r):
    earth_radius = 6378137.0
    lon_theta = (lon-lon_r)*np.pi/180.0
    lat_theta = (lat-lat_r)*np.pi/180.0
    x = earth_radius*np.cos(lat*np.pi/180.0)*(lon_theta)
    y = earth_radius*(lat_theta)
    return x,y

new_file = open("waypoint.txt",'w')
map_file = open("Town01.osm")
lines = map_file.readlines()

x_prev = None
y_prev = None
for i in range(3,20494):
    new_line = lines[i].replace("\""," ")
    lat = float(new_line.split()[8])
    lon = float(new_line.split()[10])

    x,y = gps2plane(lat,lon,38.0,125.0)
    y *=-1.0

    if x_prev and y_prev:

        dx = x - x_prev
        dy = y - y_prev
        distance = (dx**2 + dy**2)**0.5
        theta = np.arctan2(dy,dx)
        offset = 1.5
        
        x_offset = - offset * np.sin(theta)
        y_offset = offset * np.cos(theta)

        x_real = x 
        y_real = y 


        new_file.write(str(x_real)+" "+str(y_real)+" 10.0\n")  
    
    x_prev = x
    y_prev = y

new_file.close()
map_file.close()