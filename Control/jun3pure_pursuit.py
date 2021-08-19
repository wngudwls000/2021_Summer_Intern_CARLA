#!/usr/bin/env python

import sys
import math
import os
import glob
import numpy as np
import queue
import cv2



try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla




def processImage(image):

    image = np.array(image.raw_data)
    img = image.reshape((180,240,4))

    img = img[:,:,:3]
    # print(img)
    cv2.imshow("img",img)
    cv2.waitKey(1)

def processGnss(gps):
    # print(gps)
    pass

def processObstacle(obs):
    # obs.distance
    # obs.transform
    pass
    # print(obs.transform)
    

def get_speed(vehicle):

    vel = vehicle.get_velocity()
    # km/h
    vel = 3.6*math.sqrt(vel.x**2 + vel.y**2+ vel.z**2)
    # print(vel)
    return vel

class VehiclePIDController():

    def __init__(self,vehicle,args_Lateral , args_LongitudnaL , max_throttle = 0.75, max_break=0.3, max_steering=0.8):
        self.max_break = max_break
        self.max_steering = max_steering
        self.max_throttle = max_throttle
        
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.past_steering = self.vehicle.get_control().steer 
        self.long_controller = PIDLongitudnalControl(self.vehicle , **args_LongitudnaL)
        self.lat_controller = PurePursuitLateralControl(self.vehicle, **args_Lateral)

    def run_step(self, target_speed, waypoint):
        acceleration = self.long_controller.run_step(target_speed)
        current_steering = self.lat_controller.run_step(waypoint)
        control = carla.VehicleControl()

        if acceleration >=0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0

        else :
            control.throttle = 0.0
            control.brake = min(abs(acceleration),self.max_break)


        # if current_steering > self.past_steering+0.1:
        #     current_steering = self.past_steering+0.1

        # elif current_steering < self.past_steering-0.1:
        #     current_steering = self.past_steering-0.1
        
        # if current_steering >= 0:
        #     steering = min(self.max_steering,current_steering)

        # else:
        #     steering = max(-self.max_steering, current_steering)

        # print(current_steering)
        control.steer = current_steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = current_steering

        return control



class PIDLongitudnalControl():

    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt = 0.03):

        self.vehicle = vehicle
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
        current_speed = get_speed(self.vehicle)
        return self.pid_controller(target_speed,current_speed)





class PurePursuitLateralControl():
    def __init__(self, vehicle, wheel_base, max_steer):

        self.vehicle = vehicle
        self.wheel_base = wheel_base
        self.max_steer = max_steer
    
    def run_step(self,waypoint):

        return self.pure_pursuit_controller(waypoint, self.vehicle.get_transform())
        #https://gist.github.com/chardorn/aeeafa5b5e357d9690361701804b432b#file-custom_waypoints_control-py

    def pure_pursuit_controller(self,waypoint, vehicle_transform):
        origin = vehicle_transform.location
        forward = vehicle_transform.get_forward_vector()
        right = vehicle_transform.get_right_vector()
        up = vehicle_transform.get_up_vector()
        disp = waypoint.transform.location - origin
        x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
        y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
        z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])
        relative_location =  carla.Vector3D(x, y, z)

        wp_loc_rel = relative_location+ carla.Vector3D(self.wheel_base, 0, 0)
        wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
        d2 = wp_ar[0]**2 + wp_ar[1]**2
        steer_rad = math.atan(2 * self.wheel_base * wp_loc_rel.y / d2)
        steer_deg = math.degrees(steer_rad)
        steer_deg = np.clip(steer_deg, -self.max_steer, self.max_steer)
        # print(steer_deg, self.max_steer)
        return steer_deg / self.max_steer

# class PIDLateralControl():

#     def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0 , dt = 0.03):

#         self.vehicle = vehicle
#         self.K_D = K_D
#         self.K_P = K_P
#         self.K_I = K_I
#         self.dt = dt
#         self.errorBuffer = queue.deque(maxlen = 10)
        
#     def run_step(self,waypoint):

#         return self.pid_controller(waypoint, self.vehicle.get_transform())

#     def pid_controller(self,waypoint, vehicle_transform):

#         v_begin = vehicle_transform.location
#         v_end = v_begin+carla.Location(x= math.cos(math.radians(vehicle_transform.rotation.yaw)), y =math.sin(math.radians(vehicle_transform.rotation.yaw)))
        

#         v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y,0.0])
#         ## relative distance between ego_vehicle and look_ahead waypoint
#         w_vec = np.array([waypoint.transform.location.x - v_begin.x, waypoint.transform.location.y - v_begin.y, 0.0])

#         dot = math.acos(np.clip(np.dot(w_vec,v_vec)/np.linalg.norm(w_vec)*np.linalg.norm(v_vec),-1.0,1.0))
       
#         print("")
#         cross = np.cross(v_vec, w_vec)
#         if cross[2]<0:
#             dot *=-1

#         self.errorBuffer.append(dot)

#         if len(self.errorBuffer)>=2:
#             de = (self.errorBuffer[-1]-self.errorBuffer[-2]/self.dt)
#             ie = sum(self.errorBuffer)*self.dt
        
#         else:
#             de = 0.0
#             ie = 0.0

#         return np.clip((self.K_P*dot)+(self.K_I*ie)+(self.K_D*de) , -1.0,1.0)




def main():
    actor_list = []
    try :
        client = carla.Client('192.168.1.8',2000)
        # client = carla.Client('localhost',2000)
        client.set_timeout(5.0)
        print(client.get_available_maps())
        world = client.load_world('Town01')
        # world = client.load_world('Town07_Opt')
        # world.unload_map_layer(carla.MapLayer.All)
        map = world.get_map()
        world.set_weather(carla.WeatherParameters.ClearSunset)
        spectator = world.get_spectator()

        blueprint_library = world.get_blueprint_library()
        vehicle_bp =  blueprint_library.filter('vehicle.*model3*')[0]
        spawnpoint = carla.Transform(carla.Location(x=210.0,y=200.0,z=15),carla.Rotation(pitch = 0, yaw = 0, roll =0 ))
        # spawnpoint = carla.Transform(carla.Location(x=-198.0,y=-171.0,z=5),carla.Rotation(pitch = 0, yaw = 270, roll =0 ))
        # spawnpoints = world.get_map().get_spawn_points()
        # spawnpoint = np.random.choice(spawnpoints)
        # print(spawnpoints)
        vehicle = world.spawn_actor(vehicle_bp, spawnpoint)
        actor_list.append(vehicle)


        #Vehicle properties setup
        physics_control = vehicle.get_physics_control()
        for wheel in physics_control.wheels:
            print (wheel.max_steer_angle)
        max_steer = physics_control.wheels[0].max_steer_angle/2
        rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
        offset = rear_axle_center - vehicle.get_location()
        wheelbase = 2*np.linalg.norm([offset.x, offset.y, offset.z])
        print(max_steer,wheelbase)

        control_vehicle = VehiclePIDController(vehicle, args_Lateral={'wheel_base':wheelbase, 'max_steer':max_steer},args_LongitudnaL={'K_P':2,'K_D':0.0,'K_I':0.0})
        
        # # spawn the camera and attach to vehicle.
        # camera_bp = blueprint_library.find('sensor.camera.rgb')
        # camera_bp.set_attribute('image_size_x', '240')
        # camera_bp.set_attribute('image_size_y', '180')
        # camera_bp.set_attribute('fov', '90')
        # camera_tramsform = carla.Transform(carla.Location(x=1.5, z=4.7))
        # camera = world.spawn_actor(camera_bp, camera_tramsform, attach_to=vehicle)
        # camera.listen(lambda image: processImage(image))
        # actor_list.append(camera)


        # # spawn the gnss and attach to vehicle.
        # gnss_bp = blueprint_library.find('sensor.other.gnss')
        # gnss_tramsform = carla.Transform(carla.Location(x=1.5, z=0.7))
        # gnss = world.spawn_actor(gnss_bp, gnss_tramsform, attach_to=vehicle)
        # gnss.listen(lambda gps: processGnss(gps))
        # actor_list.append(gnss)


        ## spawn obstacle detector
        obs_bp = blueprint_library.find('sensor.other.obstacle')
        obs_tramsform = carla.Transform(carla.Location(x=2.5, z=0.3))
        obs = world.spawn_actor(obs_bp, obs_tramsform, attach_to=vehicle)
        obs.listen(lambda obstacle: processObstacle(obstacle))
        actor_list.append(obs)

        # ## spawn static obstacle
        # static_obs_bp = (blueprint_library.filter('vehicle.bmw.*'))[1]
        # # spawnpoint = carla.Transform(carla.Location(x=210.0,y=200.0,z=15),carla.Rotation(pitch = 0, yaw = 0, roll =0 ))
        # spawnpoint = carla.Transform(carla.Location(x=-20.0,y=-243.0,z=5),carla.Rotation(pitch = 0, yaw = 11.7, roll =0 ))
        # # spawnpoints = world.get_map().get_spawn_points()
        # # spawnpoint = np.random.choice(spawnpoints)
        # # print(spawnpoints)
        # static_obs = world.spawn_actor(static_obs_bp, spawnpoint)
        # actor_list.append(static_obs)
        # static_obs.apply_control(carla.VehicleControl(hand_brake = True))
        while True:
            
            waypoints = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
            # waypoint = np.random.choice(waypoints.next(10.0))
            waypoint = waypoints.next(8.0)[0]
            # world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
            #                            color=carla.Color(r=120, g=0, b=0), life_time=0.01,
            #                            persistent_lines=True)
            # world.debug.draw_string(vehicle.get_location(), 'O', draw_shadow=False,
            #                            color=carla.Color(r=0, g=50, b=0), life_time=120,
            #                            persistent_lines=True)
            # world.debug.draw_string(waypoints.transform.location, 'O', draw_shadow=False,
            #                            color=carla.Color(r=0, g=0, b=120), life_time=120,
            #                            persistent_lines=True)
        
            # print(np.random.choice(waypoints.next(1.0)))
            # print(vehicle.get_transform())
            # print(waypoint.transform.location)
            # print("")

            if vehicle.is_at_traffic_light():
                print("LIGHT")
                traffic_light = vehicle.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    print("RED")
                    # traffic_light.set_state(carla.TrafficLightState.Green)

            control_signal = control_vehicle.run_step(40,waypoint)
            vehicle.apply_control(control_signal)

            location = carla.Location(x = 0, y=0, z = 50.0) + vehicle.get_location()
            spectator_transform = carla.Transform(location, carla.Rotation(pitch=-90))
            spectator.set_transform(spectator_transform)
            # print("")
    finally :
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
if __name__ == '__main__':

    main()
