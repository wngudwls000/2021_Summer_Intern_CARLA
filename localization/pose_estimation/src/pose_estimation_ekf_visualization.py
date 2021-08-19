#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from error_msg_file.msg import Error

class Visualiser:
    def __init__(self):
        self.fig1, self.ax1 = plt.subplots()
        self.ln_real, = plt.plot([], [], 'b.')
        self.ln_estimate, = plt.plot([], [], 'r.')
        self.ln_gnss, = plt.plot([], [], 'g.')
        self.real_pose_x, self.real_pose_y = [], []
        self.estimate_pose_x, self.estimate_pose_y = [], []
        self.gnss_x, self.gnss_y = [], []

        self.fig2, self.ax2 = plt.subplots()
        self.ln_error, = plt.plot([], [], 'b.')
        self.error, self.time = [], []

        self.fig3, self.ax3 = plt.subplots()
        self.ln_error_mean, = plt.plot([], [], 'r')
        self.error_mean = []

    def real_estimate_plot_init(self):
        self.ax1.set_xlim(-20, 400)
        self.ax1.set_ylim(-400, 20)
        self.ax1.grid()
        self.ax1.set_xlabel('local map x [meter]')
        self.ax1.set_ylabel('local map y [meter]')
        self.ax1.set_title('carla local map')
        self.ax1.legend(['real pose', 'estimate_pose', 'gnss'])
        return self.ln_real

    def error_plot_init(self):
        self.ax2.set_xlim(0, 50)
        self.ax2.set_ylim(-1, 5)
        self.ax2.grid()
        self.ax2.set_xlabel('time')
        self.ax2.set_ylabel('error [meter]')
        self.ax2.set_title('error')
        self.ax2.legend(['error'])
        return self.ln_error

    def error_mean_plot_init(self):
        self.ax3.set_xlim(0, 50)
        self.ax3.set_ylim(-1, 5)
        self.ax3.grid()
        self.ax3.set_xlabel('time')
        self.ax3.set_ylabel('error_mean [meter]')
        self.ax3.set_title('error_mean')
        self.ax3.legend(['error_mean'])
        return self.ln_error_mean

    def Odom_Callback(self, odom_data):
        odom_pose_x = odom_data.pose.pose.position.x
        odom_pose_y = odom_data.pose.pose.position.y
        self.real_pose_x.append(odom_pose_x)
        self.real_pose_y.append(odom_pose_y)

    def Car_pose_Callback(self, car_odometry_data):
        pose_x = car_odometry_data.pose.pose.position.x
        pose_y = car_odometry_data.pose.pose.position.y
        gnss_x = car_odometry_data.twist.twist.linear.x
        gnss_y = car_odometry_data.twist.twist.linear.y
        self.estimate_pose_x.append(pose_x)
        self.estimate_pose_y.append(pose_y)
        self.gnss_x.append(gnss_x)
        self.gnss_y.append(gnss_y)

    def Error_Callback(self, error_data):
        result_time_sec = error_data.header.stamp.secs
        result_time_nsec = float(error_data.header.stamp.nsecs) / 1000000000.0
        result_time = result_time_sec + result_time_nsec
        result_error = error_data.error
        self.time.append(result_time)
        self.error.append(result_error)
        self.error_mean.append(np.mean(self.error))
    
    def update_plot_real(self, frame):
        self.ln_real.set_data(self.real_pose_x, self.real_pose_y)
        return self.ln_real

    def update_plot_estimate(self, frame):
        self.ln_estimate.set_data(self.estimate_pose_x, self.estimate_pose_y)
        return self.ln_estimate

    def update_plot_gnss(self, frame):
        self.ln_gnss.set_data(self.gnss_x, self.gnss_y)
        return self.ln_gnss

    def update_plot_error(self, frame):
        self.ln_error.set_data(self.time, self.error)
        return self.ln_error

    def update_plot_error_mean(self, frame):
        self.ln_error_mean.set_data(self.time, self.error_mean)
        return self.ln_error_mean

def start():
    print("visualization running")
    visual = Visualiser()
    rospy.init_node('visualization', anonymous=True)
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, visual.Odom_Callback)
    rospy.Subscriber('/car_odometry', Odometry, visual.Car_pose_Callback)
    rospy.Subscriber('/error', Error, visual.Error_Callback)
    ani_real = FuncAnimation(visual.fig1, visual.update_plot_real, init_func=visual.real_estimate_plot_init)
    ani_estimate = FuncAnimation(visual.fig1, visual.update_plot_estimate, init_func=visual.real_estimate_plot_init)
    ani_gnss = FuncAnimation(visual.fig1, visual.update_plot_gnss, init_func=visual.real_estimate_plot_init)
    ani_error = FuncAnimation(visual.fig2, visual.update_plot_error, init_func=visual.error_plot_init)
    ani_error_mean = FuncAnimation(visual.fig3, visual.update_plot_error_mean, init_func=visual.error_mean_plot_init)
    plt.show(block=True)


if __name__ == '__main__':
    start()
    rospy.spin()