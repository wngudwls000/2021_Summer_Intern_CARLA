#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#include "error_msg_file/Error.h"

#define Only_Prediction false
#define Only_GNSS false

class EKF{
public:
    EKF();
    Eigen::Matrix<double, 3, 1> Dead_Reckoning(Eigen::Matrix<double, 3, 1> x_pri, double vel, double angular_vel_z, double dt);
    void Pose_Estimation_EKF();
    void Calculate_Error(double estimate_x, double real_x, double estimate_y, double real_y);
    void SetPose(double pose_x, double pose_y, double pose_yaw);
    void SetPose2(double pose_x, double pose_y, double pose_yaw);
    void Pub();
    
    // Convert to map frame 
    double d_map_height_;
    const double D_GEOD_A = 6378137.0;//SemiMajorAxis
    const double D_GEOD_E2 = 0.0; // FirstEccentricitySquard, e ^ 2
    // const double D_GEOD_E2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2
    const double D_RAD_2_DEG = 180 / M_PI;
    const double D_DEG_2_RAD = M_PI / 180;

private:
    void GNSS_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void IMU_Callback(const sensor_msgs::Imu::ConstPtr &msg);
    void ODOM_Callback(const nav_msgs::Odometry::ConstPtr &msg);
    void SPEED_Callback(const std_msgs::Float32::ConstPtr &msg);

    // 1
    geometry_msgs::PoseStamped ConvertToMapFrame(float f_lat, float f_lon, float f_hgt);
    double FnKappaLat(double d_ref_latitude, double d_height);
    double FnKappaLon(double d_ref_latitude, double d_height);

    // 2
    Eigen::Matrix<double, 3, 1> geodetic_to_ecef(double lat, double lon, double h);
    Eigen::Matrix<double, 3, 1> ecef_to_enu(double x, double y, double z, double lat0, double lon0, double h0);
    geometry_msgs::PoseStamped geodetic_to_enu(double lat, double lon, double h, double lat_ref, double lon_ref, double h_ref);

    // 3
    Eigen::Matrix<double, 3, 1> geodetic2ecef(double lat, double lon, double h);
    Eigen::Matrix<double, 3, 1> uvw2enu(double u, double v, double w, double lat_ref, double lon_ref);
    geometry_msgs::PoseStamped geodetic2enu(double lat, double lon, double h, double lat_ref, double lon_ref, double h_ref);

    // ENU Conversion Data
    // 2
    double a;
    double b;
    double f_;
    double e_sq;
    Eigen::Matrix<double, 3, 1> geodetic_to_ecef_result;
    Eigen::Matrix<double, 3, 1> ecef_to_enu_result;
    Eigen::Matrix<double, 3, 1> geodetic_to_enu_progress;
    Eigen::Matrix<double, 3, 1> geodetic_to_enu_result;
    // 3
    Eigen::Matrix<double, 3, 1> xyz1;
    Eigen::Matrix<double, 3, 1> xyz2;
    Eigen::Matrix<double, 3, 1> geodetic2ecef_result;
    Eigen::Matrix<double, 3, 1> uvw2enu_result;
    Eigen::Matrix<double, 3, 1> geodetic2enu_result;

    // Error Data
    error_msg_file::Error error_msg;
    double error;
    double time_flow;

    // ROS Data
    ros::NodeHandle nh;
    ros::Publisher car_pose_pub;
    ros::Publisher car_pose_pub2;
    ros::Publisher error_pub;
    ros::Subscriber gnss_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber speed_sub;
    nav_msgs::Odometry car_pose_msg;
    nav_msgs::Odometry car_pose_msg2;

    // SPEED Data
    double speed;
    double s;
    double yr;

    
    // GNSS Data
    bool bias_init;
    bool gnss_init;
    bool call_gnss;
    double lon_raw;     // [deg]
    double lat_raw;     // [deg]
    double hei_raw;
    double lon_rad;     // [rad]
    double lat_rad;     // [rad]
    double local_x;
    double local_y;
    ros::Time prev_gnss_time;
    ros::Time gnss_time;
    ros::Duration gnss_duration;
    double gnss_dt;
    double offset_x;
    double offset_y;
    double gnss_yaw_rad;

    // IMU Data
    bool imu_init;
    double angular_velocity_x;    // [rad/s]
    double angular_velocity_y;    // [rad/s]
    double angular_velocity_z;    // [rad/s]
    double linear_acceleration_x;   // [m/s^2]
    double linear_acceleration_y;   // [m/s^2]
    double linear_acceleration_z;   // [m/s^2]
    double imu_quaternion_x;
    double imu_quaternion_y;
    double imu_quaternion_z;
    double imu_quaternion_w;
    double imu_roll_rad;
    double imu_pitch_rad;
    double imu_yaw_rad;
    double imu_roll_deg;
    double imu_pitch_deg;
    double imu_yaw_deg;
    double angular_velocity_z_bias;
    ros::Time prev_imu_time;
    ros::Time imu_time;
    ros::Duration imu_duration;
    double imu_dt;

    // ODOM Data
    bool odom_init;
    bool odom_local_init;
    double odom_quaternion_x;
    double odom_quaternion_y;
    double odom_quaternion_z;
    double odom_quaternion_w;
    double odom_roll_rad;
    double odom_pitch_rad;
    double odom_yaw_rad;
    double odom_roll_deg;
    double odom_pitch_deg;
    double odom_yaw_deg;
    double odom_local_x;
    double odom_local_y;
    ros::Time odom_time;

    // Dead Reckoning Data
    Eigen::Matrix<double, 3, 1> dr;

    // EKF Data
    bool isInit;
    double linear_vel_x;
    double linear_vel_y;
    double linear_vel_z;
    double velocity;
    Eigen::Matrix<double, 3, 1> f;
    Eigen::Matrix<double, 3, 1> f2;
    Eigen::Matrix3d F;
    Eigen::Matrix3d F2;
    Eigen::Matrix<double, 3, 1> x_prior;
    Eigen::Matrix3d P_prior;
    Eigen::Matrix<double, 3, 1> x_prior2;
    Eigen::Matrix3d P_prior2;
    Eigen::Matrix3d Q;
    Eigen::Matrix3d Q2;
    Eigen::Matrix3d R;
    // Eigen::Matrix2d R;
    Eigen::Matrix<double, 3, 1> z;
    // Eigen::Matrix<double, 2, 1> z;
    Eigen::Matrix<double, 3, 1> h;
    Eigen::Matrix<double, 3, 1> h2;
    // Eigen::Matrix<double, 2, 1> h;
    Eigen::Matrix<double, 3, 1> y;
    Eigen::Matrix<double, 3, 1> y2;
    Eigen::Matrix3d H;
    // Eigen::Matrix<double, 2, 3> H;
    Eigen::Matrix3d S;
    Eigen::Matrix3d S2;
    // Eigen::Matrix2d S;
    Eigen::Matrix3d K;
    Eigen::Matrix3d K2;
    // Eigen::Matrix<double, 3, 2> K;
    Eigen::Matrix<double, 3, 1> x_posterior;
    Eigen::Matrix3d P_posterior;
    Eigen::Matrix<double, 3, 1> x_posterior2;
    Eigen::Matrix3d P_posterior2;
    Eigen::Matrix<double, 3, 1> estimated_pose;
    Eigen::Matrix<double, 3, 1> estimated_pose2;
    Eigen::Matrix3d eye;
    double world_x;
    double world_y;
};