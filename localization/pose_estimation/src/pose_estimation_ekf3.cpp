#include "pose_estimation/pose_estimation_ekf.h"

// Define Function
EKF::EKF()
{
    gnss_sub = nh.subscribe("gnss_data", 100, &EKF::GNSS_Callback, this);
    imu_sub = nh.subscribe("/carla/ego_vehicle/imu", 100, &EKF::IMU_Callback, this);
    odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &EKF::ODOM_Callback, this);
    car_pose_pub = nh.advertise<nav_msgs::Odometry>("car_odometry", 100);

    // GNSS
    gnss_init = false;
    call_gnss = false;
    calculate_offset = false;
    lon_raw = 0.0;
    lat_raw = 0.0;
    hei_raw = 0.0;
    lon_rad = 0.0; 
    lat_rad = 0.0;
    local_x = 0.0;
    local_y = 0.0;
    prev_gnss_time = ros::Time::now();
    gnss_time = ros::Time::now();
    gnss_dt = 0.0;
    offset_y = 0.0;

    // IMU
    bias_init = false;
    imu_init = false;
    angular_velocity_x = 0.0;
    angular_velocity_y = 0.0;
    angular_velocity_z = 0.0;
    linear_acceleration_x = 0.0;
    linear_acceleration_y = 0.0;
    linear_acceleration_z = 0.0;
    prev_imu_time = ros::Time::now();
    imu_time = ros::Time::now();
    imu_dt = 0.0;

    // ODOM
    odom_init = false;
    odom_local_init = false;
    quaternion_x = 0.0;
    quaternion_y = 0.0;
    quaternion_z = 0.0;
    quaternion_w = 0.0;
    roll_rad = 0.0;
    pitch_rad = 0.0;
    yaw_rad = 0.0;
    roll_deg = 0.0;
    pitch_deg = 0.0;
    yaw_deg = 0.0;
    odom_local_y = 0.0;
    
    //EKF
    isInit = false;
    linear_vel_x = 0.0;
    linear_vel_y = 0.0;
    velocity = 0.0;
    Q <<    0.0001d, 0.0d, 0.0d,      // [x_std_m^2 0 0;
            0.0d, 0.0001d, 0.0d,      //  0 y_std_m^2 0;
            0.0d, 0.0d, 0.01d;      //  0 0 yaw_std_deg^2]
    R <<    0.01d, 0.0d,             //  [x_measure_std_m^2 0;
            0.0d, 0.01d;             //   0 y_measure_std_m^2]
    eye <<  1.0d, 0.0d, 0.0d,
            0.0d, 1.0d, 0.0d,
            0.0d, 0.0d, 1.0d;
    world_x = 0.0;
    world_y = 0.0;

    // ENU Conversion Data
    a = 6378137.0;
    b = a;
    f_ = (a - b) / a;
    e_sq = f_ * (2 - f_);
}

void EKF::GNSS_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gnss_init = true;
    call_gnss = true;
    lon_raw = msg->longitude;
    lat_raw = msg->latitude;
    hei_raw = msg->altitude;
    // lon_rad = lon_raw * D_DEG_2_RAD;
    // lat_rad = lat_raw * D_DEG_2_RAD;
    geometry_msgs::PoseStamped local_pose = geodetic2enu(lat_raw, lon_raw, hei_raw, 0.0, 0.0, 0.0);
    // geometry_msgs::PoseStamped local_pose = geodetic2enu(lat_raw, lon_raw, hei_raw, 0.0, 0.00000898315002, 2.00000008);
    // if (calculate_offset == false)
    // {
    //     offset_y = local_pose.pose.position.y - odom_local_y;
    //     calculate_offset = true;
    // }
    // std::cout <<"offset_y : "<< offset_y<<std::endl;
    local_x = local_pose.pose.position.x;
    local_y = local_pose.pose.position.y;
    // local_y = local_pose.pose.position.y - 1.333185;
    // local_x = a * lon_rad * cos(lat_rad);
    // local_y = a * lat_rad;
    prev_gnss_time = gnss_time;
    gnss_time = msg->header.stamp;
    gnss_duration = gnss_time - prev_gnss_time;
    gnss_dt = gnss_duration.toSec();
}

void EKF::IMU_Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // if(bias_init == false)
    // {
    //     angular_velocity_z_bias = msg->angular_velocity.z;
    //     bias_init = true;
    // }
    imu_init = true;
    angular_velocity_x = msg->angular_velocity.x;
    angular_velocity_y = msg->angular_velocity.y;
    angular_velocity_z = msg->angular_velocity.z;
    // angular_velocity_z = msg->angular_velocity.z - angular_velocity_z_bias;
    linear_acceleration_x = msg->linear_acceleration.x;
    linear_acceleration_y = msg->linear_acceleration.y;
    linear_acceleration_z = msg->linear_acceleration.z;
    prev_imu_time = imu_time;
    imu_time = msg->header.stamp;
    imu_duration = imu_time - prev_imu_time;
    imu_dt = imu_duration.toSec();
}

void EKF::ODOM_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // if (odom_local_init == false)
    // {
    //     odom_local_y = msg->pose.pose.position.y;
    //     odom_local_init = true;
    // }

    odom_init = true;
    quaternion_x = msg->pose.pose.orientation.x;
    quaternion_y = msg->pose.pose.orientation.y;
    quaternion_z = msg->pose.pose.orientation.z;
    quaternion_w = msg->pose.pose.orientation.w;

    roll_rad = atan2(2.0 * (quaternion_y * quaternion_z + quaternion_w * quaternion_x), quaternion_w * quaternion_w - quaternion_x * quaternion_x - quaternion_y * quaternion_y + quaternion_z * quaternion_z);
    pitch_rad = asin(-2.0 * (quaternion_x * quaternion_z - quaternion_w * quaternion_y));
    yaw_rad = atan2(2.0 * (quaternion_x * quaternion_y + quaternion_w * quaternion_z), quaternion_w * quaternion_w + quaternion_x * quaternion_x - quaternion_y * quaternion_y - quaternion_z * quaternion_z);
    roll_deg = roll_rad * D_RAD_2_DEG;
    pitch_deg = pitch_rad * D_RAD_2_DEG;
    yaw_deg = yaw_rad * D_RAD_2_DEG;
}

Eigen::Matrix<double, 3, 1> EKF::Dead_Reckoning(Eigen::Matrix<double, 3, 1> x_pri, double vel, double angular_vel_z, double dt)
{
    dr <<   x_pri(0, 0) + vel * dt * cos(x_pri(2, 0)),
            x_pri(1, 0) + vel * dt * sin(x_pri(2, 0)),
            x_pri(2, 0) + angular_vel_z * dt;
    return dr;
}

void EKF::Pose_Estimation_EKF()
{    
    std::cout << "ekf running"<< std::endl;

    if (Only_GNSS == true)
    {
        ros::Rate loop_rate_only_gnss(4);
        while(ros::ok())
        {
            estimated_pose(0,0) = local_x;
            estimated_pose(1,0) = local_y;
            estimated_pose(2,0) = yaw_rad;
            SetPose(estimated_pose(0, 0), estimated_pose(1, 0), estimated_pose(2, 0));
            PosePub();
            loop_rate_only_gnss.sleep();
            ros::spinOnce();
        }
    }
    else
    {
        ros::Rate loop_rate(100);
        while(ros::ok())
        {
            if(gnss_init == true && isInit == false && odom_init == true)
            {
                x_prior <<  local_x,
                            local_y,
                            yaw_rad;
                P_prior <<  1000.0d, 0.0d, 0.0d,
                            0.0d, 1000.0d, 0.0d,
                            0.0d, 0.0d, 1000.0d;
                estimated_pose = x_prior;

                isInit = true;
            }
            else
            {
                // System function
                linear_vel_x += linear_acceleration_x * imu_dt;
                linear_vel_y += linear_acceleration_y * imu_dt;
                velocity = sqrt(pow(linear_vel_x, 2) + pow(linear_vel_y, 2));

                f = Dead_Reckoning(x_prior, velocity, angular_velocity_z, imu_dt);

                // System jacobian
                F <<    1.0d, 0.0d, ((-1.0) * (velocity * imu_dt * sin(x_prior(2, 0)))),
                        0.0d, 1.0d, (velocity * imu_dt * cos(x_prior(2, 0))),
                        0.0d, 0.0d, 1.0d;

                x_prior = f;
                P_prior = F * P_prior * F.transpose() + Q;

                // Gnss Measurement
                z <<    local_x,
                        local_y;

                if (call_gnss == true && Only_Prediction == false)  // Measurement Update 
                {
                    // Measurement Update
                    h <<    x_prior(0, 0),
                            x_prior(1, 0);

                    // Residual
                    y = z - h;

                    // Measurement jacobian
                    H <<    1.0d, 0.0d, 0.0d,
                            0.0d, 1.0d, 0.0d;

                    // Innovation covariance
                    S = H * P_prior * H.transpose() + R;

                    // Kalman gain
                    K = P_prior * H.transpose() * S.inverse();

                    //Update
                    x_posterior = x_prior + K * y;
                    P_posterior = (eye - K * H) * P_prior;
                    call_gnss = false;
                }
                else
                {
                    x_posterior = x_prior;
                    P_posterior = P_prior;
                }
                
                // Estimate Pose
                estimated_pose = x_posterior;
                x_prior = x_posterior;
                P_prior = P_posterior;
            }
            SetPose(estimated_pose(0, 0), estimated_pose(1, 0), estimated_pose(2, 0));
            PosePub();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}

void EKF::SetPose(double pose_x, double pose_y, double pose_yaw)
{
    // world_y = pose_y / E;
    // world_x = pose_x / (E * cos(world_y));
    // pose_msg.lon = (world_x * 180.0) / M_PI;
    // pose_msg.lat = (world_y * 180.0) / M_PI;

    car_pose_msg.pose.pose.position.x = pose_x;
    car_pose_msg.pose.pose.position.y = pose_y;
    car_pose_msg.twist.twist.linear.z = yaw_deg;
    car_pose_msg.twist.twist.angular.z = pose_yaw * D_RAD_2_DEG;
}

void EKF::PosePub()
{
    car_pose_pub.publish(car_pose_msg);
}

geometry_msgs::PoseStamped EKF::geodetic2enu(double lat, double lon, double h, double lat_ref, double lon_ref, double h_ref)
{
    xyz1 = geodetic2ecef(lat, lon, h);
    xyz2 = geodetic2ecef(lat_ref, lon_ref, h_ref);

    geodetic2enu_result = uvw2enu((xyz1(0, 0) - xyz2(0, 0)), (xyz1(1, 0) - xyz2(1, 0)), (xyz1(2, 0) - xyz2(2, 0)), lat_ref, lon_ref);

    geometry_msgs::PoseStamped psstp_pose;

    psstp_pose.header.stamp = ros::Time::now();
    psstp_pose.header.frame_id = "map";

    psstp_pose.pose.position.x = geodetic2enu_result(0,0);
    psstp_pose.pose.position.y = geodetic2enu_result(1,0);
    psstp_pose.pose.position.z = geodetic2enu_result(2,0);

    return(psstp_pose);

}

Eigen::Matrix<double, 3, 1> EKF::geodetic2ecef(double lat, double lon, double h)
{
    double lat_rad = lat * D_DEG_2_RAD;
    double lon_rad = lon * D_DEG_2_RAD;
    double N = (pow(a, 2)) / sqrt(pow(a, 2) * pow(cos(lat_rad), 2) + pow(b, 2) * pow(sin(lat_rad), 2));
    geodetic2ecef_result <<     (N + h) * cos(lat_rad) * cos(lon_rad),
                                (N + h) * cos(lat_rad) * sin(lon_rad),
                                (N * pow((b / a), 2) + h) * sin(lat_rad);
    return geodetic2ecef_result;
}

Eigen::Matrix<double, 3, 1> EKF::uvw2enu(double u, double v, double w, double lat_ref, double lon_ref)
{
    double lat_ref_rad = lat_ref * D_DEG_2_RAD;
    double lon_ref_rad = lon_ref * D_DEG_2_RAD;
    double t = cos(lon_ref_rad) * u + sin(lon_ref_rad) * v;
    uvw2enu_result <<       (-1) * sin(lon_ref_rad) * u + cos(lon_ref_rad) * v,
                            (-1) * sin(lat_ref_rad) * t + cos(lat_ref_rad) * w,
                            cos(lat_ref_rad) * t + sin(lat_ref_rad) * w;
    return uvw2enu_result;
}
