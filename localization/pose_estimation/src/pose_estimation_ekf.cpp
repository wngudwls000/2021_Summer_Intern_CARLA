#include "pose_estimation/pose_estimation_ekf.h"

// Define Function
EKF::EKF()
{
    gnss_sub = nh.subscribe("gnss_data", 100, &EKF::GNSS_Callback, this);
    imu_sub = nh.subscribe("/carla/ego_vehicle/imu", 100, &EKF::IMU_Callback, this);
    odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 100, &EKF::ODOM_Callback, this);
    speed_sub = nh.subscribe("/carla/ego_vehicle/speedometer", 100, &EKF::SPEED_Callback, this);
    car_pose_pub = nh.advertise<nav_msgs::Odometry>("car_odometry", 100);
    car_pose_pub2 = nh.advertise<nav_msgs::Odometry>("car_odometry2", 100);
    error_pub = nh.advertise<error_msg_file::Error>("error", 100);

    // Error
    error = 0.0;
    time_flow = 0.0;

    // SPEED
    speed = 0.0;

    // GNSS
    bias_init = false;
    gnss_init = false;
    call_gnss = false;
    lon_raw = 0.0;
    lat_raw = 0.0;
    lon_rad = 0.0; 
    lat_rad = 0.0;
    local_x = 0.0;
    local_y = 0.0;
    prev_gnss_time = ros::Time::now();
    gnss_time = ros::Time::now();
    gnss_dt = 0.0;
    gnss_yaw_rad = 0.0;
    offset_x = 0.000004492;
    offset_y = 0.000004492;

    // IMU
    imu_init = false;
    angular_velocity_x = 0.0;
    angular_velocity_y = 0.0;
    angular_velocity_z = 0.0;
    linear_acceleration_x = 0.0;
    linear_acceleration_y = 0.0;
    linear_acceleration_z = 0.0;
    imu_quaternion_x;
    imu_quaternion_y;
    imu_quaternion_z;
    imu_quaternion_w;
    imu_roll_rad;
    imu_pitch_rad;
    imu_yaw_rad;
    imu_roll_deg;
    imu_pitch_deg;
    imu_yaw_deg;
    angular_velocity_z_bias = 0.0;
    prev_imu_time = ros::Time::now();
    imu_time = ros::Time::now();
    imu_dt = 0.0;

    // ODOM
    odom_init = false;
    odom_local_init = false;
    odom_quaternion_x = 0.0;
    odom_quaternion_y = 0.0;
    odom_quaternion_z = 0.0;
    odom_quaternion_w = 0.0;
    odom_roll_rad = 0.0;
    odom_pitch_rad = 0.0;
    odom_yaw_rad = 0.0;
    odom_roll_deg = 0.0;
    odom_pitch_deg = 0.0;
    odom_yaw_deg = 0.0;
    odom_local_x = 0.0;
    odom_local_y = 0.0;

    //EKF
    isInit = false;
    linear_vel_x = 0.0;
    linear_vel_y = 0.0;
    velocity = 0.0;
    Q <<    0.0001d, 0.0d, 0.0d,           // [x_std_m^2 0 0;
            0.0d, 0.0001d, 0.0d,           //  0 y_std_m^2 0;
            0.0d, 0.0d, 0.01d;           //  0 0 yaw_std_deg^2]
    Q2 <<   0.0d, 0.0d, 0.0d,           // [x_std_m^2 0 0;
            0.0d, 0.0d, 0.0d,           //  0 y_std_m^2 0;
            0.0d, 0.0d, 0.0d;           //  0 0 yaw_std_deg^2]
    R <<    0.25d, 0.0d, 0.0d,                //  [x_measure_std_m^2 0;
            0.0d, 0.25d, 0.0d,
            0.0d, 0.0d, 0.0d;                //   0 y_measure_std_m^2]
    // R <<    0.64d, 0.0d, 0.0d,                //  [x_measure_std_m^2 0;
    //         0.0d, 0.64d, 0.0d,
    //         0.0d, 0.0d, 0.001d;                //   0 y_measure_std_m^2]
    eye <<  1.0d, 0.0d, 0.0d,
            0.0d, 1.0d, 0.0d,
            0.0d, 0.0d, 1.0d;
    world_x = 0.0;
    world_y = 0.0;
    
    a = 6378137.0;

}

void EKF::GNSS_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gnss_init = true;
    call_gnss = true;
    // lon_raw = msg->longitude;
    // lat_raw = msg->latitude;
    lon_raw = msg->longitude - offset_x;
    lat_raw = msg->latitude - offset_y;
    gnss_yaw_rad = imu_yaw_rad;
    // lon_rad = lon_raw * D_DEG_2_RAD;
    // lat_rad = lat_raw * D_DEG_2_RAD;

    geometry_msgs::PoseStamped local_pose = ConvertToMapFrame((float)lat_raw, (float)lon_raw, 0.0);
    
    // if (calculate_offset == false)
    // {
    //     offset_y = local_pose.pose.position.y - odom_local_y;
    //     calculate_offset = true;
    // }
    // std::cout <<"offset_y : "<< offset_y<<std::endl;

    // if(bias_init == false)
    // {
    //     offset_x = local_x - ;
    //     offset_y = local_y - ; 
    //     bias_init = true;
    // }

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
    if (fabs(linear_acceleration_x) < 0.05)
    {
        linear_acceleration_x = 0.0;
    }
    if (fabs(linear_acceleration_y) < 0.05)
    {
        linear_acceleration_y = 0.0;
    }
    if (fabs(angular_velocity_z) < 0.005)
    {
        angular_velocity_z = 0.0;
    }
    // if (angular_velocity_z > M_PI)
    // {
    //     angular_velocity_z -= 2 * M_PI;
    // }
    // if (angular_velocity_z < -M_PI)
    // {
    //     angular_velocity_z += 2 * M_PI;
    // }
    imu_quaternion_x = msg->orientation.x;
    imu_quaternion_y = msg->orientation.y;
    imu_quaternion_z = msg->orientation.z;
    imu_quaternion_w = msg->orientation.w;
    tf::Quaternion imu_q(msg->orientation.x,
                        msg->orientation.y,
                        msg->orientation.z,
                        msg->orientation.w);
    tf::Matrix3x3 imu_rpy(imu_q);
    
    // imu_roll_rad = atan2(2.0 * (imu_quaternion_y * imu_quaternion_z + imu_quaternion_w * imu_quaternion_x), imu_quaternion_w * imu_quaternion_w - imu_quaternion_x * imu_quaternion_x - imu_quaternion_y * imu_quaternion_y + imu_quaternion_z * imu_quaternion_z);
    // imu_pitch_rad = asin(-2.0 * (imu_quaternion_x * imu_quaternion_z - imu_quaternion_w * imu_quaternion_y));
    // imu_yaw_rad = atan2(2.0 * (imu_quaternion_x * imu_quaternion_y + imu_quaternion_w * imu_quaternion_z), imu_quaternion_w * imu_quaternion_w + imu_quaternion_x * imu_quaternion_x - imu_quaternion_y * imu_quaternion_y - imu_quaternion_z * imu_quaternion_z);
    imu_rpy.getRPY(imu_roll_rad, imu_pitch_rad, imu_yaw_rad);
    
    if (imu_yaw_rad > M_PI)
    {
        imu_yaw_rad -= 2 * M_PI;
    }
    if (imu_yaw_rad < -M_PI)
    {
        imu_yaw_rad += 2 * M_PI;
    }
    imu_roll_deg = imu_roll_rad * D_RAD_2_DEG;
    imu_pitch_deg = imu_pitch_rad * D_RAD_2_DEG;
    imu_yaw_deg = imu_yaw_rad * D_RAD_2_DEG;

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
    odom_local_x = msg->pose.pose.position.x;
    odom_local_y = msg->pose.pose.position.y;

    odom_init = true;
    odom_quaternion_x = msg->pose.pose.orientation.x;
    odom_quaternion_y = msg->pose.pose.orientation.y;
    odom_quaternion_z = msg->pose.pose.orientation.z;
    odom_quaternion_w = msg->pose.pose.orientation.w;

    odom_roll_rad = atan2(2.0 * (odom_quaternion_y * odom_quaternion_z + odom_quaternion_w * odom_quaternion_x), odom_quaternion_w * odom_quaternion_w - odom_quaternion_x * odom_quaternion_x - odom_quaternion_y * odom_quaternion_y + odom_quaternion_z * odom_quaternion_z);
    odom_pitch_rad = asin(-2.0 * (odom_quaternion_x * odom_quaternion_z - odom_quaternion_w * odom_quaternion_y));
    odom_yaw_rad = atan2(2.0 * (odom_quaternion_x * odom_quaternion_y + odom_quaternion_w * odom_quaternion_z), odom_quaternion_w * odom_quaternion_w + odom_quaternion_x * odom_quaternion_x - odom_quaternion_y * odom_quaternion_y - odom_quaternion_z * odom_quaternion_z);
    if (odom_yaw_rad > M_PI)
    {
        odom_yaw_rad -= 2 * M_PI;
    }
    if (odom_yaw_rad < -M_PI)
    {
        odom_yaw_rad += 2 * M_PI;
    }
    odom_roll_deg = odom_roll_rad * D_RAD_2_DEG;
    odom_pitch_deg = odom_pitch_rad * D_RAD_2_DEG;
    odom_yaw_deg = odom_yaw_rad * D_RAD_2_DEG;

    odom_time = msg->header.stamp;
}

void EKF::SPEED_Callback(const std_msgs::Float32::ConstPtr &msg)
{
    speed = msg->data;
}

Eigen::Matrix<double, 3, 1> EKF::Dead_Reckoning(Eigen::Matrix<double, 3, 1> x_pri, double vel, double angular_vel_z, double dt)
{
    dr <<   x_pri(0, 0) + vel * dt * cos(x_pri(2, 0)),
            x_pri(1, 0) + vel * dt * sin(x_pri(2, 0)),
            x_pri(2, 0) + angular_vel_z * dt;
    if (dr(2, 0) > M_PI)
    {
        dr(2, 0)  -= 2 * M_PI;
    }

    if (dr(2, 0)  < -M_PI)
    {
        dr(2, 0)  += 2 * M_PI;
    }

    return dr;
}

void EKF::Pose_Estimation_EKF()
{   
    std::cout << "ekf running"<< std::endl;

    if (Only_GNSS == true)
    {
        ros::Rate loop_rate_only_gnss(4.0);
        while(ros::ok())
        {
            estimated_pose(0,0) = local_x;
            estimated_pose(1,0) = local_y;
            estimated_pose(2,0) = gnss_yaw_rad;
            std::cout<<"gnss_yaw"<<gnss_yaw_rad<<std::endl;
            SetPose(estimated_pose(0, 0), estimated_pose(1, 0), estimated_pose(2, 0));
            Calculate_Error(estimated_pose(0, 0), odom_local_x, estimated_pose(1, 0), odom_local_y);
            std::cout<<error<<std::endl;
            Pub();
            loop_rate_only_gnss.sleep();
            ros::spinOnce();
        }
    }
    else
    {
        ros::Rate loop_rate(100.0);
        while(ros::ok())
        {
            if(gnss_init == true && isInit == false && odom_init == true)
            {
                x_prior <<  local_x,
                            local_y,
                            odom_yaw_rad;
                P_prior <<  1000.0d, 0.0d, 0.0d,
                            0.0d, 1000.0d, 0.0d,
                            0.0d, 0.0d, 1000.0d;
                x_prior2 <<     local_x,
                                local_y,
                                odom_yaw_rad;
                P_prior2 <<     1000.0d, 0.0d, 0.0d,
                                0.0d, 1000.0d, 0.0d,
                                0.0d, 0.0d, 1000.0d;
                estimated_pose = x_prior;
                estimated_pose2 = x_prior2;

                isInit = true;
            }
            else
            {
                // System function
                linear_vel_x += linear_acceleration_x * imu_dt;
                linear_vel_y += linear_acceleration_y * imu_dt;
                linear_vel_z += linear_acceleration_z * imu_dt;
                // velocity = linear_vel_x * cos(imu_pitch_rad) * cos(imu_yaw_rad) + linear_vel_y * cos(imu_pitch_rad) * sin(imu_yaw_rad) + linear_vel_z * sin(imu_pitch_rad);
                // velocity = sqrt(pow(linear_vel_x, 2) + pow(linear_vel_y, 2));
                // velocity = linear_vel_x;
                velocity = speed;
                f = Dead_Reckoning(x_prior, velocity, angular_velocity_z, imu_dt);
                // f = Dead_Reckoning(x_prior2, speed, angular_velocity_z, imu_dt);
                f2 = Dead_Reckoning(x_prior2, s, yr, imu_dt);
                
                s = speed;
                yr = angular_velocity_z;

                // System jacobian
                F <<    1.0d, 0.0d, ((-1.0) * (velocity * imu_dt * sin(x_prior(2, 0)))),
                        0.0d, 1.0d, (velocity * imu_dt * cos(x_prior(2, 0))),
                        0.0d, 0.0d, 1.0d;
                F2 <<   1.0d, 0.0d, ((-1.0) * (speed * imu_dt * sin(x_prior2(2, 0)))),
                        0.0d, 1.0d, (speed * imu_dt * cos(x_prior2(2, 0))),
                        0.0d, 0.0d, 1.0d;

                x_prior = f;
                P_prior = F * P_prior * F.transpose() + Q;
                x_prior2 = f2;
                P_prior2 = F2 * P_prior2 * F2.transpose() + Q2;

                // Gnss Measurement
                z <<    local_x,
                        local_y,
                        gnss_yaw_rad;

                if (call_gnss == true && Only_Prediction == false)  // Measurement Update 
                {
                    // Measurement Update
                    h <<    x_prior(0, 0),
                            x_prior(1, 0),
                            x_prior(2, 0);
                    h2 <<   x_prior2(0, 0),
                            x_prior2(1, 0),
                            x_prior2(2, 0);

                    // Residual
                    y = z - h;
                    y2 = z - h2;
                    // std::cout<<"y : "<<y<<std::endl;
                    // std::cout<<"z : "<<z<<std::endl;
                    // std::cout<<"h : "<<h<<std::endl;
                    // Measurement jacobian
                    H <<    1.0d, 0.0d, 0.0d,
                            0.0d, 1.0d, 0.0d,
                            0.0d, 0.0d, 1.0d;

                    // Innovation covariance
                    S = H * P_prior * H.transpose() + R;
                    S2 = H * P_prior2 * H.transpose() + R;

                    // Kalman gain
                    K = P_prior * H.transpose() * S.inverse();
                    K2 = P_prior2 * H.transpose() * S2.inverse();

                    //Update
                    x_posterior = x_prior + K * y;
                    P_posterior = (eye - K * H) * P_prior;
                    x_posterior2 = x_prior2 + K2 * y2;
                    P_posterior2 = (eye - K2 * H) * P_prior2;
                    call_gnss = false;
                }
                else
                {
                    x_posterior = x_prior;
                    P_posterior = P_prior;
                    x_posterior2 = x_prior2;
                    P_posterior2 = P_prior2;
                }
                
                // Estimate Pose
                estimated_pose = x_posterior;
                estimated_pose2 = x_posterior2;
                x_prior = x_posterior;
                P_prior = P_posterior;
                x_prior2 = x_posterior2;
                P_prior2 = P_posterior2;
            }
            SetPose(estimated_pose(0, 0), estimated_pose(1, 0), estimated_pose(2, 0));
            SetPose2(estimated_pose2(0, 0), estimated_pose2(1, 0), estimated_pose2(2, 0));
            Calculate_Error(estimated_pose(0, 0), odom_local_x, estimated_pose(1, 0), odom_local_y);
            // std::cout<<error<<std::endl;
            Pub();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}

void EKF::Calculate_Error(double estimate_x, double real_x, double estimate_y, double real_y)
{
    error = sqrt(pow((estimate_x - real_x), 2) + pow((estimate_y - real_y), 2));
    error_msg.error = error;
    error_msg.header.stamp = odom_time;
}


void EKF::SetPose(double pose_x, double pose_y, double pose_yaw)
{
    // world_y = pose_y / E;
    // world_x = pose_x / (E * cos(world_y));
    // pose_msg.lon = (world_x * 180.0) / M_PI;
    // pose_msg.lat = (world_y * 180.0) / M_PI;

    car_pose_msg.pose.pose.position.x = pose_x;
    car_pose_msg.pose.pose.position.y = pose_y;
    car_pose_msg.twist.twist.linear.x = local_x + 0.5;
    car_pose_msg.twist.twist.linear.y = local_y + 0.5;
    car_pose_msg.twist.twist.linear.z = odom_yaw_rad;
    car_pose_msg.twist.twist.angular.z = pose_yaw;

    car_pose_msg.pose.pose.orientation.x = error;
    car_pose_msg.header.stamp = odom_time;
}

void EKF::SetPose2(double pose_x, double pose_y, double pose_yaw)
{
    car_pose_msg2.pose.pose.position.x = pose_x;
    car_pose_msg2.pose.pose.position.y = pose_y;
    car_pose_msg2.twist.twist.angular.z = pose_yaw;
}

void EKF::Pub()
{
    car_pose_pub.publish(car_pose_msg);
    car_pose_pub2.publish(car_pose_msg2);
    error_pub.publish(error_msg);
}

geometry_msgs::PoseStamped EKF::ConvertToMapFrame(float f_lat, float f_lon, float f_hgt)
{
    double d_kappa_lat = 0;
    double d_kappa_lon = 0;  

    double d_ref_latitude_deg = 0.0;
    double d_ref_longigude_deg = 0.0;
    // double d_ref_longigude_deg = 0.00000898315002;
    // double m_dRefLatitude_deg = init_gnss.latitude;
    // double d_ref_longigude_deg = init_gnss.longitude;

    f_hgt = 0.;
    // f_hgt = 2.00000008;

    d_kappa_lat = FnKappaLat( d_ref_latitude_deg , f_hgt );
    d_kappa_lon = FnKappaLon( d_ref_latitude_deg , f_hgt );

    geometry_msgs::PoseStamped psstp_pose;

    psstp_pose.header.stamp = ros::Time::now();
    psstp_pose.header.frame_id = "map";

    psstp_pose.pose.position.x = (f_lon - d_ref_longigude_deg)/d_kappa_lon;
    psstp_pose.pose.position.y = (f_lat - d_ref_latitude_deg)/d_kappa_lat;
    psstp_pose.pose.position.z = f_hgt;

    return(psstp_pose);
}

double EKF::FnKappaLat(double d_ref_latitude, double d_height)
{
	double d_kappa_lat = 0;
	double d_denominator = 0;
	double d_m = 0;

	// estimate the meridional radius
	d_denominator = sqrt(1 - D_GEOD_E2 * pow(sin(d_ref_latitude * D_DEG_2_RAD), 2));
	d_m = D_GEOD_A * (1 - D_GEOD_E2) / pow(d_denominator, 3);

	// Curvature for the meridian
	d_kappa_lat = 1 / (d_m + d_height) * D_RAD_2_DEG;

	return d_kappa_lat;
}
double EKF::FnKappaLon(double d_ref_latitude, double d_height)
{
	double d_kappa_lon = 0;
	double d_denominator = 0;
	double d_n = 0;

	// estimate the normal radius
	d_denominator = sqrt(1 - D_GEOD_E2 * pow(sin(d_ref_latitude * D_DEG_2_RAD), 2));
	d_n = D_GEOD_A / d_denominator;

	// Curvature for the meridian
	d_kappa_lon = 1 / ((d_n + d_height) * cos(d_ref_latitude * D_DEG_2_RAD)) * D_RAD_2_DEG;

	return d_kappa_lon;
}