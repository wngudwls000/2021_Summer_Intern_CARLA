#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <iostream>

sensor_msgs::NavSatFix gnss_msg;

void GNSS_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gnss_msg.header.stamp = msg->header.stamp;
    gnss_msg.latitude = msg->latitude;
    gnss_msg.longitude = msg->longitude;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_data");
    ros::NodeHandle nh;
    ros::Subscriber gnss_sub = nh.subscribe("/carla/ego_vehicle/gnss", 100, GNSS_Callback);
    ros::Publisher gnss_pub = nh.advertise<sensor_msgs::NavSatFix>("gnss_data", 100);
    std::cout<<"gnss_hz_node"<<std::endl;
    ros::Rate loop_rate(4.0);
    while(ros::ok())
    {
        gnss_pub.publish(gnss_msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
}