#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

using namespace ros;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"generate_vehicle_pose");
    ros::NodeHandle n;
    Publisher vehicle_pose_pub = n.advertise<nav_msgs::Odometry>("/ll2utm_odom", 1000 );
    Publisher target_pub = n.advertise<nav_msgs::Odometry>("end_position", 1000 );
    Rate loop_rate(1);
    while(ok())
{
    nav_msgs::Odometry msg;
    nav_msgs::Odometry end_position;
    end_position.pose.pose.position.x = 100;
    end_position.pose.pose.position.y = -50;
    end_position.pose.covariance[0] = 0.2;
    target_pub.publish(end_position);
    msg.pose.pose.position.x = 300;
    msg.pose.pose.position.y = -250;
    msg.pose.covariance[0] = -1.5;
    vehicle_pose_pub.publish(msg);
    ROS_INFO("send ok");
    loop_rate.sleep();
}
    return 0;
}
