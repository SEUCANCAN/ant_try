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
    
    int cnt = 3;
    while(cnt--)
	{
		nav_msgs::Odometry msg;
		nav_msgs::Odometry end_position;
		end_position.pose.pose.position.x =   671331.680;
		end_position.pose.pose.position.y = 	3529414.460	;
		end_position.pose.covariance[0] = 5.250-3.1415;
		
		
		msg.pose.pose.position.x = 671080.970;		 
		msg.pose.pose.position.y = 3529490.760;
		msg.pose.covariance[0] = 0.720;
		target_pub.publish(msg); //end_position
		vehicle_pose_pub.publish(end_position); //msg
		ROS_INFO("send ok");
		loop_rate.sleep();
	}
    return 0;
}
