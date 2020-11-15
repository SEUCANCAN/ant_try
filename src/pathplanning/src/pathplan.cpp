#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include "pathplan.h"
#include <vector>
#include "planfunction.h"
#include "geometry_msgs/Pose2D.h"
#include "pathplaning_msgs/expected_path.h"

using namespace ros;
Road_info Road[24];
vehicleinfo end_point;
int receive_ok = 0;
int once = 1;
Publisher track_path_pub;
pathplaning_msgs::expected_path real_path;

//target
void targetpointCallback(const nav_msgs::Odometry::ConstPtr& end_position)
{
     end_point.x = end_position->pose.pose.position.x;
     end_point.y = end_position->pose.pose.position.y;
     end_point.yaw = end_position->pose.covariance[0]; 
     ROS_INFO("target_point ok!");
     receive_ok = 1;
}

void vehicleinfoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
     vehicleinfo position;
     position.x = msg->pose.pose.position.x;
     position.y = msg->pose.pose.position.y;
     position.yaw = msg->pose.covariance[0]; 
     std::vector<gpsMsg_t> track_path;
     if ((receive_ok == 1)&&(once == 1))
     {
		track_path = find_track_path(position,end_point,Road);
		for (int i = 0; i < track_path.size(); i++)
		{
			geometry_msgs::Pose2D temp;
			temp.x = track_path[i].x;
			temp.y = track_path[i].y;
			temp.theta = track_path[i].yaw;
			real_path.points.push_back(temp);
		}
        real_path.direction = real_path.DIRECTION_DRIVE;
        real_path.path_resolution = 0.1 ; 
        real_path.expect_speed = 20 ; 
		ROS_INFO("%d",track_path.size());
		track_path_pub.publish(real_path);
		once = 0;
     }   
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"globel_pathplanning");
    ros::NodeHandle n;
    Road_init(Road);
    ros::Subscriber target = n.subscribe("end_position", 1000, targetpointCallback);
    ros::Subscriber sub = n.subscribe("/ll2utm_odom", 1000, vehicleinfoCallback); //subscribe from gps
    track_path_pub = n.advertise<pathplaning_msgs::expected_path>("/driverless/expect_path", 1000 );
    ros::spin();

    return 0;
}
