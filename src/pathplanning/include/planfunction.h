#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <vector>
#include "pathplan.h"
#include "utils.hpp"
#ifndef _PLANFUNCTION_H_
#define _PLANFUNCTION_H_



void Road_init(Road_info Road[]);

std::vector<gpsMsg_t> find_track_path(vehicleinfo position, vehicleinfo end_point, Road_info Road[]);

int findnearRoad(vehicleinfo position, Road_info Road[],int road_point[]);

void ID_to_point(int id,int road_point[]);

std::vector<pathwith_modlepoint> modle_point_path_generate(std::vector<pathwith_modlepoint> path_try,int target_point[], int targetroad_id, Road_info Road[]);

void point_to_Road(modle_point point_first, modle_point point_second, Road_info Road[], float road_cost[]);

void pointpath_to_track_path(std::vector<gpsMsg_t>& track_path, pathwith_modlepoint mincost_path, Road_info Road[]);

#endif


