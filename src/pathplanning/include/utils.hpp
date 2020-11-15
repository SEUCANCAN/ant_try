#ifndef __UTILS_H_
#define __UTILS_H_
#include <iostream>
#include <fstream>

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	double x;
	double y;
	float curvature;
}gpsMsg_t;


static float loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points)
{
	std::ifstream in_file(file_path.c_str());
	if(!in_file.is_open())
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return 0.0;
	}
	gpsMsg_t point;
	std::string line;
	
	while(in_file.good())
	{
		getline(in_file,line);
		std::stringstream ss(line);
		ss >> point.x >> point.y >> point.yaw >> point.curvature;
		points.push_back(point);
	}
	
	in_file.close();
	float reslution = 0.1;
	return reslution;
}




#endif
