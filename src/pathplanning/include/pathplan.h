
#include <vector>
#include "utils.hpp"
#ifndef _PATHPLAN_H_
#define _PATHPLAN_H_

struct vehicleinfo
{
     float x;
     float y;
     float yaw;
};

struct Road_info
{
     std::vector<gpsMsg_t> centerline;
     int ID;
     float leftoffset;
     float middleoffset;
     float rightoffset;
     float cost_positive;
     float cost_negative;    
};

struct modle_point
{
    int x;
    int y;
};

struct pathwith_modlepoint
{   
    std::vector<modle_point>mod_point;
    float cost;   
};
struct roadoffset
{
    int road_id;
    float road_offset;
};
#endif
