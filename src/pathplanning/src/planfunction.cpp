#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <vector>
#include "pathplan.h"
#include "planfunction.h"
#include <math.h>
#include <stdlib.h>

float road_offset[24][3] = {1,3,5,1,3,5,1,3,5,   1,3,3,1,3,3,1,3,3,1,3,3,   1,3,5,1,3,5,1,3,5,   1,3,3,1,3,3,1,3,3,1,3,3,   1,3,5,1,3,5,1,3,5,   1,3,3,1,3,3,1,3,3,1,3,3,   1,3,5,1,3,5, 1,3,5};
float road_cost[24][2] = {1,1,1,1,1,1,     1,1,1,1,1,1,1,1,  1,1,1,1,1,1,  1,1,1,1,1,1,1,1,  1,1,1,1,1,1,  1,1,1,1,1,1,1,1,     1,1,1,1,1,1};


void Road_init(Road_info Road[])
{
     for (int seq = 1 ; seq <= 24 ;seq++)
     {
     std::string ROAD_NAME = "/home/cancan/ant_try/src/pathplanning/src/road_info/ROAD" + std::to_string(seq) + ".txt" ;
     loadPathPoints(ROAD_NAME,Road[seq-1].centerline);
     Road[seq-1].ID = seq;
     Road[seq-1].leftoffset = road_offset[seq-1][0];
     Road[seq-1].middleoffset = road_offset[seq-1][1];
     Road[seq-1].rightoffset = road_offset[seq-1][2];
     Road[seq-1].cost_positive = road_cost[seq-1][0];
     Road[seq-1].cost_negative = road_cost[seq-1][1];
     }
     return;
}

std::vector<gpsMsg_t>find_track_path(vehicleinfo position, vehicleinfo end_point, Road_info Road[])
{
    int start_point[4];
    int target_point[4];
    int startroad_id;
    int targetroad_id;
    startroad_id = findnearRoad(position,Road,start_point) ;
    targetroad_id = findnearRoad(end_point,Road,target_point) ;
    ROS_INFO("near x1 is %d", target_point[0]);
    ROS_INFO("near y1 is %d", target_point[1]);
    ROS_INFO("near x2 is %d", target_point[2]);
    ROS_INFO("near y2 is %d", target_point[3]);
    ROS_INFO("start road id is %d", startroad_id);
    ROS_INFO("target road id is %d", targetroad_id);
    std::vector<pathwith_modlepoint> path_try;
    pathwith_modlepoint point_start;
    modle_point first_point;
    modle_point second_point;
    first_point.x = start_point[0]; first_point.y = start_point[1];
    second_point.x = start_point[2]; second_point.y = start_point[3];
    point_start.mod_point.push_back(first_point);
    point_start.mod_point.push_back(second_point);
    point_start.cost = Road[startroad_id-1].cost_positive;
    path_try.push_back(point_start);
    std::vector<pathwith_modlepoint> path_ok;
    path_ok = modle_point_path_generate(path_try,target_point,targetroad_id, Road);
    pathwith_modlepoint mincost_path;
    float min_cost = 10000;
    int min_index = 1;
    for (int index = 0; index < path_ok.size(); index++)
    {
        //ROS_INFO("cost is %f ",path_ok[index].cost);
        if(path_ok[index].cost < min_cost)
        {
            min_cost = path_ok[index].cost;
            min_index = index;   
        }
    }
    mincost_path = path_ok[min_index];
    for (int i = 0; i < mincost_path.mod_point.size(); i++)
    {
        ROS_INFO("x %d is %d ",i+1,mincost_path.mod_point[i].x);
        ROS_INFO("y %d is %d ",i+1,mincost_path.mod_point[i].y);
    }
    std::vector<gpsMsg_t> track_path;
    pointpath_to_track_path(track_path, mincost_path, Road);
    ROS_INFO("%d",track_path.size());
    return track_path;
}


void pointpath_to_track_path(std::vector<gpsMsg_t>& track_path, pathwith_modlepoint mincost_path, Road_info Road[])
{
    std::vector<roadoffset> roadtrans;
    for(int i = 0; i<(mincost_path.mod_point.size()-2); i++)
    {
        int x1 = mincost_path.mod_point[i].x; int y1 = mincost_path.mod_point[i].y;
        int x2 = mincost_path.mod_point[i+1].x; int y2 = mincost_path.mod_point[i+1].y;
        int x3 = mincost_path.mod_point[i+2].x; int y3 = mincost_path.mod_point[i+2].y; 
        float road_cost[3];
        point_to_Road(mincost_path.mod_point[i], mincost_path.mod_point[i+1], Road, road_cost);
        roadoffset temp;
        temp.road_id = road_cost[1];
        if(((x1 == x2)&&(x3 ==x2)) || ((y1 == y2)&&(y3 == y2)))
        {
            temp.road_offset = road_cost[0] * Road[(int)road_cost[1]-1].middleoffset;
        }
        else if ((((x2-x1) * (y3-y2)) < 0) || (((x3-x2) * (y2-y1)) > 0))
        {
            temp.road_offset = road_cost[0] * Road[(int)road_cost[1]-1].leftoffset;
        }
        else
        {
            temp.road_offset = road_cost[0] * Road[(int)road_cost[1]-1].rightoffset;
        }
        ROS_INFO("%d   %f", temp.road_id, temp.road_offset);
        roadtrans.push_back(temp);
    }
    float last_road_cost[3];
    point_to_Road(mincost_path.mod_point[mincost_path.mod_point.size()-2], mincost_path.mod_point[mincost_path.mod_point.size()-1], Road, last_road_cost);
    roadoffset last_temp;
    last_temp.road_id = last_road_cost[1];
    last_temp.road_offset = last_road_cost[0] * Road[(int)last_road_cost[1]].middleoffset;
    roadtrans.push_back(last_temp); 
    for (int i = 0; i < roadtrans.size(); i++)
    {
        for (int j = 0; j<Road[(int)roadtrans[i].road_id - 1].centerline.size(); j++)
        {
            float x = Road[(int)roadtrans[i].road_id - 1].centerline[j].x;
            float y = Road[(int)roadtrans[i].road_id - 1].centerline[j].y;
            float yaw = Road[(int)roadtrans[i].road_id - 1].centerline[j].yaw;
            float offset = roadtrans[i].road_offset;
            gpsMsg_t temp ;
            temp.x = offset * sin(yaw) + x;
            temp.y = -offset * cos(yaw) + y;
            temp.yaw = (sqrt(pow(offset,2)) / offset) * yaw;
            track_path.push_back(temp); 
            //ROS_INFO("%d",track_path.size());
            ROS_INFO("road id is %d , x is %f , y is %f , yaw is % f ",(int)roadtrans[i].road_id , temp.x , temp.y , temp.yaw );
        }
    }
    return;
}





std::vector<pathwith_modlepoint> modle_point_path_generate(std::vector<pathwith_modlepoint> path_try,int target_point[], int targetroad_id, Road_info Road[] )
{
    std::vector<pathwith_modlepoint> path_ok ;
    int last_path_num = 1;
    for (int num_try = 0; num_try<8 ;num_try++)
    {
       // ROS_INFO("path_try is %d ",path_try.size());
        int current_size = path_try.size();
        for (int i = last_path_num; i <= current_size ;i++)
        {
            if(((path_try[i-1].mod_point[1+num_try].x == target_point[0])&&(path_try[i-1].mod_point[1+num_try].y == target_point[1])) || ((path_try[i-1].mod_point[1+num_try].x == target_point[2])&&(path_try[i-1].mod_point[1+num_try].y == target_point[3])))
            {
                continue;
            }
            else
            {
            //ROS_INFO("num_try is %d ", num_try);
            //ROS_INFO("i is %d ", i);
            //try the left path 
            if (path_try[i-1].mod_point[1+num_try].x-1 > 0)
            {
               modle_point temp_point;
               temp_point.x = path_try[i-1].mod_point[1+num_try].x-1;
               temp_point.y = path_try[i-1].mod_point[1+num_try].y;
               pathwith_modlepoint new_pointpath;
               for (int index = 0; index < path_try[i-1].mod_point.size(); index++)
               {
                   new_pointpath.mod_point.push_back(path_try[i-1].mod_point[index]);
               }
               new_pointpath.mod_point.push_back(temp_point);
               float road_cost[3];
               point_to_Road(path_try[i-1].mod_point[1+num_try],temp_point,Road,road_cost);
               float cost_turn = 0;
               if(path_try[i-1].mod_point[1+num_try].y != path_try[i-1].mod_point[1+num_try-1].y)
               {
                   cost_turn = 0.8;
               }
               if(path_try[i-1].mod_point[1+num_try-1].x == (path_try[i-1].mod_point[1+num_try].x-1))
               {
                   cost_turn = 2;
               }
               new_pointpath.cost = road_cost[2] + path_try[i-1].cost + cost_turn;
               //ROS_INFO("%f , %f , %f",road_cost[0], road_cost[1], road_cost[2]);
               path_try.push_back(new_pointpath);
               if ((temp_point.x == target_point[0]) && (temp_point.y == target_point[1]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[2];
                   end_point.y = target_point[3];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_positive;
                   path_ok.push_back(maybe_path);
               }
               if ((temp_point.x == target_point[2]) && (temp_point.y == target_point[3]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[0];
                   end_point.y = target_point[1];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_negative;
                   path_ok.push_back(maybe_path);
               }
            }
            //try the rifht path 
            if (path_try[i-1].mod_point[1+num_try].x+1 < 5)
            {
               modle_point temp_point;
               temp_point.x = path_try[i-1].mod_point[1+num_try].x+1;
               temp_point.y = path_try[i-1].mod_point[1+num_try].y;
               pathwith_modlepoint new_pointpath;
               for (int index = 0; index < path_try[i-1].mod_point.size(); index++)
               {
                   new_pointpath.mod_point.push_back(path_try[i-1].mod_point[index]);
               }
               new_pointpath.mod_point.push_back(temp_point);
               float road_cost[3];
               point_to_Road(path_try[i-1].mod_point[1+num_try],temp_point,Road,road_cost);
               float cost_turn = 0;
               if(path_try[i-1].mod_point[1+num_try].y != path_try[i-1].mod_point[1+num_try-1].y)
               {
                   cost_turn = 0.8;
               }
               if(path_try[i-1].mod_point[1+num_try-1].x == (path_try[i-1].mod_point[1+num_try].x+1))
               {
                   cost_turn = 2;
               }
               new_pointpath.cost = road_cost[2] + path_try[i-1].cost + cost_turn;
               path_try.push_back(new_pointpath);
               if ((temp_point.x == target_point[0]) && (temp_point.y == target_point[1]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[2];
                   end_point.y = target_point[3];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_positive;
                   path_ok.push_back(maybe_path);
               }
               if ((temp_point.x == target_point[2]) && (temp_point.y == target_point[3]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[0];
                   end_point.y = target_point[1];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_negative;
                   path_ok.push_back(maybe_path);
               }
            }
            //try the up path 
            if (path_try[i-1].mod_point[1+num_try].y-1 > 0)
            {
               modle_point temp_point;
               temp_point.x = path_try[i-1].mod_point[1+num_try].x;
               temp_point.y = path_try[i-1].mod_point[1+num_try].y-1;
               pathwith_modlepoint new_pointpath;
               for (int index = 0; index < path_try[i-1].mod_point.size(); index++)
               {
                   new_pointpath.mod_point.push_back(path_try[i-1].mod_point[index]);
               }
               new_pointpath.mod_point.push_back(temp_point);
               float road_cost[3];
               point_to_Road(path_try[i-1].mod_point[1+num_try],temp_point,Road,road_cost);
               float cost_turn = 0;
               if(path_try[i-1].mod_point[1+num_try].x != path_try[i-1].mod_point[1+num_try-1].x)
               {
                   cost_turn = 0.8;
               }
               if(path_try[i-1].mod_point[1+num_try-1].y == (path_try[i-1].mod_point[1+num_try].y-1))
               {
                   cost_turn = 2;
               }
               new_pointpath.cost = road_cost[2] + path_try[i-1].cost + cost_turn;
               path_try.push_back(new_pointpath);
               if ((temp_point.x == target_point[0]) && (temp_point.y == target_point[1]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[2];
                   end_point.y = target_point[3];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_positive;
                   path_ok.push_back(maybe_path);
               }
               if ((temp_point.x == target_point[2]) && (temp_point.y == target_point[3]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[0];
                   end_point.y = target_point[1];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_negative;
                   path_ok.push_back(maybe_path);
               }
            }
            //try the down path 
            if (path_try[i-1].mod_point[1+num_try].y+1 < 5)
            {
               modle_point temp_point;
               temp_point.x = path_try[i-1].mod_point[1+num_try].x;
               temp_point.y = path_try[i-1].mod_point[1+num_try].y+1;
               pathwith_modlepoint new_pointpath;
               for (int index = 0; index < path_try[i-1].mod_point.size(); index++)
               {
                   new_pointpath.mod_point.push_back(path_try[i-1].mod_point[index]);
               }
               new_pointpath.mod_point.push_back(temp_point);
               float road_cost[3];
               point_to_Road(path_try[i-1].mod_point[1+num_try],temp_point,Road,road_cost);
               float cost_turn = 0;
               if(path_try[i-1].mod_point[1+num_try].x != path_try[i-1].mod_point[1+num_try-1].x)
               {
                  cost_turn = 0.8;
               }
               if(path_try[i-1].mod_point[1+num_try-1].y == (path_try[i-1].mod_point[1+num_try].y+1))
               {
                   cost_turn = 2;
               }
               new_pointpath.cost = road_cost[2] + path_try[i-1].cost + cost_turn;
               path_try.push_back(new_pointpath);
               if ((temp_point.x == target_point[0]) && (temp_point.y == target_point[1]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[2];
                   end_point.y = target_point[3];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_positive;
                   path_ok.push_back(maybe_path);
               }
               if ((temp_point.x == target_point[2]) && (temp_point.y == target_point[3]))
               {
                   pathwith_modlepoint maybe_path;
                   for (int index = 0; index < new_pointpath.mod_point.size(); index++)
                   {
                       maybe_path.mod_point.push_back(new_pointpath.mod_point[index]);
                   }
                   modle_point end_point;
                   end_point.x = target_point[0];
                   end_point.y = target_point[1];
                   maybe_path.mod_point.push_back(end_point);
                   maybe_path.cost = new_pointpath.cost + Road[targetroad_id-1].cost_negative;
                   path_ok.push_back(maybe_path);
               }
            }
            }
        }
        last_path_num = current_size + 1;
    }
   // ROS_INFO("num path is %d ", path_ok.size());
    return path_ok;
}

int findnearRoad(vehicleinfo position, Road_info Road[],int road_point[]) 
{
    float min_dis = 1000;
    int min_id = 1;
    float min_road_yaw = 0;
    int near_point_id = 0;
    for(int road_id = 1; road_id <= 24; road_id++)
    {
        for (int num_point = 1; num_point <= Road[road_id-1].centerline.size(); num_point++)
        {
            float dis;
            dis = sqrt(pow(position.x - Road[road_id-1].centerline[num_point-1].x,2) + pow(position.y - Road[road_id-1].centerline[num_point-1].y,2));
            if (dis < min_dis)
            {
                min_dis = dis;
                min_id = Road[road_id-1].ID;
                min_road_yaw = Road[road_id - 1].centerline[num_point-1].yaw;
                near_point_id = num_point - 1;
            }
        }     
    }
    float direction = sqrt(pow(position.yaw - Road[min_id -1].centerline[near_point_id].yaw,2));
    if ((direction < 1.57)||(direction > 4.71))
    {
       ID_to_point(min_id,road_point);
       return min_id;
    }
    else
    {
        ID_to_point((-1*min_id),road_point);
        return min_id;
    }
    
}

void point_to_Road(modle_point point_first, modle_point point_second, Road_info Road[], float road_cost[])
{
    int x1 = (float)point_first.x;  int y1 = point_first.y;   int x2 = point_second.x;  int y2 = point_second.y;
    road_cost[0] = (x2-x1) + (y2-y1);
    road_cost[1] = (7*(y1-1) + std::max(x1,x2)-1)*pow(x2-x1 , 2) + (7*std::max(y1,y2)-11+x1) * pow(y2-y1,2);
    if(road_cost[1] > 24)
    {ROS_INFO("%d ",(int)road_cost[1]);}
    if (road_cost[0] == 1)
    {
        road_cost[2] = Road[(int)road_cost[1]-1].cost_positive;
    }
    else
    {
        road_cost[2] = Road[(int)road_cost[1]-1].cost_negative;
    }
    return;
}



void ID_to_point(int id,int road_point[])
{
    if(id >0)
    {
       int a = id % 7;
       int b = id / 7;
       if (a == 0)
       {
          road_point[0] = 4; road_point[1] = b; 
          road_point[2] = 4; road_point[3] = b+1;
       }
       else if (a <= 3)
       {
          road_point[0] = a; road_point[1] = b+1; 
         road_point[2] = a+1; road_point[3] = b+1;
       }
       else
       {
          road_point[0] = a-3; road_point[1] = b+1;
          road_point[2] = a-3; road_point[3] = b+2;
       }
       return;
    }
    else
    {
       id = id * (-1);
       int a = id % 7;
       int b = id / 7;
       if (a == 0)
       {
          road_point[2] = 4; road_point[3] = b; 
          road_point[0] = 4; road_point[1] = b+1;
       }
       else if (a <= 3)
       {
          road_point[2] = a; road_point[3] = b+1; 
          road_point[0] = a+1; road_point[1] = b+1;
       }
       else
       {
          road_point[2] = a-3; road_point[3] = b+1;
          road_point[0] = a-3; road_point[1] = b+2;
       }
       return;
    }
}











