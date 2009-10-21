/*  Copyright Michael Otte, University of Colorado, 9-9-2009
 *
 *  This file is part of Mapper_CU.
 *
 *  Mapper_CU is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapper_CU is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Mapper_CU. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <sstream>
#include <vector>

#include <ros/ros.h>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"

#include "sensor_msgs/PointCloud.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"

#include "hokuyo_listener_cu/PointCloudWithOrigin.h"

#define HEIGHT 250
#define WIDTH 250
#define RESOLUTION .08
#define OBS_PRIOR 0.2

#define PROBMAP // HITMAP: map remembers all laser scanner obstacle data
                  // SIMPLEMAP: map updates to reflect current info from scanner (obstacles and free space)
                  // PROBMAP: map calculates prob of obstacle based on last MAP_MEMORY readings about a grid

#define MAP_MEMORY 100 // only used if PROBMAP is defined above

#define SCANNER_RANGE 5.5 // the range of the scanner in meters;

float MAP_INC = 1/(float)MAP_MEMORY; // each reading is worth this much probability

using namespace std;
        
struct POINT;
typedef struct POINT POINT;

struct MAP;
typedef struct MAP MAP;

// global ROS subscriber handles
ros::Subscriber laser_scan_sub;
ros::Subscriber pose_sub;

// global ROS publisher handles
ros::Publisher map_changes_pub;

// global ROS provide service server handles
ros::ServiceServer get_map_srv;


// globals
MAP* costmap = NULL;
vector<POINT> map_changes;
float robot_radius = .2;

/* -------------------------- POINT -------------------------------------*/
struct POINT
{
    float x;
    float y;
    float z;
};

/*---------------------- MAP --------------------------------------------*/
struct MAP
{
    float** cost;    
    int height;
    int width;
    float resolution;
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
      int** in_list;
    #endif           
};

// this creates and returns a pointer to a map struct
MAP* make_map(int height, int width, float resolution)
{
  MAP* map = (MAP*)calloc(1, sizeof(MAP));
  map->height = height;
  map->width = width;
  map->resolution = resolution; 
  
  map->cost = (float**)calloc(height, sizeof(float*));
  for (int y = 0; y < height; y++)  
  {
	map->cost[y] = (float*)calloc(width, sizeof(float)); 
    for(int x = 0; x < width; x++) 
      map->cost[y][x] = OBS_PRIOR;
  }

  #if defined(SIMPLEMAP) || defined(PROBMAP)
    map->in_list = (int**)calloc(height, sizeof(int*));
    for (int y = 0; y < height; y++)   
    {
      map->in_list[y] = (int*)calloc(width, sizeof(int));
      for(int x = 0; x < width; x++) 
        map->in_list[y][x] = -1;
    }
  #endif
                 
  return map;    
}


// this allocates all required memory for a map
void destroy_map(MAP* map)
{
  int y;
  if(map != NULL)
  {
    for (y = 0; y < map->height; y++)  
    {
	  if(map->cost[y] != NULL)
        free(map->cost[y]);
    }

    if(map->cost != NULL)
      free(map->cost);
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
      for (y = 0; y < map->height; y++)  
      {
        if(map->in_list[y] != NULL)
          free(map->in_list[y]);
      }
    
      if(map->in_list != NULL)
        free(map->in_list);
    #endif
                    
    map->height = 0;
    map->width = 0;
    free(map);
  }
}

// prints map on command line
void print_map(MAP* map)
{
  int y, x;

  if(map->cost != NULL)
  {
    printf("\n");  
    for (y = 0; y < map->height; y++)  
    {
      for (x = 0; x < map->width; x++)  
	    printf(" %f", map->cost[y][x]);
      printf("\n");  
    }
    printf("\n");  
  } 
}

// creates a blank map of height, width, and resolution, and cost cst
MAP* load_blank_map(int map_height, int map_width, float map_resolution, float cst)
{
  MAP* map = make_map(map_height, map_width, map_resolution);
     
  for(int i = 0; i < map_height; i++)
    for(int j = 0; j < map_width; j++)
        map->cost[i][j] = cst;   
  return map;
}

/*---------------------------- ROS Callbacks ----------------------------*/
void laser_scan_callback(const hokuyo_listener_cu::PointCloudWithOrigin::ConstPtr& msg)
{ 
  if(costmap == NULL)
      return;
    
  int length = msg->cloud.points.size();
  
  if(length <= 0)
    return;
 
  
  float x_scl = 1/costmap->resolution;
  float y_scl = 1/costmap->resolution;
  
  // change map, and remember changes
  
  
  //zeroth pass transform to grid coords (super annoying error, can't figure out why this needs to be +1)
  vector<POINT> scl_cloud_points;
  scl_cloud_points.resize(length);
  for(int i = 0; i < length; i++)
  {
    scl_cloud_points[i].x = x_scl*msg->cloud.points[i].x + 1;
    scl_cloud_points[i].y = y_scl*msg->cloud.points[i].y + 1;
  }
      
  #ifdef HITMAP // just add new hits to the map
          
    map_changes.resize(length);
    float xf, yf;
    int x, y;
    for(int i = 0; i < length; i++)
    {
      xf = scl_cloud_points[i].x;
      yf = scl_cloud_points[i].y;
    
      x = (int)xf;
      y = (int)yf;
    
      float this_cost = 1;
    
      if(x >= 0 && x < costmap->width && y >= 0 && y < costmap->height)
        costmap->cost[y][x] = this_cost;
    
      map_changes[i].x = (float)x;
      map_changes[i].y = (float)y;
      map_changes[i].z = this_cost;
    }
    
  #elif defined(SIMPLEMAP) || defined(PROBMAP) // add new hits, but first remove old hits the sensor had to see through in order to see the new hits
           
    float origin_x = x_scl*msg->origin.x;
    float origin_y = y_scl*msg->origin.y; 
          
    // first pass, see the max memory we will need to store all changes
    float min_x = scl_cloud_points[0].x;
    float max_x = scl_cloud_points[0].x;
    float min_y = scl_cloud_points[0].y; 
    float max_y = scl_cloud_points[0].y; 
    for(int i = 0; i < length; i++)
    {
      if(min_x > scl_cloud_points[i].x)
        min_x = scl_cloud_points[i].x;
      else if(max_x < scl_cloud_points[i].x)
        max_x = scl_cloud_points[i].x;
      
      if(min_y > scl_cloud_points[i].y)
        min_y = scl_cloud_points[i].y;
      else if(max_y < scl_cloud_points[i].y)
        max_y = scl_cloud_points[i].y;     
    }
          
    int max_changes = ((int)max_x - (int)min_x + 2)*((int)max_y - (int)min_y + 2);  
    int next_change = map_changes.size(); // remember previous changes that have not been sent out yet
    map_changes.resize(next_change + max_changes);  
          
    // create a small temp map to accumulate changes
    MAP*  tempmap = load_blank_map((int)max_y - (int)min_y + 1, (int)max_x - (int)min_x + 1, RESOLUTION, 0);

    // second pass, calculate empty spaces
    float rise, run, y_start = 0, y_end = 0, x_start = 0, x_end = 0;
    int direction_flag = -1;
    float** cost = costmap->cost;
    float** tcost = tempmap->cost;
    int map_x_offset = (int)min_x;
    int map_y_offset = (int)min_y;
    for(int i = 0; i < length; i++)
    {     
      rise =  scl_cloud_points[i].y - origin_y;
      run =  scl_cloud_points[i].x - origin_x;
    
      if(rise >= 0 && run >= 0) // work from origin to hit
      {
        y_start = origin_y;
        y_end = scl_cloud_points[i].y;
        x_start = origin_x;
        x_end = scl_cloud_points[i].x;
        
        if(run > rise) // work in x direction
          direction_flag = 0;
        else // work in y direction
          direction_flag = 1;
      }
      else if(rise < 0 && run < 0) // work from hit to origin
      {
        y_end = origin_y;
        y_start = scl_cloud_points[i].y;
        x_end = origin_x;
        x_start = scl_cloud_points[i].x;
        
        rise *= -1;
        run *= -1;
      
        if(run > rise) // work in x direction
          direction_flag = 0;
        else // work in y direction
          direction_flag = 1;  
      }
      else if(rise < 0)
      {
        if(run >= -rise) // work from origin to hit in x direction
        {
          y_start = origin_y;
          y_end = scl_cloud_points[i].y;
          x_start = origin_x;
          x_end = scl_cloud_points[i].x;
      
          direction_flag = 0;
        }
        else // work from hit to origin in y direction
        {
          y_end = origin_y;
          y_start = scl_cloud_points[i].y;
          x_end = origin_x;
          x_start = scl_cloud_points[i].x;
        
          rise *= -1;
          run *= -1;
      
          direction_flag = 1;  
        }
      }
      else if(run < 0)
      {
        if(-run >= rise) // work from hit to origin in x direction
        {  
          y_end = origin_y;
          y_start = scl_cloud_points[i].y;
          x_end = origin_x;
          x_start = scl_cloud_points[i].x;
           
          rise *= -1;
          run *= -1;
        
          direction_flag = 0;
        }
        else // work from origin to hit in y direction
        {
          y_start = origin_y;
          y_end = scl_cloud_points[i].y;
          x_start = origin_x;
          x_end = scl_cloud_points[i].x;
        
          direction_flag = 1;
        }
      }
   
      if(direction_flag == 0) // working in x direction
      {
        float m = rise/run;
        int r;
        int the_test = (int)x_end;
        for(int c = (int)x_start + 2; c < the_test; c++)
        {
          r = (int)(m*(c - x_start) + y_start);
          
          // update positions [r][c-1]  and [r][c] in large map
          tcost[r-map_y_offset][c-1-map_x_offset] -= 1;
          tcost[r-map_y_offset][c-map_x_offset] -= 1;    
        }
      }
      else // work in y direction
      {
        float m = run/rise;
        int c;
        int the_test = (int)y_end;
        for(int r = (int)y_start + 2; r < the_test; r++)
        {
          c = (int)(m*(r - y_start) + x_start);
          
          // update positions [r-1][c] and [r][c] in large map
          tcost[r-1-map_y_offset][c-map_x_offset] -= 1;
          tcost[r-map_y_offset][c-map_x_offset] -= 1;   
        }
      } 
    }
  
  
    // third pass, remember hits
    for(int i = 0; i < length; i++)
    {
      #ifdef SCANNER_RANGE  
      // only mark hits within range
      float no_scale_dx = msg->cloud.points[i].x-msg->origin.x;
      float no_scale_dy = msg->cloud.points[i].y-msg->origin.y;
      
      if(SCANNER_RANGE <= sqrt(no_scale_dx*no_scale_dx + no_scale_dy*no_scale_dy))
        continue;
      #endif
              
      #ifdef SIMPLEMAP  
        tcost[(int)(scl_cloud_points[i].y) - map_y_offset][
              (int)(scl_cloud_points[i].x) - map_x_offset] = 1;   
      #elif defined(PROBMAP)
        tcost[(int)(scl_cloud_points[i].y) - map_y_offset][
              (int)(scl_cloud_points[i].x) - map_x_offset] += 1;   
      #endif
      
    }  

    // fourth pass (first and only pass through the temp map)
    int** in_list = costmap->in_list;
    int height = costmap->height;
    int width = costmap->width;
    for(int r = 0; r < tempmap->height; r++)
    {
      for(int c = 0; c < tempmap->width; c++)
      { 
        if(tcost[r][c] != 0)
        {  
          int global_r = r+map_y_offset;
          int global_c = c+map_x_offset;
              
          if(global_r >= 0 && global_r < height && global_c >= 0 && global_c < width)
          {  
            if(in_list[global_r][global_c] == -1) // not already in list
            {  
              if(tcost[r][c] < 0 && cost[global_r][global_c] == 0) // don't need to report this change
                continue;
              if(tcost[r][c] > 0 && cost[global_r][global_c] == 1) // don't need to report this change
                continue; 
              
              map_changes[next_change].x = (float)(global_c);
              map_changes[next_change].y = (float)(global_r);
          
              in_list[global_r][global_c] = next_change;
              next_change++;
            }
             
            #ifdef SIMPLEMAP
              if(tcost[r][c] < 0)
              {
                cost[global_r][global_c] = 0;
                map_changes[in_list[global_r][global_c]].z = 0;
              }
              else
              {
                cost[global_r][global_c] = 1; 
                map_changes[in_list[global_r][global_c]].z = 1; 
              }
            #elif defined(PROBMAP)
              float new_cost = cost[global_r][global_c] + MAP_INC*tcost[r][c];
            
              if(new_cost < 0)
              {
                cost[global_r][global_c] = 0;
                map_changes[in_list[global_r][global_c]].z = 0;
              }
              else if(new_cost > 1)
              {
                cost[global_r][global_c] = 1;
                map_changes[in_list[global_r][global_c]].z = 1;
              }
              else
              {
                cost[global_r][global_c] = new_cost;  
                map_changes[in_list[global_r][global_c]].z = new_cost;
              }
            #endif

          }
        }
      }
    }
    map_changes.resize(next_change);
    destroy_map(tempmap);
  
  #endif
  scl_cloud_points.resize(0);       
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
  if(costmap == NULL)
    return;
      
  // mark grids within the radius of the robot as safe 
  float resolution = costmap->resolution;
  float pose_x = msg->pose.position.x/resolution;
  float pose_y = msg->pose.position.y/resolution;
  float rob_rad = robot_radius/resolution;
  
  int width = costmap->width;
  int height = costmap->height;
  
  int start_r = (int)(pose_y - rob_rad);
  if(start_r < 0)
    start_r = 0;
          
  int end_r = (int)(pose_y + rob_rad + 1);
  if(end_r >= height)
    end_r = height-1;
 
  int start_c = (int)(pose_x - rob_rad);
  if(start_c < 0)
    start_c = 0;
  
  int end_c = (int)(pose_x + rob_rad + 1);
  if(end_c >= width)
    end_c = width-1;
  
  int max_changes = (end_r - start_r)*(end_c - start_c);
  float** cost = costmap->cost;
  
  #if defined(SIMPLEMAP) || defined(PROBMAP)
  int** in_list = costmap->in_list;
  #endif
  
  int next_change = map_changes.size();
  map_changes.resize(next_change+max_changes);
  
  float dist;
  for(int r = start_r; r <= end_r; r++)
  {
    for(int c = start_c; c <= end_c; c++)   
    {
      float x_dist = ((float)c - .5 - pose_x);
      float y_dist = ((float)r - .5 - pose_y);
      
      dist = sqrt(x_dist*x_dist + y_dist*y_dist);
      
      if(dist < rob_rad)
      {
        #if defined(SIMPLEMAP) || defined(PROBMAP)
          if(in_list[r][c] == -1) // not already in list
          {  
            if(cost[r][c] == 0) // don't need to report this change
              continue;

            map_changes[next_change].x = (float)c;
            map_changes[next_change].y = (float)r;
            in_list[r][c] = next_change;
            next_change++;
          }
        
          // change costmap
          cost[r][c] = 0;
          map_changes[in_list[r][c]].z = 0; 
        #elif defined(HITMAP)    
          if(cost[r][c] == 0) // don't need to report this change
            continue;

          map_changes[next_change].x = (float)c;
          map_changes[next_change].y = (float)r;
          next_change++;   
          
          // change costmap
          cost[r][c] = 0;
        #endif
      }
    }
  }
  map_changes.resize(next_change);
      
}

bool get_map_callback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &resp)
{
 if(costmap == NULL)
   return false;
  
  int width = costmap->width;
  int height = costmap->height;
  
  resp.map.header.frame_id = "/2Dmap";
  resp.map.info.width = width;
  resp.map.info.height = height;
  resp.map.info.resolution = costmap->resolution;
           
  resp.map.data.resize(height*width);
          
  float** cost = costmap->cost;

  #if defined(SIMPLEMAP) || defined(PROBMAP)
  int** in_list = costmap->in_list;
  #endif
          
  int i = 0;
  for(int r = 0; r < height; r++)
  {
    for(int c = 0; c < width; c++)
    {
      resp.map.data[i] = (uint8_t)(100*cost[r][c]);
      #if defined(SIMPLEMAP) || defined(PROBMAP)
        in_list[r][c] = -1;
      #endif
      i++;
    }
  }
  
   return true;
}


/*---------------------- ROS Publisher Functions ------------------------*/
void publish_map_changes()
{
  int length = map_changes.size();
    
  if(length <= 0)
      return;
  
  sensor_msgs::PointCloud msg;  
  
  msg.points.resize(length);

  #if defined(SIMPLEMAP) || defined(PROBMAP)
  int** in_list = costmap->in_list;
  #endif
  
  for(int i = 0; i < length; i++)
  {
  
    msg.points[i].x = map_changes[i].x;
    msg.points[i].y = map_changes[i].y;
    msg.points[i].z = map_changes[i].z;
    
    #if defined(SIMPLEMAP) || defined(PROBMAP)
    // mark as not in change list
    in_list[(int)map_changes[i].y][(int)map_changes[i].x] = -1;
    #endif
  } 
  
  map_changes_pub.publish(msg);
 
  map_changes.resize(0);
}



int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mapper_cu");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    
    destroy_map(costmap);
    costmap = load_blank_map(HEIGHT, WIDTH, RESOLUTION, OBS_PRIOR);

    // set up subscribers
    laser_scan_sub = nh.subscribe("/cu/laser_scan_cu", 1, laser_scan_callback);
  	pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
    
    // set up publishers
    map_changes_pub = nh.advertise<sensor_msgs::PointCloud>("/cu/map_changes_cu", 1);
    
    // set up service servers
    get_map_srv = nh.advertiseService("/cu/get_map_cu", get_map_callback);
 
   while (ros::ok()) 
   {
       publish_map_changes();
       
       ros::spinOnce();
       loop_rate.sleep();
   }
    
    laser_scan_sub.shutdown();
    map_changes_pub.shutdown();
      
    destroy_map(costmap);
 
}
