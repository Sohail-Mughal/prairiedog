/*  Copyright Michael Otte, University of Colorado, 9-9-2009
 *
 *  This file is part of Multi_Robot_Planner_CU.
 *
 *  Multi_Robot_Planner_CU is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Multi_Robot_Planner_CU is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Base_Planner_CU. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 */

// this node assumes map is received as a probability occupancy grid
// cost is determined by the function prob_to_cost().

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>

#include <math.h>
#include <list>
#include <sstream>
#include <iostream>   // file I/O
#include <fstream>   // file I/O
#include <vector>
#include <pthread.h>
#include </usr/include/GL/glut.h>	    // GLUT
#include </usr/include/GL/glu.h>	    // GLU
#include </usr/include/GL/gl.h>	        // OpenGL

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GridCells.h"

#include "sensor_msgs/PointCloud.h"

#include "std_msgs/Int32.h"

#include "localization_cu/GetPose.h"

#include "geometry_msgs/Polygon.h"
#include "multi_robot_planner_cu/PolygonArray.h"
#include "multi_robot_planner_cu/TeamList_CU.h"

using namespace std;

#define LARGE 100000000
#define SMALL 0.00000001
#define PI 3.1415926535897 
//#define using_glut

 #define treev2            //options: treev3, treev3rho, treev2, treev2rho, rrt, rrtrho, rrtfast or nothing i.e. not defined
 #define using_smoothing   // when defined the rrt algorithms perform smoothing after each new solution is found
// #define save_time_data    // when defined stats [time, best, path_length, nodes_in_tree] are saved (for entire run) after the final solution is found
// #define use_kd_tree       // when defined a kd_tree is used for finding nearest neighbors (only useful with trees where this is allowed)  rrtrho, rrtfast

struct POSE;
typedef struct POSE POSE;

struct UPDATE;
typedef struct UPDATE UPDATE;

#ifdef use_kd_tree
  #include "kd_tree.cpp"  
#endif
#include "MultiRobotComs.h" 
#include "NavScene.h"
#include "MultiRobotWorkspace.h"
#include "Workspace.h"
#include "Cspace.h"
#include "MultiAgentSolution.h"
#include "glut_functions.h"
#include "helper_functions.h"

#define pre_calculated_free_space
      
bool want_clean_start = true;  // if true, we wait for other ros procs to start and also until pose and goal are recieved before doing anything else

float lookup_sum = 0;
float n_lookup = 0;

float out_collision = 0;
float n_collision = 0;

float out_sum = 0;
float n_out = 0;

float elookup_sum = 0;
float en_lookup = 0;

float eout_collision = 0;
float en_collision = 0;

float eout_sum = 0;
float en_out = 0;

int mode = 0;  // 0: partial solution sharing, 1: solution sharing, 2: single robot (only agent 0 actively plans)
int add_points_to_messages = 0; // 1 = yes, 0 = no
int uni_tree_build = 0; // sends trees in files 1 = yes, 0 = no

int add_best_path_to_this_c_space = 1; //1 = yes

int using_edge_lookup_table = 0; // 1 : yes

int attempt_random_path_improve = 1; // 1: once per loop we try to rerout a random node through its best parent (only with trees where this is allowed)

float rand_move_dist = 2; // > 0, move this dist toward random point from old config when using treev3rho or rrtrho

int num_points_per_file = 1000; // put data about this many edges and points into a message, only used if add_points_to_messages = 1
int num_edges_per_file = 1000; // put data about this many edges and points into a message, only used if add_points_to_messages = 1
int num_nodes_per_file = 100; // put data about up to this many nodes in a file, only used if uni_tree_build = 1
int pctr = 0;
int ectr = 0;
int pctr_all = 0;
int ectr_all = 0;

// temp message location, here most recent messages are stored from other robots until they can be read
char message_dir[] = "../temp_data";  // directory
float message_wait_time = 1; //(sec) can be overridden with command line input, this long between message sends
float sync_message_wait_time = 1; //(sec) can be overridden with command line input, this long between message sends during sync phase

// globals used for keeping track of glut display attributes, only used if using_glut is defined on compile
int display_size_init = 600; // pixels
int menu_pixels = 35;
int win_width = 1;
int win_height = 1;
float win_pan_x = 0;
float win_pan_y = 0;
float win_aspect_ratio = 1;
float scale_factor = 1; // radius of the size of the world

// globals
vector<float> startc;
vector<float> goalc;
float dist_threshold;
bool found_path;
int next_ind_animate = -1;
int display_flag = 0;
unsigned int the_seed; // rand seed
 
int min_num_agents = 1; // waits until this many agents have contact before planning (this number includes this agent), -1 means wait for all

// this map stuff may no longer be used
float map_resolution = -1; // reset later to proper value
float map_width = 0;
float map_height = 0;
        

// this map stuff is used
float default_map_x_size = -1;  // used only when finding single robot path, reset 
float default_map_y_size = -1;  // used only when finding single robot path, reset

float DISTANCE_METRIC = 0; // used only to calculate and compare path lengths, 0 = sum of all robots path segments, 1 = max robot segment length per time step (tends to cause robots to move at same time)

vector<float> x_temp_vec; // used to help debug
vector<float> y_temp_vec; // used to help debug

bool debugging_flag_1 = false;

float rrt_improvement_ratio = 1; //.95; // new rrts need to be at max this*(old cost) (when rrt type methods are used)

#ifdef save_time_data
  vector<float> time_stats;
  vector<float> nodes_in_tree_stats;
  vector<float> best_path_len_stats;
  vector<int> collision_checks_stats;
  int total_collision_checks = 0;
#endif
clock_t start_time;

clock_t last_chop_t;               // yes, a bit messy putting this and the next 2 here
double chop_time_limit = LARGE;
double chop_time_limit_mult = 4; // mait max this * time it took to grow last tree before choping
double chop_time_limit_mult_b = 1.5; // after a tree is chopped, next time wait this much times as much as we waited last time before chopping again
        
int max_message_size = 5000;

NavScene Scene; // this is the NavScene we are using
Cspace Cspc;  // this is the Cspace we are using
MultiAgentSolution MultAgSln;  // this is the MultiAgentSolution we are using
GlobalVariables Globals;       // note, GlobalVariables defined in MultiRobotComs.h
vector<vector<float> > ThisAgentsPath; // holds the path that is sent to the controller each point is [x y rotation]
vector<float> Parametric_Times; // holds time parametry of best solution
  
bool JOIN_ON_OVERLAPPING_AREAS = false; // if true, then we conservatively combine teams based on overlappingplanning areas. If false, then teams are only combined if paths intersect (or cause collisions)

float path_conflict_combine_dist = 3; // distance robots have to be near each other to combine teams if thier paths conflict
float team_combine_dist = 0.5;  // distance robots have to be near to each other to combine teams
float team_drop_dist = 5;     // distance robots have to be away from each other to dissolve teams
float team_drop_time = 5;    // after this long without hearing from a robot we drop it from the team
        
bool robot_is_moving = false;
float change_plase_thresh = .02; // if start or goal change less than this, then we say they are the same

#ifdef pre_calculated_free_space
vector<vector<float> > all_free_space;  // holds freespace points to resolution of bitmap map
vector<vector<float> > free_space;      // updated for each search to hold subspace of all_free_space within the planning area, transformed approperiatly
#endif

bool use_smart_plan_time = false; // if min_planning_time is input <= 0 then this gets set to true, and min_planning_time is adjusted based on circumstance
float plan_time_mult = 2;         // if(use_smart_plan_time) then plan for at least plan_time_mult X time it takes to compute first solution 
float smart_min_time_to_plan = 5; // if(use_smart_plan_time) then plan for at least smart_min_time_to_plan
bool calculated_smart_plan_time = false; //set to true once we calcualte the above

vector<float> drop_point;         // stores the point on the map at which we can drop team members

float very_first_path_length = -1;

/* ----------------------- POSE -----------------------------------------*/
struct POSE
{
    float x;
    float y;
    float z;
    
    float qw;
    float qx;
    float qy;
    float qz; 
    
    float alpha; // rotation in 2D plane
    
    float cos_alpha;
    float sin_alpha;
};


// this creates and returns a pointer to a POSE struct
POSE* make_pose(float x, float y, float z, float alpha)
{
  POSE* pose = (POSE*)calloc(1, sizeof(POSE));
  pose->x = x;
  pose->y = y;
  pose->z = z;
  pose->alpha = alpha;

  float r = sqrt(2-(2*cos(pose->alpha)));

  if(r == 0)
    pose->qw = 1;
  else
    pose->qw = sin(pose->alpha)/r; 

  pose->qx = 0;
  pose->qy = 0;
  pose->qz = r/2; 

  pose->cos_alpha = cos(pose->alpha);
  pose->sin_alpha = sin(pose->alpha);
  
  return pose;
}


// this deallocates all required memory for a POSE
void destroy_pose(POSE* pose)
{
  if(pose != NULL)
    free(pose);
    pose = NULL;
}

// prints pose on command line
void print_pose(POSE* pose)
{
  if(pose != NULL)
  {
    printf("\n");  
    printf("x: %f, y: %f, z: %f \n",pose->x, pose->y, pose->z);  
    printf("qw: %f, qx: %f, qy: %f, qz: %f \n", pose->qw, pose->qx, pose->qy, pose->qz);  
    printf("\n");  
  } 
}

#include "helper_functions.cpp"
#include "NavScene.cpp"
#include "MultiRobotWorkspace.cpp"  
#include "Workspace.cpp"
#include "Cspace.cpp"
#include "MultiAgentSolution.cpp"
#include "MultiRobotComs.cpp" 
#include "glut_functions.cpp"


// stuff from original base_planner_cu
float OBSTACLE_COST = 10000;         // the cost of an obstacle (of probability 1) note: this should be greater than the max path length
float robot_radius = .2;             // (m), radius of the robot
float safety_distance = .1;          // (m),  distance that must be maintained between robot and obstacles

// global ROS subscriber handles
ros::Subscriber pose_sub;
ros::Subscriber goal_sub;

// global ROS publisher handles
ros::Publisher global_path_pub;
ros::Publisher system_update_pub;
ros::Publisher planning_area_pub;   
ros::Publisher obstacles_pub; 
ros::Publisher team_list_pub;

// globals for robot and goal
POSE* robot_pose = NULL;
POSE* goal_pose = NULL;


bool new_goal = false;       
bool change_token_used = false;
bool reload_map = false; 
        

/*--------------------------- ROS callbacks -----------------------------*/
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
  //printf("--- 1 \n");  
    
  /*
  if((msg->pose.position.x/map_resolution) < 1.)
    return;
  if((msg->pose.position.x/map_resolution) > map_width - 1.)
    return;
  if((msg->pose.position.y/map_resolution) < 1.)
    return;
  if((msg->pose.position.y/map_resolution) > map_height - 1.)
    return;
  */
    
  if(robot_pose == NULL)
    robot_pose = make_pose(0,0,0,0);
  
  while(change_token_used)
    {printf(" change token used, pose \n");}
  change_token_used = true;

  robot_pose->x = msg->pose.position.x;
  robot_pose->y = msg->pose.position.y;
  robot_pose->z = msg->pose.position.z;
  robot_pose->qw = msg->pose.orientation.w;
  robot_pose->qx = msg->pose.orientation.x;
  robot_pose->qy = msg->pose.orientation.y;
  robot_pose->qz = msg->pose.orientation.z; 
  
  float qw = robot_pose->qw;
  float qx = robot_pose->qx;
  float qy = robot_pose->qy;
  float qz = robot_pose->qz; 
  
  robot_pose->cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
  robot_pose->sin_alpha = 2*qw*qz + 2*qx*qy;
  robot_pose->alpha = atan2(robot_pose->sin_alpha, robot_pose->cos_alpha);
  //printf(" new pose: \n");
  //print_pose(robot_pose);
  //getchar();
  change_token_used = false;
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{  
   //printf("--- 2 \n");   
    
  /*
  if((msg->pose.position.x/map_resolution) < 1.)
    return;
  if((msg->pose.position.x/map_resolution) > map_width - 1.)
    return;
  if((msg->pose.position.y/map_resolution) < 1.)
    return;
  if((msg->pose.position.y/map_resolution) > map_height - 1.)
    return;
  */
    
  if(goal_pose == NULL)
    goal_pose = make_pose(0,0,0,0);
  else if(goal_pose->x == msg->pose.position.x && 
          goal_pose->y == msg->pose.position.y &&
          goal_pose->z == msg->pose.position.z && 
          goal_pose->qw == msg->pose.orientation.w && 
          goal_pose->qx == msg->pose.orientation.x && 
          goal_pose->qy == msg->pose.orientation.y && 
          goal_pose->qz == msg->pose.orientation.z) // if it is the same as the old pose, then do nothing
      return;
  
  printf("new goal -------------------------------------------------------------------\n");
  

  while(change_token_used)
    {printf(" change token used, goal \n");}
  change_token_used = true;

  goal_pose->x = msg->pose.position.x;
  goal_pose->y = msg->pose.position.y;
  goal_pose->z = msg->pose.position.z;
  goal_pose->qw = msg->pose.orientation.w;
  goal_pose->qx = msg->pose.orientation.x;
  goal_pose->qy = msg->pose.orientation.y;
  goal_pose->qz = msg->pose.orientation.z; 
  
  float qw = goal_pose->qw;
  float qx = goal_pose->qx;
  float qy = goal_pose->qy;
  float qz = goal_pose->qz; 
  
  goal_pose->cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
  goal_pose->sin_alpha = 2*qw*qz + 2*qx*qy;
  goal_pose->alpha = atan2(goal_pose->sin_alpha, goal_pose->cos_alpha);
  
  //print_pose(goal_pose);
  new_goal = true;
  
  //Globals.planning_iteration[Globals.agent_number]++;
  Globals.master_reset = true; // goal change, so reset
  printf("master reset due to goal change \n");
  change_token_used = false;
}


/*---------------------- ROS Publisher Functions ------------------------*/
void publish_global_path(const vector<vector<float> >& path, const vector<float>& times_p)
{   
  while(change_token_used)
    {printf("change token used, publish path \n");}
  change_token_used = true;

  int length = path.size();
    
  if(length <= 0)
  {
    change_token_used = false;
    return;
  }
  
  nav_msgs::Path msg;  
  
  msg.header.frame_id = "/map_cu";
  msg.poses.resize(length);
  
  for(int i = 0; i < length; i++)
  {
    if(path[i].size() < 3)
    {
      change_token_used = false;
      return;
    }
    
    msg.poses[i].pose.position.x = path[i][0];
    msg.poses[i].pose.position.y = path[i][1];
    msg.poses[i].pose.position.z = times_p[i];  // time schedule
  
    // orientation
    float r = sqrt(2-(2*cos(path[i][2])));

    if(r == 0)
      msg.poses[i].pose.orientation.w = 1;
    else
      msg.poses[i].pose.orientation.w = sin(path[i][2])/r; 
  
    
    msg.poses[i].pose.orientation.x = 0;
    msg.poses[i].pose.orientation.y = 0;
    msg.poses[i].pose.orientation.z = r/2;  
    
    //printf("planner theta: %f (%f, %f, %f, %f)\n", path[i][2],  msg.poses[i].pose.orientation.w, msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z);
  } 
  global_path_pub.publish(msg); 

  //printf("sending global path %d \n",length);
  change_token_used = false;
}

void publish_system_update(int data)
{
  std_msgs::Int32 msg;  
  
  msg.data = data;
  system_update_pub.publish(msg); 
}

void publish_planning_area(const NavScene& S)
{
  geometry_msgs::Polygon msg;

  float x_min = -S.translation[0];
  float y_min = -S.translation[1];
  float x_max = x_min + S.dim_max[0];
  float y_max = y_min + S.dim_max[1];        

  msg.points.resize(4);

  msg.points[0].x = x_min;
  msg.points[0].y = y_min;
  msg.points[0].z = 0;

  msg.points[1].x = x_max;
  msg.points[1].y = y_min;
  msg.points[1].z = 0;

  msg.points[2].x = x_max;
  msg.points[2].y = y_max;
  msg.points[2].z = 0;

  msg.points[3].x = x_min;
  msg.points[3].y = y_max;
  msg.points[3].z = 0;

  planning_area_pub.publish(msg); 
}

void publish_obstacles(const NavScene& S)
{
  multi_robot_planner_cu::PolygonArray msg;

  int num_polygons = S.polygon_list.size();
  
  msg.header.frame_id = "/map_cu";
  msg.polygons.resize(num_polygons);

  for(int i = 0; i < num_polygons; i++)
  {
    int num_points = S.polygon_list[i].size();
    msg.polygons[i].points.resize(num_points);
    
    for(int j = 0; j < num_points; j++)
    {
      msg.polygons[i].points[j].x = S.polygon_list[i][j][0] - S.translation[0];
      msg.polygons[i].points[j].y = S.polygon_list[i][j][1] - S.translation[1];
      msg.polygons[i].points[j].z = 0;        
    }
  }

  obstacles_pub.publish(msg); 
}

void publish_team_list(const vector<bool>& TL)
{
  //printf("publishing team list:\n");
  multi_robot_planner_cu::TeamList_CU msg;
  msg.data.resize(TL.size());

  for(uint i = 0; i < TL.size(); i++)
  {
    msg.data[i] = TL[i];
    //printf("[%d], ", msg.data[i]);
  }

//printf("\n");

  team_list_pub.publish(msg); 
}


/*----------------------- ROS service functions -------------------------*/

bool load_pose()
{
    
  //printf("--- 6 \n"); 
   
  localization_cu::GetPose::Request req;
  localization_cu::GetPose::Response resp;
  ROS_INFO("Requesting the pose...\n");
  if( !ros::service::call("/cu/get_pose_cu", req, resp) )
  {
    ROS_INFO("load pose failed\n");
    return false;
  }
  
  /*
  if((resp.pose.pose.position.x/map_resolution) < 1.)
    return false;
  if((resp.pose.pose.position.x/map_resolution) > map_width - 1.)
    return false;
  if((resp.pose.pose.position.y/map_resolution) < 1.)
    return false;
  if((resp.pose.pose.position.y/map_resolution) > map_height - 1.)
    return false;
   */
  
  if(robot_pose == NULL)
    robot_pose = make_pose(0,0,0,0);
  
  robot_pose->x = resp.pose.pose.position.x;
  robot_pose->y = resp.pose.pose.position.y;
  robot_pose->z = resp.pose.pose.position.z;
  robot_pose->qw = resp.pose.pose.orientation.w;
  robot_pose->qx = resp.pose.pose.orientation.x;
  robot_pose->qy = resp.pose.pose.orientation.y;
  robot_pose->qz = resp.pose.pose.orientation.z; 
  
  float qw = robot_pose->qw;
  float qx = robot_pose->qx;
  float qy = robot_pose->qy;
  float qz = robot_pose->qz; 
  
  robot_pose->cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
  robot_pose->sin_alpha = 2*qw*qz + 2*qx*qy;
  robot_pose->alpha = atan2(robot_pose->sin_alpha, robot_pose->cos_alpha);
  //printf(" new pose: \n");
  //print_pose(robot_pose);
  //getchar();
  
  return true;
}

bool load_goal()
{
   //printf("--- 7 \n");   
    
  localization_cu::GetPose::Request req;
  
  localization_cu::GetPose::Response resp;
  ROS_INFO("Requesting the goal...\n");
  if( !ros::service::call("/cu/get_goal_cu", req, resp) )
  {
    ROS_INFO("load goal failed\n");
    return false;
  }
  
  if(goal_pose == NULL)
    goal_pose = make_pose(0,0,0,0);
  else if(goal_pose->x == resp.pose.pose.position.x && 
          goal_pose->y == resp.pose.pose.position.y &&
          goal_pose->z == resp.pose.pose.position.z && 
          goal_pose->qw == resp.pose.pose.orientation.w && 
          goal_pose->qx == resp.pose.pose.orientation.x && 
          goal_pose->qy == resp.pose.pose.orientation.y && 
          goal_pose->qz == resp.pose.pose.orientation.z) // if it is the same as the old pose, then do nothing
  {
   return false;
  }
  
  /*
  if((resp.pose.pose.position.x/map_resolution) < 1.)
    return false;
  if((resp.pose.pose.position.x/map_resolution) > map_width - 1.)
    return false;
  if((resp.pose.pose.position.y/map_resolution) < 1.)
    return false;
  if((resp.pose.pose.position.y/map_resolution) > map_height - 1.)
    return false;
  */
  
  goal_pose->x = resp.pose.pose.position.x;
  goal_pose->y = resp.pose.pose.position.y;
  goal_pose->z = resp.pose.pose.position.z;
  goal_pose->qw = resp.pose.pose.orientation.w;
  goal_pose->qx = resp.pose.pose.orientation.x;
  goal_pose->qy = resp.pose.pose.orientation.y;
  goal_pose->qz = resp.pose.pose.orientation.z; 
  
  float qw = goal_pose->qw;
  float qx = goal_pose->qx;
  float qy = goal_pose->qy;
  float qz = goal_pose->qz; 
  
  goal_pose->cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
  goal_pose->sin_alpha = 2*qw*qz + 2*qx*qy;
  goal_pose->alpha = atan2(goal_pose->sin_alpha, goal_pose->cos_alpha);
  //printf(" new goal: \n");
  //print_pose(goal_pose);
  //getchar();
  
  new_goal = true;
  printf("new goal via service -------------------------------------------------------------------\n");
  return true;
}




// requests the map from the mapping system, stores free locations in a global list 
bool load_map(vector<vector<float> >& global_list)
{
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...\n");
  if( !ros::service::call("/cu/get_map_cu", req, resp) )
  {
    ROS_INFO("request failed\n");
    return NULL;
  }
  
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n", resp.map.info.width, resp.map.info.height, resp.map.info.resolution);

  printf("pre-calculating free space \n");
  
  if (resp.map.info.width == 0 && resp.map.info.height == 0)
    return false;
  

  global_list.resize(0);

  float map_resolution = resp.map.info.resolution;
  int map_width = resp.map.info.width;
  int map_height = resp.map.info.height;

  for(int i = 0; i < map_height; i+=2)
  {
    for(int j = 0; j < map_width; j+=2)
    {
      if(resp.map.data[i*map_width+j] < 50)  // less than 50% chance of obstacles here
      {
          
          
          
         vector<float> temp_vec(3,0);
         
         temp_vec[0] = ((float)j)*map_resolution;
         temp_vec[1]  = ((float)i)*map_resolution;
         // temp_vec[2] = 0;  // already set, note ignoring theta
         
         global_list.push_back(temp_vec);
      }
      
      
    }
  }
  return true;
}


// puts subset of all_f_s that is within sub_area into f_s
void get_valid_subset_freespace(vector<vector<float> >& all_f_s, vector<vector<float> >& f_s, vector<float>& team_bound_area_min,  vector<float>& team_bound_area_size)
{
  f_s.resize(0);

  vector<float> temp(3,0);

  for(uint i = 0; i < all_f_s.size(); i++)
  {
    if(all_f_s[i][0] < team_bound_area_min[0])
      continue;
    if(all_f_s[i][1] < team_bound_area_min[1])
      continue;

    temp[0] = all_f_s[i][0] - team_bound_area_min[0];
    if(temp[0] > team_bound_area_size[0])
      continue;

    temp[1] = all_f_s[i][1] - team_bound_area_min[1];
    if(temp[1] > team_bound_area_size[1])
      continue;

    f_s.push_back(temp);
  }
}

/*---------------------------- ROS tf functions -------------------------*/
void broadcast_map_tf()
{
  static tf::TransformBroadcaster br;  
    
  tf::Transform transform;   
  transform.setOrigin(tf::Vector3(0, 0, 0));
  
  //transform.setRotation(tf::Quaternion(global_map_theta_offset, 0, 0)); // this is currently ypr, but being depreciated and then changed to rpy
  tf::Quaternion Q;
  Q.setRPY(0, 0, 0);
  transform.setRotation(Q);
  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world_cu", "/map_cu"));  
}



int main(int argc, char** argv) 
{
  //argv[1] = this robot number (int), 0 .... n-1
  //argv[2] = total number of robots (int), n
  //argv[3] = min number of seconds allowed for planning
  //argv[4] = probability a message is sucessfully transmitted    
  //argv[5] = (1) then this goes into display mode
  //argv[6] = experiment name  
  //argv[7] = 1/(message rate)
  //argv[8] = base_ip
  //argv[9] = 1/(sync_message rate)    
  //argv[10] = robot radius 
  //argv[11] = rrt_prob_at_goal 
  //argv[12] = rrt_move_max 
  //argv[13] = rrt_theta_max  
  //argv[14] = rrt_resolution
  //argv[15] = rrt_angular_resolution
  //argv[16] = planning_border_width
   

  // remove old files from temp directory
  int unused_result;
  char system_call[200];
  sprintf(system_call,"rm -rf %s/*", message_dir); 
  unused_result = system(system_call);
    
  ros::init(argc, argv, "base_planner_cu");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
    
  #ifdef pre_calculated_free_space
  // wait until the map service is provided (we need its tf /world_cu -> /map_cu to be broadcast)
  ros::service::waitForService("/cu/get_map_cu", -1);
  
  // wait for a map --- using this to get list of free space
  while(!load_map(all_free_space) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep(); 
  }
  #endif
  
  // set up ROS topic subscriber callbacks
  pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
  goal_sub = nh.subscribe("/cu/goal_cu", 1, goal_callback);
    
  // set up ROS topic publishers
  global_path_pub = nh.advertise<nav_msgs::Path>("/cu/global_path_cu", 1);
  system_update_pub = nh.advertise<std_msgs::Int32>("/cu/system_update_cu", 10);
  planning_area_pub = nh.advertise<geometry_msgs::Polygon>("/cu/planning_area_cu", 1);
  obstacles_pub = nh.advertise<multi_robot_planner_cu::PolygonArray>("/cu/obstacles_cu", 1);
  team_list_pub = nh.advertise<multi_robot_planner_cu::TeamList_CU>("/cu/team_list_cu", 1);
  
  // spin ros once
  ros::spinOnce();
  loop_rate.sleep(); 
    
  #ifdef using_glut
    //  Initialize GLUT
    glutInit(&argc,argv);
  
    //  Request double buffered, true color window with Z buffering at 600x600
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(display_size_init,display_size_init);
    glutCreateWindow("RRT");
  
    //  Set GLUT callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    
    glutIdleFunc(idle);
    glutPostRedisplay();
  #endif
  
              
  // default values for possible command line inputs
  int agent_number = 0;  // this agent's id number
  int total_agents = 1;  // number of agents attempting to compute a solution
  float min_clock_to_plan = 10; // sec
  float prob_success = 1;
  bool display_path = false; //true;     
  char experiment_name[50];
  sprintf(experiment_name, "test");
  char base_ip[100];
  float robot_radius = 1.0;
  float prob_at_goal = .8;
  float move_max = 1;
  float theta_max = 2*PI;
  float resolution = .01;
  float angular_resolution = .3;
  float planning_border_width = 1;
  float target_mps = .2 ; // target speed m/s
  float target_rps = PI/8; // target turn rad/s
  
  if(argc > 8) // then there is an agent number associated with this (and we are running in multi agent mode), and a min time for planning, and may want display mode, and experiment name, and 1/(message rate), and we have a base ip
  {
    agent_number = atoi(argv[1]);
    total_agents = atoi(argv[2]);  
    min_clock_to_plan = (float)atof(argv[3]);
    prob_success = (float)atof(argv[4]);
    if(atoi(argv[5]) == 1)
      display_path = true;    
    sprintf(experiment_name, "%s",argv[6]);
    message_wait_time = (float)atof(argv[7]);
    sprintf(base_ip, "%s",argv[8]);
    
    if(argc > 9)
       sync_message_wait_time = (float)atof(argv[9]);
    else
       sync_message_wait_time = message_wait_time;     
    
   if(argc > 10)
     robot_radius = (float)atof(argv[10]);    
   
   if(argc > 11)
     prob_at_goal = (float)atof(argv[11]);
    
   if(argc > 12)
     move_max = (float)atof(argv[12]);
   
   if(argc > 13)
     theta_max = (float)atof(argv[13]);
   
   if(argc > 14)
     resolution = (float)atof(argv[14]);
    
   if(argc > 15)
     angular_resolution = (float)atof(argv[15]);
    
   if(argc > 16)
     planning_border_width = (float)atof(argv[16]);
    
  }
  else
  {  
    printf("error: need a bunch of stuff as inputs, you didn't have enough, \nhopefully parameter server has what I need \n");
    agent_number = 0;
    total_agents = 1;    
  }
  
  // stuff from paramiter server overrides command line inputs
  double double_input;
  int int_input;
  std::string string_input;
  if(ros::param::get("multi_robot_planner_cu/agent_id", int_input))
    agent_number = int_input;                                                        // this agent's id
  if(ros::param::get("multi_robot_planner_cu/total_agents", int_input))
    total_agents = int_input;                                                        // the total number of agents
  if(ros::param::get("multi_robot_planner_cu/min_clock_to_plan", double_input))
    min_clock_to_plan = (float)double_input;                                         // the total time allowed for planning
  if(ros::param::get("multi_robot_planner_cu/prob_message_success", double_input))
    prob_success = (float)double_input;                                              // the probability a message is sent sucessfully 
  if(ros::param::get("multi_robot_planner_cu/experiment_name", string_input))                       
    sprintf(experiment_name, "%s", string_input.c_str());                            // the name of this experiment
  if(ros::param::get("multi_robot_planner_cu/message_wait_time", double_input))
    message_wait_time = (float)double_input;                                         // the time between message sends in planning phase
  if(ros::param::get("multi_robot_planner_cu/sync_message_wait_time", double_input))
    sync_message_wait_time = (float)double_input;                                    // the time between message sends in sync phase
  if(ros::param::get("multi_robot_planner_cu/base_ip", string_input))                       
    sprintf(base_ip, "%s", string_input.c_str());                                    // the base_ip of all robots
  if(ros::param::get("multi_robot_planner_cu/robot_radius", double_input))
    robot_radius = (float)double_input;                                              // the radius of the robot
  if(ros::param::get("multi_robot_planner_cu/safety_distance", double_input))
    robot_radius += (float)double_input;                                             // the safety distance between robot and obstacles (adds to radius)
  if(ros::param::get("multi_robot_planner_cu/rrt_prob_at_goal", double_input))
    prob_at_goal = (float)double_input;                                              // probability a new edge goes at the goal
  if(ros::param::get("multi_robot_planner_cu/rrt_move_max", double_input))
    move_max = (float)double_input;                                                  // max move allowed during rrt generation
  if(ros::param::get("multi_robot_planner_cu/rrt_resolution", double_input))
    resolution = (float)double_input;                                                // resolution of rrt
  if(ros::param::get("multi_robot_planner_cu/rrt_angular_resolution", double_input))
    angular_resolution = (float)double_input;                                        // angular resolution of rrt 
  if(ros::param::get("multi_robot_planner_cu/planning_border_width", double_input))
    planning_border_width = (float)double_input;                                     // this much more space provided around robots for planning
  if(ros::param::get("multi_robot_planner_cu/target_speed", double_input))
    target_mps = (float)double_input;                                                // target speed m/s
  if(ros::param::get("multi_robot_planner_cu/target_turn", double_input))
    target_rps = (float)double_input;                                                // target turn speed rad/s
  if(ros::param::get("mapper_cu/map_x_size", double_input))
    default_map_x_size = (float)double_input;                                                // default map size x
  if(ros::param::get("mapper_cu/map_y_size", double_input))
    default_map_y_size = (float)double_input;                                                // default map size y

  printf("I am agent #%d of %d, and can plan for %f seconds\nmessages sent with probability %f\nmessage_wait_time: %f, sync_message_wait_time: %f\n", agent_number, total_agents, min_clock_to_plan, prob_success, message_wait_time, sync_message_wait_time);
  
  if(min_clock_to_plan <= 0) // use smart planning, where we plan for max(MX,N) where M and N are defined up near the top of this file
  {
     printf("using smart plan time with max(%f(first_sln_time), %f) \n", plan_time_mult, smart_min_time_to_plan);
     use_smart_plan_time = true;
  }


  // set up random number generator 
  time_t seed;
  time(&seed);
  the_seed = (unsigned int)(seed*10) + (unsigned int)agent_number + (unsigned int)total_agents;
  srand(the_seed);
  
  if(want_clean_start)
  {      
    // wait for pose server to start
    ros::service::waitForService("/cu/get_pose_cu", -1);
      
    // wait for goal server to start
    ros::service::waitForService("/cu/get_goal_cu", -1);
    
    // wait until we have a goal and a robot pose
    while((goal_pose == NULL || robot_pose == NULL) && ros::ok())
    {
        
      printf("waiting for goal and/or pose \n");
      if(goal_pose == NULL)
      {
        //printf("waiting for goal\n");
      
        //load_goal();
        if(goal_pose != NULL)
        {
          printf("recieved goal: \n");
          print_pose(goal_pose);
        }
      }
      
      if(robot_pose == NULL)
      {
        //printf("waiting for pose\n"); 
      
        //load_pose();
        if(robot_pose != NULL)
        {
          printf("recieved pose: \n");
          print_pose(robot_pose);
        }
      }
      
      // broadcast tf
      broadcast_map_tf();
      
      ros::spinOnce();
      loop_rate.sleep();   

      usleep(sync_message_wait_time*1000000); 
    }
  }
  else
  {
    if(robot_pose == NULL)
      robot_pose = make_pose(0,0,0,0);
    if(goal_pose == NULL)
      goal_pose = make_pose(1,1,0,3.14);
  }  
  
  printf("have pose and goal: \n");
  print_pose(robot_pose);
  print_pose(goal_pose);
  
  // spin ros one more time to check callbacks
  if(ros::ok())
  {
    new_goal = false; 
    ros::spinOnce();  // check in on callbacks
  }
  
  clock_t start_time, now_time, last_time;  
  Globals.Populate(total_agents);  // sets master_reset to true, so globals are not used as they are being changed
      
  // set up globals used by communication threads
  Globals.agent_number = agent_number;
  Globals.InTeam[agent_number] = true;
  Globals.local_ID[agent_number] = 0;
  Globals.global_ID.push_back(agent_number);
  Globals.team_size = 1;  
  Globals.robot_pose = robot_pose;
  Globals.path_conflict_combine_dist = path_conflict_combine_dist;
  Globals.combine_dist = team_combine_dist;
  Globals.drop_dist = team_drop_dist;
  Globals.drop_time = team_drop_time;
          
  if(min_num_agents < 0)
    Globals.min_team_size = total_agents;
  else
    Globals.min_team_size = min_num_agents;

  sprintf(Globals.base_IP, "%s.", base_ip);
  sprintf(Globals.my_IP, "%s.%d", base_ip, agent_number+1);
  printf("ip addresses of all other robots: \n");
  for(int i = 0; i < total_agents; i++)
  {  
    if(i == agent_number)
      continue;
    
    // save ip of each robot
    sprintf(Globals.other_IP_strings[i], "%s.%d", base_ip, i+1);
    
    // setup socket address stuff for each robot
    struct hostent *hp_this = gethostbyname(Globals.other_IP_strings[i]);
        
    if(hp_this==0) 
    {
      error("Unknown host");
      return false;
    }

    Globals.other_addresses[i].sin_family = AF_INET;

    bcopy((char *)hp_this->h_addr, (char *)&Globals.other_addresses[i].sin_addr, hp_this->h_length); // copy in address    

    Globals.other_addresses[i].sin_port = htons(Globals.InPorts[i]);
    
    printf("#%d: %s \n", i, Globals.other_IP_strings[i]);
  }

  Globals.sync_message_wait_time = sync_message_wait_time;
  Globals.message_wait_time = message_wait_time;
  
  Globals.robot_radius = robot_radius;
  Globals.prob_at_goal = prob_at_goal;
  Globals.move_max = move_max;
  Globals.theta_max = theta_max;
  Globals.resolution = resolution;
  Globals.angular_resolution = angular_resolution;         
  Globals.planning_border_width = planning_border_width;
  
  // communication threads
  pthread_t Listener_thread, Sender_thread;
  //pthread_create( &Listener_thread, NULL, Robot_Listner_Ad_Hoc, &Globals);         
 
  Globals.default_map_x_size = default_map_x_size;
  Globals.default_map_y_size = default_map_y_size;


  // set this robots best path to be remaining at its initial position for a minute or two
  vector<float> path_for_wait_here(4,0);
  path_for_wait_here[0] = robot_pose->x;
  path_for_wait_here[1] = robot_pose->y;
  Globals.single_robot_solution.resize(0);
  Globals.single_robot_solution.resize(4,path_for_wait_here);
  Globals.single_robot_solution[1][2] = 10;
  Globals.single_robot_solution[2][2] = 20;
  Globals.single_robot_solution[3][2] = 30;


  timeval first_start_time;
  gettimeofday(&first_start_time, NULL);

  // each trip throught the loop is a complete planning and move cycle
  Globals.sender_Ad_Hoc_running = false;
  Globals.listener_active = false;
  while(!Globals.kill_master)
  {   

    // =========================================== starting/reseting phase (0) ===========================================
    // ----> this phase resets global data to reflect the new team size
    Globals.nav_state[Globals.agent_number] = 0;  
    Globals.nav_state_iteration[Globals.agent_number]++;

    // stop robot from moving
    if(robot_is_moving)
    {
      unused_result = system(system_call);  // remove old files
      start_time = clock();
      now_time = clock(); 
      while(difftime_clock(now_time,start_time) < 2.0) // wait for two seconds /////// USE DIFFERENT CLOCK FUNCTION !!!!!!!!!!!!
      {
        publish_system_update(1); // if we've reset, then tell the controller
        ros::spinOnce();  
        now_time = clock(); 
      }
    }  
    robot_is_moving = false;  

    timeval time_start_reset;  
    gettimeofday(&time_start_reset, NULL);
    while(Globals.master_reset && (Globals.sender_Ad_Hoc_running || Globals.listener_active))
    {
      timeval right_now;  
      gettimeofday(&right_now, NULL);

      if(difftime_timeval(right_now, time_start_reset) > 10.0) // in case listener has no chance to get a message, and thus stop blocking to sleep
      {
        printf("timeout while waiting for threads to stop \n");
        break;
      }

      if(Globals.sender_Ad_Hoc_running && Globals.listener_active)
        printf("master: both listener and sender are running\n");
      else if(Globals.sender_Ad_Hoc_running)
        printf("master: sender running\n");
      else if(Globals.listener_active)
        printf("master: listener running\n");
      else
        printf("master: other threads not running\n");

      usleep(100000); // sleep for 1/10 sec
    }
    


    // reset globals for a new planning cycle
    printf("master: resetting globals\n");
    Globals.Reset();   // sets master_reset to false

    // if using smart plan time then reset min time to plan ?????????
    if(use_smart_plan_time)
    {
      calculated_smart_plan_time = false;
      min_clock_to_plan = smart_min_time_to_plan;
      printf("using smart plan time : %f \n", min_clock_to_plan);
    }

    // kick off sender thread
    if(!Globals.sender_Ad_Hoc_running)
      pthread_create( &Sender_thread, NULL, Robot_Data_Sync_Sender_Ad_Hoc, &Globals);  // this is used for startup, to send data to other robots, will terminate on master_reset

    // kick off listener thread
    if(!Globals.listener_active)
      pthread_create( &Listener_thread, NULL, Robot_Listner_Ad_Hoc, &Globals);         
 


    // ================================ exchange prefered path phase and calculate start/goal phase (1) ============================
    // ----> This phase exchanges desired paths between robots so that each robot can calculate their new sub-start and 
    // ----> sub-goal for the next problem, based on the sub-area that will be used for planning
    Globals.nav_state[Globals.agent_number] = 1;  
    Globals.nav_state_iteration[Globals.agent_number]++;

    Globals.planning_iteration_single_solutions[Globals.agent_number] = Globals.planning_iteration[Globals.agent_number];

    // only need to worry about this if there are other robots in the team
    if(Globals.team_size > 1)
    {
      // do message passing (in sender thread) until we have everybody's updated prefered single robot paths
      while(!Globals.have_all_team_single_paths() && !Globals.master_reset)
      {
        printf("master: waiting for other member's single paths\n");
        Globals.output_state_data();
  
        publish_system_update(1); // if we've reset, then tell the controller
        ros::spinOnce();  
        usleep(100000); // sleep for 1/10 sec

        Globals.team_member_timeout(60.0);
      }

      // if problem is reset then restart the planning loop
      if(Globals.master_reset)
      {
        printf("master: restarting planning -1\n");
        continue;
      }

      // look at this agent's single path vs other agent's single paths and 
      // calculate start and goal for us based on intersections with other agents

      vector<float> sub_start;
      vector<float> sub_goal;

      Globals.other_robots_single_solutions[Globals.agent_number] = Globals.single_robot_solution;
      float preferred_min_planning_area_side_length = 1.0*Globals.team_size;
      float preferred_max_planning_area_side_length = 1.0*Globals.team_size;
      float accuracy_resolution = .05;
      float time_resolution = .05;

      printf("master: calculating sub_start and sub_goals \n");


      bool no_conflicts_between_sub_paths = false;
      if(Globals.revert_to_single_robot_path)
      {
        printf("master: single robot team\n");
        Globals.use_sub_sg = false;
      }
      else if(Globals.team_size <= 2)
      {
        Globals.use_sub_sg = false;
      }
      else if(calculate_sub_goal(Globals.other_robots_single_solutions, Globals.InTeam, Globals.agent_number, 
                            preferred_min_planning_area_side_length,  preferred_max_planning_area_side_length, 
                            robot_radius, accuracy_resolution, time_resolution, sub_start, sub_goal, no_conflicts_between_sub_paths) )
      {
        // if here then sub_start and sub_goal have changed
        printf("master: new sub area \n");

        Globals.start_coords[0].resize(3); 
        Globals.start_coords[0][0] = sub_start[0];
        Globals.start_coords[0][1] = sub_start[1];
        Globals.start_coords[0][2] = 0;

        Globals.goal_coords[0].resize(3); 
        Globals.goal_coords[0][0] = sub_goal[0];
        Globals.goal_coords[0][1] = sub_goal[1];
        Globals.goal_coords[0][2] = 0;

        Globals.use_sub_sg = true;

        Globals.sub_start_coords[Globals.agent_number] =  sub_start;
        Globals.sub_goal_coords[Globals.agent_number] =  sub_goal;
      }
      /*else if(no_conflicts_between_sub_paths)
      {
        printf("master: no conflicts between sub-paths\n");

        // reset single robot path 
        Globals.use_sub_sg = false;
        Globals.revert_to_single_robot_path = true;
      }*/
      else if(Globals.use_sub_sg)
      {
        printf("master: old sub area %u %u\n", sub_start.size(), sub_goal.size());
    
        // continue using old sub_start and sub_goal (which we have saved)
        Globals.start_coords[0].resize(3); 
        Globals.start_coords[0][0] = sub_start[0];
        Globals.start_coords[0][1] = sub_start[1];
        Globals.start_coords[0][2] = 0;

        Globals.goal_coords[0].resize(3); 
        Globals.goal_coords[0][0] = sub_goal[0];
        Globals.goal_coords[0][1] = sub_goal[1];
        Globals.goal_coords[0][2] = 0;

        Globals.sub_start_coords[Globals.agent_number] =  sub_start;
        Globals.sub_goal_coords[Globals.agent_number] =  sub_goal;
      }

      printf("master: Done calculating sub_start and sub_goals \n");
    }

    if(!Globals.use_sub_sg) // are not using sub area
    {
      printf("master: default sub area \n");

      Globals.goal_coords[0].resize(3); 
      Globals.goal_coords[0][0] = goal_pose->x;
      Globals.goal_coords[0][1] = goal_pose->y;
      Globals.goal_coords[0][2] = goal_pose->alpha;

      Globals.start_coords[0].resize(3); 
      Globals.start_coords[0][0] = robot_pose->x;
      Globals.start_coords[0][1] = robot_pose->y;
      Globals.start_coords[0][2] = robot_pose->alpha;

      Globals.sub_start_coords[Globals.agent_number][0] = robot_pose->x;
      Globals.sub_start_coords[Globals.agent_number][1] = robot_pose->y;

      Globals.sub_goal_coords[Globals.agent_number][0] = goal_pose->x;
      Globals.sub_goal_coords[Globals.agent_number][1] = goal_pose->y;
    }

    printf("master: Done setting sub-start and sub-goal data\n");



    // =========================================== exchange start/goal phase (2) ===========================================
    // ----> This phase exchanges sub-start and sub-goal
    Globals.nav_state[Globals.agent_number] = 2;  
    Globals.nav_state_iteration[Globals.agent_number]++;

    Globals.sub_start_and_goal_iteration[Globals.agent_number] = Globals.planning_iteration[Globals.agent_number];

    printf("My IP: %s\n",Globals.my_IP);
    printf("My start: %f %f %f\n", Globals.start_coords[0][0], Globals.start_coords[0][1], Globals.start_coords[0][2]);
    printf("My goal: %f %f %f\n", Globals.goal_coords[0][0], Globals.goal_coords[0][1], Globals.goal_coords[0][2]);
    
    // remember the start time
    start_time = clock();
    now_time = clock();
    last_chop_t = clock();

    Globals.have_calculated_start_and_goal = true;  // allows sender thread to move forward to message sync phase
    Globals.start_time = start_time;
 
    clock_t start_wait_t = clock();
    int world_dims = 3; // reset later
    if(!Globals.revert_to_single_robot_path) // do only if we need to calculate a path
    {
      // start-up phase loop (wait until we have min number of agents start and goal locations)
      while(!Globals.have_all_team_start_and_goal_data() && !Globals.master_reset)
      {
        printf("master: exchanging start and goals with team\n");
        Globals.output_state_data();

        publish_system_update(1); // if we've reset, then tell the controller
        ros::spinOnce(); 

        usleep(Globals.sync_message_wait_time*1000000);
        Globals.team_member_timeout(60.0);
      } 

      if(Globals.master_reset)
      {
        printf("master: master reset A\n");
        continue;  // a team member has been added, need to restart planning with more dimensions
      }
    
      printf("setting start %d and goal %d from sub_start and sub_goal\n", Globals.start_coords.size(), Globals.goal_coords.size());
      // use sub_start and sub_goal as start and goal
      Globals.start_coords.resize(Globals.team_size);
      Globals.goal_coords.resize(Globals.team_size);
      for(int i = 0; i < Globals.team_size; i++)
      {
        int global_i = Globals.global_ID[i];
  
        printf("agent %d (%d): [%f %f] [%f %f] \n", global_i, i, Globals.sub_start_coords[global_i][0], Globals.sub_start_coords[global_i][1],
                                                                 Globals.sub_goal_coords[global_i][0], Globals.sub_goal_coords[global_i][1]);
        Globals.start_coords[i].resize(3); 
        Globals.start_coords[i][0] = Globals.sub_start_coords[global_i][0];
        Globals.start_coords[i][1] = Globals.sub_start_coords[global_i][1];
        Globals.start_coords[i][2] = 0;  

        Globals.goal_coords[i].resize(3); 
        Globals.goal_coords[i][0] = Globals.sub_goal_coords[global_i][0];
        Globals.goal_coords[i][1] = Globals.sub_goal_coords[global_i][1];
        Globals.goal_coords[i][2] = 0; 

        Globals.have_info[i] = 1;
      }
      printf("Done setting start and goal from sub_start and sub_goal\n");


      // =========================================== planning phase (3) ===========================================
      // ----> This is the planning phase
      Globals.nav_state[Globals.agent_number] = 3;  
      Globals.nav_state_iteration[Globals.agent_number]++;

      // this is where the planning stuff starts
      // set up the scene based on all start/goal locations and any maps
      

      printf("loading globals \n");
      if(!Scene.LoadFromGlobals(Globals))
        return 0;
      printf("done loading globals \n");


      printf("loading map \n");
      char map_file[] = "../lab.txt";
      if(!Scene.LoadMapFromFile(map_file))
      {
        printf("master: unable to load map file: %s\n", map_file);
        return 0;
      }
      printf("done loading map \n");
   
      

      Scene.PrintSceneInfo(); 
      publish_planning_area(Scene);
      publish_obstacles(Scene);
      publish_team_list(Globals.InTeam);
      ros::spinOnce();
  
      int num_robots = Scene.num_robots;
      world_dims = Scene.world_dims;
      float robot_rad = Scene.robot_rad[0];      
      map_resolution = Scene.resolution*num_robots; //////////////////// this is kind of wierd, someday it may be a good idea to go down the rabbit hole and figure out if this actually affects anything, or if it is now ignorred by stuff
      startc = Scene.startC;
      goalc = Scene.goalC;
      dist_threshold = map_resolution;
    

      Cspc.W.Populate(num_robots, robot_rad, Scene.dim_max);
      Cspc.W.Gbls = &Globals;
    

      if(!Cspc.Populate(startc, goalc, num_robots*world_dims) && !Globals.kill_master)  // first case fail on invalid start or goal location
      {
        printf("!!!!!!!!!!!!! INVALID START OR GOAL !!!!!!!!!!!!!!\n");
        
        Globals.master_reset = true;
        printf("master: master reset due to invalid start or goal \n");

        sleep(1);
        continue;
      }

   
      MultAgSln.Populate(total_agents, agent_number, &Globals, world_dims);
      MultAgSln.obstacles_pub = &obstacles_pub;
      Globals.MAgSln = &MultAgSln;

      int iterations_left = -1; // negative means that time is used instead
  
      clock_t phase_two_start_t = clock();
    
      printf("master: resetting planning start time\n");
      timeval temp_time;
      gettimeofday(&temp_time, NULL);
      Globals.start_time_of_planning = temp_time;
      Globals.last_update_time[Globals.agent_number] = Globals.start_time_of_planning; 
      Globals.min_clock_to_plan = min_clock_to_plan;

      #ifdef pre_calculated_free_space
      get_valid_subset_freespace(all_free_space, free_space, Globals.team_bound_area_min, Globals.team_bound_area_size);
      #endif

      // find at least one solution (between all robots), also does one round of message passing per loop
      found_path = false;
      while(!found_path && (!Globals.master_reset || !Globals.found_single_robot_solution))
      {  
        // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second second case

        data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose, first_start_time);
        // broadcast tf
        //broadcast_map_tf();  
      
        last_time = clock();
        if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0)) // a planning agent
        {
          now_time = clock();
       
          vector<float> not_used;     

          //printf("planning, tree has %d points \n", Cspc.num_points);

          #ifdef treev2
          if(Cspc.BuildTreeV2(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution, false, not_used))
          #elif defined(treev3)
          if(Cspc.BuildTreeV3(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))   
          #elif defined(treev4)
          if(Cspc.BuildTreeV4(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))  
          #elif defined(treev2rho)
          if(Cspc.BuildTreeV2rho(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))   
          #elif defined(treev3rho)
          if(Cspc.BuildTreeV3rho(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))  
          #elif defined(rrt)
          if(Cspc.BuildRRT(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
          #elif defined(rrtrho)
          if(Cspc.BuildRRTRho(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
          #elif defined(rrtfast)
          if(Cspc.BuildRRTFast(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
          #else
          if(Cspc.BuildTree(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
          #endif  
          {
            found_path = true;
            MultAgSln.ExtractSolution(Cspc);
          }
        
          if(mode == 0) // share partial solutions
          {
            if(MultAgSln.GetMessages(startc, goalc))
            {
              MultAgSln.AddSolution(Cspc);  // add the path from the message to our cspace
              found_path = true;
            }
            //printf("sending (planning) messages \n");
            MultAgSln.SendMessageUDP(prob_success);
          }
        }
        else if (mode == 2 && agent_number != 0) // client agents just wait for solution
        {
          if(difftime_clock(now_time,start_time) <= min_clock_to_plan)  
            phase_two_start_t = clock();
        
          if(MultAgSln.GetMessages(startc, goalc))
          {
            MultAgSln.AddSolution(Cspc);  // add the path from the message to our cspace
            found_path = true;
          } 
        }
        now_time = clock();
      
        publish_planning_area(Scene);
        //publish_obstacles(Scene);
        publish_team_list(Globals.InTeam);   
        ros::spinOnce();
      }
      now_time = clock();
      float actual_solution_time = difftime_clock(now_time,start_time); // time for first solution
    
      printf("master: found first path in %f seconds\n", actual_solution_time);
              

      if(use_smart_plan_time)
      {
        if(actual_solution_time*plan_time_mult > smart_min_time_to_plan)
          min_clock_to_plan = actual_solution_time*plan_time_mult;
        else
          min_clock_to_plan = smart_min_time_to_plan;

        printf("master: using smart plan time : %f \n", min_clock_to_plan);
      }

      if(Globals.master_reset && Globals.found_single_robot_solution) // master reset and have a single robot solution
      {
        // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second case

        printf("master: restarting planning 1\n");
        continue;  // a team member has been added, need to restart planning with more dimensions
      }
    
      // record how much time left there is for planning (on this agent)
      Globals.planning_time_remaining[Globals.agent_number] = min_clock_to_plan - actual_solution_time;

      timeval new_temp_time;
      gettimeofday(&new_temp_time, NULL);
      Globals.last_update_time[Globals.agent_number] = new_temp_time;
  
      if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0))   // a planning agent
      {
        // do anytime until a better solution is found, or we run out of time, also does one round of message passing per loop
        

        float time_left_to_plan = Globals.calculate_time_left_for_planning();  // this also considers when other robots are expected to move
      
        float second_phase_time_total = min_clock_to_plan - actual_solution_time;

        timeval time_now;  
        gettimeofday(&time_now, NULL);

        timeval second_phase_start_time;  
        gettimeofday(&second_phase_start_time, NULL);


        printf("time left to plan start %f \n", time_left_to_plan);

     //   // we want to keep planning for the maximum of time_left_to_plan or message_wait_time
        float this_time_to_plan = message_wait_time;
     //   if(time_left_to_plan < message_wait_time)
     //     this_time_to_plan = time_left_to_plan;
     // 
        int last_time_left_floor = (int)time_left_to_plan;
//        while(time_left_to_plan > 0.0 && (!Globals.master_reset  || !Globals.found_single_robot_solution))
        while(second_phase_time_total > difftime_timeval(time_now, second_phase_start_time) && (!Globals.master_reset  || !Globals.found_single_robot_solution))
        {    
          // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second second case


          data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose, first_start_time);  
          
          if(last_time_left_floor != (int)time_left_to_plan)
          {
            // printf("\nmaster: time left to plan: %f\n", time_left_to_plan);
            printf("\nmaster: time left to plan: %f\n", second_phase_time_total - difftime_timeval(time_now, second_phase_start_time) );
            Globals.output_state_data();
            last_time_left_floor = (int)time_left_to_plan;
          }

          //printf("planning, tree has %d points \n", Cspc.num_points);

          now_time = clock();
          vector<float> not_used;     

          #ifdef treev2
          if(Cspc.BuildTreeV2(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution, false, not_used))
          #elif defined(treev3)
          if(Cspc.BuildTreeV3(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))  
          #elif defined(treev4)
          if(Cspc.BuildTreeV4(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))  
          #elif defined(treev2rho)
          if(Cspc.BuildTreeV2rho(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))   
          #elif defined(treev3rho)
          if(Cspc.BuildTreeV3rho(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))   
          #elif defined(rrt)
          if(Cspc.BuildRRT(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
          #elif defined(rrtrho)
          if(Cspc.BuildRRTRho(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
          #elif defined(rrtfast)
          if(Cspc.BuildRRTFast(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
          #else
          if(Cspc.BuildTree(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))  
          #endif
          {
            found_path = true;
            MultAgSln.ExtractSolution(Cspc);
          }
    
          if(mode == 0) // share partial solutions
          {
            if(MultAgSln.GetMessages(startc, goalc))
            {  
              MultAgSln.AddSolution(Cspc);  // add the path from the message to our cspace
              found_path = true;
            }
            MultAgSln.SendMessageUDP(prob_success);
          }

          // ROS stuff
          publish_planning_area(Scene);
          publish_obstacles(Scene);
          publish_team_list(Globals.InTeam);
          ros::spinOnce();
       
          gettimeofday(&time_now, NULL);

        //  time_left_to_plan = Globals.calculate_time_left_for_planning();  // this also considers when other robots are expected to move

        //  // we want to keep planning for the minimum of time_left_to_plan or message_wait_time
        //  this_time_to_plan = message_wait_time;
        //  if(time_left_to_plan < message_wait_time)
        //    this_time_to_plan = time_left_to_plan;
        }
      
        now_time = clock(); 
            
        if(Globals.master_reset && Globals.found_single_robot_solution)
        {
          // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second case

          printf("master: restarting planning 2\n");
          continue;  // a team member has been added, need to restart planning with more dimensions
        }
      }
 
      if(!Globals.found_single_robot_solution)
      {
        // this is the first single robot solution

        // we want to calculate ThisAgentsPath [x y angle] and Parametric_Times [time] for the controller 
        // and single_robot_solution [x y time angle] for sending to team

        vector<vector<float> > mult_agent_bst_sln_doubled;
        double_up_points(MultAgSln.BestSolution, mult_agent_bst_sln_doubled);  // double points since we need rotation at either end of each edge
        calculate_rotation(mult_agent_bst_sln_doubled);                        // calculate the rotation (of all robots in solution)

        // calculate times of each point from multi solution, this accounts for speed and rotation speed of each robot
        calculate_times(Parametric_Times, mult_agent_bst_sln_doubled, target_mps, target_rps); 

        // extract this robots path from the multi solution, each point in ThisAgentsPath will be [x y rotation]
        extract_and_translate_solution(ThisAgentsPath, mult_agent_bst_sln_doubled, Scene.translation, Globals.local_ID[agent_number], world_dims);

        printf("master: single robot solution:\n");
        vector<vector <float> > temp_single_robot_solution(ThisAgentsPath.size());
        for(uint i = 0; i < ThisAgentsPath.size(); i++)
        {
          temp_single_robot_solution[i].resize(4);
          temp_single_robot_solution[i][0] = ThisAgentsPath[i][0];
          temp_single_robot_solution[i][1] = ThisAgentsPath[i][1];
          temp_single_robot_solution[i][2] = Parametric_Times[i];
          temp_single_robot_solution[i][3] = ThisAgentsPath[i][2];
          printf("[%f %f %f %f]\n", temp_single_robot_solution[i][0], temp_single_robot_solution[i][1], temp_single_robot_solution[i][2], temp_single_robot_solution[i][3]);
        }

        printf("\n");

        Globals.single_robot_solution = temp_single_robot_solution;
        Globals.found_single_robot_solution = true;

        very_first_path_length = MultAgSln.best_solution_length;

        if(Globals.master_reset)
        {
          printf("master: restarting planning 2.5\n");
          continue;  // a team member has been added, need to restart planning with more dimensions
        }
      }

      printf("master: --- Done with actual path planning --- \n");
  




      // =========================================== consensus phase (4) ===========================================
      // ----> get consensus among the agents as to who's solution to use
      Globals.nav_state[Globals.agent_number] = 4;  
      Globals.nav_state_iteration[Globals.agent_number]++;



      // record that this agent has reach the end of path planning
      MultAgSln.FinalSolutionSent[agent_number] = 1;
  
      if(mode == 2) // just tell the clients to start moving, and server also starts moving 
        MultAgSln.moving = true;
  
      // now we try to get consensus among the agents as to who's solution to use
      if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0))
        phase_two_start_t = clock();

      while(!MultAgSln.StartMoving() && !Globals.master_reset)
      {    
        printf("master: waiting for consensus\n");
        Globals.output_state_data();
        data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose, first_start_time);

        if(mode == 2 && agent_number != 0)
        {
          usleep(sync_message_wait_time*1000000);
          break;
        }
        // while we cannot move, we broadcaset the best solution to everybody, and recieve thier best solutions
        MultAgSln.GetMessages(startc, goalc);  
        MultAgSln.SendMessageUDP(prob_success);
         
        publish_planning_area(Scene);
        // publish_obstacles(Scene);

        publish_team_list(Globals.InTeam);   
        ros::spinOnce();

        usleep(sync_message_wait_time*1000000);
        Globals.team_member_timeout(60.0);
      }
      now_time = clock();
    
      if(Globals.master_reset)
      {
        printf("master: restarting planning 3\n");
        continue;  // a team member has been added, need to restart planning with more dimensions
      }

      float phase_two_time = difftime_clock(now_time,phase_two_start_t); // time since planning ended until this robot is aware of an agreement
      //float total_time = difftime_clock(now_time,start_time);
      printf("Done with communication phase, (which took %f secs)\n", phase_two_time); 
      printf("agent #%d found the best overall solution, with length %f \n", MultAgSln.best_solution_agent, MultAgSln.best_solution_length);
  
      // save this info to a file so we can look at stats later
      //if(total_agents > 1)
      //  data_dump(experiment_name, prob_success, min_clock_to_plan, phase_two_time, Cspc, MultAgSln,actual_solution_time,total_time);
  
    
      data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose, first_start_time);
  
    }
    else // (Globals.revert_to_single_robot_path)
    {
      printf("master: using old single robot path \n");
    }


    // =========================================== move phase (5) ===========================================
    // ----> move along path
    Globals.nav_state[Globals.agent_number] = 5;  
    Globals.nav_state_iteration[Globals.agent_number]++;

    vector<bool> OldInTeam = Globals.InTeam;

    Globals.done_planning = true;
    bool need_to_calculate_path_to_broadcast = true;
    printf("masater: enter move loop\n");
    while(!display_path && !Globals.master_reset) // if we want to display the path then we ignore this part, otherwise loop here until goal or path conflict
    {       
      data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose, first_start_time);
      printf("master: moving\n");
      Globals.output_state_data();

      start_wait_t = clock();
      now_time = clock();

      if(mode == 2 && agent_number != 0) 
      {
        usleep(sync_message_wait_time*1000000);
        continue; 
      }
 
      // now extract this robot's path and calculate times 

      if(need_to_calculate_path_to_broadcast)
      {
        printf("master: calculating path to broadcast \n");

        // extract ThisAgentsPath from the multi_solution where each point is [x y rotation]

        vector<vector<float> > mult_agent_bst_sln_doubled;
        double_up_points(MultAgSln.BestSolution, mult_agent_bst_sln_doubled);  // double points since we need rotation at either end of each edge
        calculate_rotation(mult_agent_bst_sln_doubled);                        // calculate the rotation (of all robots in solution)

        // calculate times of each point from multi solution, this accounts for speed and rotation speed of each robot
        calculate_times(Parametric_Times, mult_agent_bst_sln_doubled, target_mps, target_rps); 

        // extract this robots path from the multi solution, each point in ThisAgentsPath will be [x y rotation]
        extract_and_translate_solution(ThisAgentsPath, mult_agent_bst_sln_doubled, Scene.translation, Globals.local_ID[agent_number], world_dims);

        printf("master: done extracting path from planning area\n");

        if(!Globals.use_sub_sg)
        {
          // now we need to set single_robot_solution based on the extracted and translated solution
          // reset single robot solution [x y time angle] based on the new ThisAgentsPath [x y angle] and Parametric_Times [time]
          vector<float> temp(4, -1);   
          vector<vector<float> > new_single_robot_solution(ThisAgentsPath.size(), temp);
          for(uint p = 0; p < ThisAgentsPath.size(); p++)
          {
            new_single_robot_solution[p][0] = ThisAgentsPath[p][0];  // x
            new_single_robot_solution[p][1] = ThisAgentsPath[p][1];  // y
            new_single_robot_solution[p][2] = Parametric_Times[p];   // time
            new_single_robot_solution[p][3] = ThisAgentsPath[p][2];  // angle
          }
          Globals.single_robot_solution = new_single_robot_solution;
        }
        else // if(Globals.use_sub_sg) // if we were planning in a sub area
        {
          printf("master: sandwitching new path between valid old path parts\n");

          // if this is a multi-agent solution that used sub-area selection, then need to account for getting to and from the sub area
          // assume that the time at wich the teams starts moving through the sub-ara is 0, so adjust time to be negative before entering the sub-area
          // (let controllers worry about how far behind 0 they are and how to adjust accordingly)

          // extract paths to sub-area from the old single robot solution (note points in old solution are [x y old_time angle]
          printf("master: extracting path to planning sub area from old path\n");
           
          // find the edge of the single robot path that contains the sub_start point, and the corresponding point as calculated from the path
          vector<float> path_sub_start;
          int ind_with_sub_start = find_edge_containing_point(Globals.single_robot_solution, ThisAgentsPath[0], path_sub_start); // based on [x y]

          if(ind_with_sub_start < 0)
          {
            printf("master: problems finding point at sub_start \n");
            Globals.use_sub_sg = false;
            continue;
          }

          // store portion of path to the sub area
          vector<vector<float> > single_robot_solution_to_sub_start = Globals.single_robot_solution;
          single_robot_solution_to_sub_start.resize(ind_with_sub_start + 2);

          // now we add the end of the last edge in the path to the sub area
          single_robot_solution_to_sub_start[ind_with_sub_start+1][0] = ThisAgentsPath[0][0];  // x
          single_robot_solution_to_sub_start[ind_with_sub_start+1][1] = ThisAgentsPath[0][1];  // y
          single_robot_solution_to_sub_start[ind_with_sub_start+1][3] = ThisAgentsPath[0][2];  // angle

          // calculate the time that we would have reached sub_start given old path and set the last point in the path to sub area to be that    
          float dist_of_entire_old_edge = euclid_dist(Globals.single_robot_solution[ind_with_sub_start], 
                                                      Globals.single_robot_solution[ind_with_sub_start+1]);
          float dist_of_new_edge = euclid_dist(single_robot_solution_to_sub_start[ind_with_sub_start], single_robot_solution_to_sub_start[ind_with_sub_start+1]);


          single_robot_solution_to_sub_start[ind_with_sub_start+1][2] = single_robot_solution_to_sub_start[ind_with_sub_start][2] + (dist_of_new_edge/dist_of_entire_old_edge)*(single_robot_solution_to_sub_start[ind_with_sub_start+1][2]-single_robot_solution_to_sub_start[ind_with_sub_start][2]);
     

          // fix wierd bug /// !!!!!!!!!!!!!!!!!! hard coded speed here
          float temp_dist_of_first_edge = euclid_dist(single_robot_solution_to_sub_start[0], single_robot_solution_to_sub_start[1]);
          if(single_robot_solution_to_sub_start[0][2] > single_robot_solution_to_sub_start[1][2] - temp_dist_of_first_edge/0.2)
            single_robot_solution_to_sub_start[0][2] = single_robot_solution_to_sub_start[1][2] - temp_dist_of_first_edge/0.2;


          // calculate parametric time of path to the sub area
          uint size_to_sub_start = single_robot_solution_to_sub_start.size();


          printf("master: dist and time of the thing here : \n");
          for(uint i = 0; i < size_to_sub_start; i++)
            printf("%f, %f, %f\n", single_robot_solution_to_sub_start[i][0], single_robot_solution_to_sub_start[i][1], single_robot_solution_to_sub_start[i][2]);
          printf("\n");

          // need to make the final time of the path to sub area = 0, 
          // and the rest negative but with the same delta time that they had before
 
          float final_time = single_robot_solution_to_sub_start[size_to_sub_start-1][2];

          for(uint i = 0; i < size_to_sub_start; i++)
            single_robot_solution_to_sub_start[i][2] -= final_time;

          printf("master: Done extracting path to planning sub area from old path\n");



          // extract path from sub-area to global goal from the old path

          printf("master: extracting path from planning sub area to global goal\n");
           
          // find the edge of the single robot path that contains the sub_goal point, and the corresponding point as calculated from the path
          vector<float> path_sub_goal;
          int ind_with_sub_goal = find_edge_containing_point(Globals.single_robot_solution, ThisAgentsPath[ThisAgentsPath.size()-1], path_sub_goal); // based on [x y]

          if(ind_with_sub_goal < 0)
            printf("master: problems finding point at sub_goal \n");

          // store portion of path from the sub area
          vector<vector<float> > single_robot_solution_from_sub_goal(Globals.single_robot_solution.size() - ind_with_sub_goal);  // make empty path of the correct length

          int last_ind = ThisAgentsPath.size()-1;

          // add the first point on the path from the sub area
          single_robot_solution_from_sub_goal[0].resize(4);
          single_robot_solution_from_sub_goal[0][0] = ThisAgentsPath[last_ind][0];  // x
          single_robot_solution_from_sub_goal[0][1] = ThisAgentsPath[last_ind][1];  // y
          single_robot_solution_from_sub_goal[0][2] = Parametric_Times[last_ind];   // time
          single_robot_solution_from_sub_goal[0][3] = ThisAgentsPath[last_ind][2];  // angle

          // the following loop get us from the sub area to the global goal (but times are incorrect)
          int j = 1;
          for(uint p = ind_with_sub_goal + 1; p < Globals.single_robot_solution.size(); p++, j++)
          {
            single_robot_solution_from_sub_goal[j] = Globals.single_robot_solution[p];
          }

          // calculate the time that we would have reached sub_goal given old path
          dist_of_entire_old_edge = euclid_dist(Globals.single_robot_solution[ind_with_sub_goal], 
                                                      Globals.single_robot_solution[ind_with_sub_goal+1]);
          float dist_of_lost_edge = euclid_dist(Globals.single_robot_solution[ind_with_sub_goal], ThisAgentsPath[last_ind]);

          float old_time_at_sub_goal = 0;
          if(dist_of_entire_old_edge <= SMALL) // no movement from start of edge
            old_time_at_sub_goal = Globals.single_robot_solution[ind_with_sub_goal][2];
          else                                 // movement from start of edge
            old_time_at_sub_goal = Globals.single_robot_solution[ind_with_sub_goal][2] + dist_of_lost_edge/dist_of_entire_old_edge*(Globals.single_robot_solution[ind_with_sub_goal+1][2] - Globals.single_robot_solution[ind_with_sub_goal][2]);


          // need to make the final time of the path from sub area account for both the old path time offset from the new path, 
          float time_adjust =  Parametric_Times[last_ind] /*+ sub_goal_rotate_time_adjust */ - old_time_at_sub_goal;

          uint size_from_sub_goal = single_robot_solution_from_sub_goal.size();
          for(uint i = 1; i < size_from_sub_goal; i++)
            single_robot_solution_from_sub_goal[i][2] += time_adjust;

          printf("master: Done extracting path from planning sub area to global goal\n");


          // adjust rotation to and from sub area

          if(single_robot_solution_to_sub_start.size() > 1)
          {
            single_robot_solution_to_sub_start[single_robot_solution_to_sub_start.size()-1][3] = 
              single_robot_solution_to_sub_start[single_robot_solution_to_sub_start.size()-2][3];
          }

          if(single_robot_solution_from_sub_goal.size() > 1)
          {
            single_robot_solution_from_sub_goal[0][3] = single_robot_solution_to_sub_start[1][3];
          }

          printf("master: concatonating sub paths\n");
          // now need to concatonate path to sub-area on the front of the multi_robot path and from the sub area on the back
          uint first_part_size = single_robot_solution_to_sub_start.size();
          uint sub_area_path_size = ThisAgentsPath.size();
          uint last_part_size = single_robot_solution_from_sub_goal.size();
          uint total_size = first_part_size + sub_area_path_size + last_part_size;
          //printf("path lengths: %d %d %d\n", single_robot_solution_to_sub_start.size(), ThisAgentsPath.size(), path_from_sub_goal_doubled.size());        

          vector<vector<float> > NewThisAgentsPath(total_size);         // the ThisAgentsPath version where we store [x y angle]
          vector<vector<float> > new_single_robot_solution(total_size); // the single_robot_solution version where we store [x y time angle]
          vector<float> NewParametric_Times(total_size);                // the Parametric_Times version where we store [time]

          // old path to sub area
          uint p;
          for(p = 0; p < first_part_size; p++)
          {
            new_single_robot_solution[p] = single_robot_solution_to_sub_start[p];  // [x, y, time angle]

            NewThisAgentsPath[p].resize(3);
            NewThisAgentsPath[p][0] = single_robot_solution_to_sub_start[p][0];    // x
            NewThisAgentsPath[p][1] = single_robot_solution_to_sub_start[p][1];    // y
            NewThisAgentsPath[p][2] = single_robot_solution_to_sub_start[p][3];    // angle

            NewParametric_Times[p] = single_robot_solution_to_sub_start[p][2];     // [time]
          }

          // need to adust the angle of the final point on NewThisAgentsPath and the first point on    

          // path through sub area
          for(uint q = 0; q < sub_area_path_size; q++, p++)
          {
            new_single_robot_solution[p].resize(4);
            new_single_robot_solution[p][0] = ThisAgentsPath[q][0];      // x
            new_single_robot_solution[p][1] = ThisAgentsPath[q][1];      // y
            new_single_robot_solution[p][2] = Parametric_Times[q];       // time
            new_single_robot_solution[p][3] = ThisAgentsPath[q][2];      // angle

            NewThisAgentsPath[p] = ThisAgentsPath[q];                    // [x, y, angle]

            NewParametric_Times[p] = Parametric_Times[q];                // [time]
          }
 
          // path from sub area
          for(uint q = 0; q < last_part_size; q++, p++)
          {
            new_single_robot_solution[p] = single_robot_solution_from_sub_goal[q];  // [x, y, time angle]

            NewThisAgentsPath[p].resize(3);
            NewThisAgentsPath[p][0] = single_robot_solution_from_sub_goal[q][0];    // x
            NewThisAgentsPath[p][1] = single_robot_solution_from_sub_goal[q][1];    // y
            NewThisAgentsPath[p][2] = single_robot_solution_from_sub_goal[q][3];    // angle

            NewParametric_Times[p] = single_robot_solution_from_sub_goal[q][2];     // [time]
          }

          ThisAgentsPath = NewThisAgentsPath;
          Parametric_Times = NewParametric_Times;
          Globals.single_robot_solution = new_single_robot_solution;

          printf("master: Done concatonating sub paths:\n");
          for(uint p = 0; p < Globals.single_robot_solution.size(); p++)
            printf("%f %f %f %f\n", Globals.single_robot_solution[p][0], Globals.single_robot_solution[p][1], Globals.single_robot_solution[p][2], Globals.single_robot_solution[p][3]);
          printf("\n");

         // remember the first point beyond the sub_area path as the point that we can drop these team members

         drop_point = single_robot_solution_from_sub_goal[0];
        }
        
        need_to_calculate_path_to_broadcast = false;
      }
      else // update single robot solution to reflect the robot's current location and time (shorten path to reflect distance traveled)
      {
        // find closest point on path to robot's current position
        //printf("updating path for robot position : [%f %f]\n", robot_pose->x, robot_pose->y);
        vector<float> temp_robot_position(2);
        temp_robot_position[0] = robot_pose->x;
        temp_robot_position[1] = robot_pose->y;

        vector<float> temp_best_point_found(0);

        //printf("finding edge containing the start point \n");
        int first_i_of_edge = find_edge_containing_point(Globals.single_robot_solution, temp_robot_position, temp_best_point_found);
        //printf("done finding edge containing the start point \n");
 
        // the following was taken out to reflect the rational that we should explicitly shorten Globals.single_robot_solution to reflect the starting position
        // of the robot when a new plan is made based on what was actually used
  
        if(first_i_of_edge >= 0 && Globals.single_robot_solution.size() > 1) //found the first edge and at least one edge
        {
          if(euclid_dist(temp_robot_position, temp_best_point_found) < 2*Globals.robot_radius) // robot is within 2*radius of the closest point on the path
          {     
      
            // move backward until we are not too far ahead of scheual (in case paths cross themselves)
            while(first_i_of_edge > 0 && Globals.single_robot_solution[first_i_of_edge][2] - Globals.single_robot_solution[first_i_of_edge][0] > 2.0)  // two secs hard coded here !!!!!!!!!!!!!!!!!!!!!!!! // this means we cannot be too ahead of schecual of the points we remove
            {
              first_i_of_edge --; 
            } 

            if(first_i_of_edge > 0) // we can remove all points up until now
            {
              //printf("removing edges\n");
              uint orig_size = Globals.single_robot_solution.size();
              uint new_size = orig_size - first_i_of_edge;
              vector<vector<float> > new_single_robot_solution(new_size);

              uint p = first_i_of_edge;
              for(uint q = 0; p < Globals.single_robot_solution.size(); q++, p++)
              {
                //printf("removing an edge [%f %f]\n", Globals.single_robot_solution[p][0], Globals.single_robot_solution[p][1]);
                new_single_robot_solution[q] = Globals.single_robot_solution[p];
              }

              Globals.single_robot_solution = new_single_robot_solution;
            }
            else if(first_i_of_edge == 0) // we are somewhere along the first edge
            {
              //printf("adjusting an edge\n");
              // calculate the time associated with that point
        
              float old_edge_dist = euclid_dist(Globals.single_robot_solution[0], Globals.single_robot_solution[1]);
              float new_edge_dist = euclid_dist(Globals.single_robot_solution[0], temp_robot_position);
              float time_of_old_edge = Globals.single_robot_solution[1][2] - Globals.single_robot_solution[0][2];
              //float time_of_new_edge = new_edge_dist/old_edge_dist*time_of_old_edge;
              float time_of_new_edge = new_edge_dist/0.2;  // NOTE !!!!!!!!!!!!!!!!!!!!!!!!!! 0.2 is expected robot speed (change from hard coded later)

              // calculate target angle
              float target_angle = Globals.single_robot_solution[0][4];
          
              if(Globals.single_robot_solution[0][0] != Globals.single_robot_solution[1][0] || 
                 Globals.single_robot_solution[0][1] != Globals.single_robot_solution[1][1]) // not on a rotate point
              {
                target_angle = atan2(Globals.single_robot_solution[1][1] - Globals.single_robot_solution[0][1], 
                                     Globals.single_robot_solution[1][0] - Globals.single_robot_solution[0][0]);
              }
                
              Globals.single_robot_solution[0][0] = temp_robot_position[0];                                  // x
              Globals.single_robot_solution[0][1] = temp_robot_position[1];                                  // y
              Globals.single_robot_solution[0][2] = Globals.single_robot_solution[1][2] - time_of_new_edge;  // time
              Globals.single_robot_solution[0][3] = target_angle;                                            // angle 

              //printf("modifying first edge [%f %f]\n", Globals.single_robot_solution[0][0], Globals.single_robot_solution[0][1]);
            }
          }
        }
        else
        {
          //printf("did not find the first edge \n");
        }
       

        if(Globals.single_robot_solution.size() > 0)
        {
          // update Globals.single_robot_solution time to reflect the time (from now) that we plan to be there 
          // (since this is used by other agents for collision checking)
          float delta_time_for_collision_checks = Globals.single_robot_solution[0][2];
          
          for(uint i = 0; i < Globals.single_robot_solution.size(); i++)
          {
            Globals.single_robot_solution[i][2] -= delta_time_for_collision_checks;
            //printf("---> [%f %f %f]\n", Globals.single_robot_solution[i][0], Globals.single_robot_solution[i][1], Globals.single_robot_solution[i][2]);
          }
        }


        //printf("Done updating path for robot position : [%f %f]\n", robot_pose->x, robot_pose->y);

        // check the distance to the point at which we can reset team (end of last sub area we planned for)
        bool at_drop_point = false;
        if(drop_point.size() >= 2 && temp_best_point_found.size() >= 2) // drop point exists and so does temp_best_point_found
        {
          if(euclid_dist(drop_point, temp_best_point_found) < Globals.robot_radius) // close enough point to drop team members based on path conflicts
          {
             at_drop_point = true;
             drop_point.resize(0); // resizing to 0 signals we can drop agents
          }
        }

        if(at_drop_point)
        {
          // =========================================== drop phase (6) ===========================================
          // ----> drop all members from team
          Globals.nav_state[Globals.agent_number] = 6;  
          Globals.nav_state_iteration[Globals.agent_number]++;

          for(int j = 1; j < Globals.team_size; j++) // start at 1 because this agent is 0
          {
            int j_global = Globals.global_ID[j];
      
            if(Globals.InTeam[j_global])
            {
              // make sure we can drop without just adding back in
              //if(team_drop_dist < euclid_dist(Globals.last_known_pose[j_global], Globals.last_known_pose[agent_number]))
              //{
                // drop this agent from our team
                printf("master: __________________Dropping agent %d from team__________________\n", j_global);          

                Globals.InTeam[j_global] = false;
                Globals.local_ID[j_global] = -1; 
          
                // swap local index with the last one
                Globals.global_ID[j] = Globals.global_ID[Globals.team_size-1];         
                Globals.local_ID[Globals.global_ID[j]] = j;
   
                Globals.team_size--;
                Globals.global_ID.resize(Globals.team_size);

                j--;

                if(Globals.team_size == 1)
                {
                  printf("master: only member of team is us \n");
                  Globals.planning_iteration[Globals.agent_number]++;
                }
              //}
            }
          }
          //printf("Done checking position vs. drop point");
        }
        else if(Globals.old_team_disolved(OldInTeam))
        {
          printf("master: teams disolved (resetting)\n");
        
          Globals.planning_iteration[Globals.agent_number]++;
          //Globals.revert_to_single_robot_path = true;
          Globals.use_sub_sg = false;
          Globals.master_reset = true;
          Globals.found_single_robot_solution = false;
          Globals.single_robot_solution.resize(0);
          break;
        }

      }

      publish_global_path(ThisAgentsPath, Parametric_Times); 
      robot_is_moving = true;
              
      publish_planning_area(Scene);
      publish_obstacles(Scene);
      publish_team_list(Globals.InTeam);

      ros::spinOnce(); ///////////////// error only happens when spinning
    
      MultAgSln.GetMessages(startc, goalc);
      MultAgSln.SendMessageUDP(prob_success);
 
     Globals.team_member_timeout(60.0);

      if(Globals.master_reset)
      {
        break;  // problem has been changed
      }

      usleep(sync_message_wait_time*1000000);
    }

    if(Globals.master_reset)
    {
      printf("master: restarting planning 4\n");
      continue;  // a team member has been added, need to restart planning with more dimensions
    }
    
    #ifdef using_glut
      //  Pass control to GLUT so it can interact with the user
      glutMainLoop();
    #endif     
   
    ros::spinOnce();
    //loop_rate.sleep();
    
    Globals.kill_master = true; // shutdown listener thread
  }
  
  // shutdown ros stuff
  pose_sub.shutdown();
  goal_sub.shutdown();
  global_path_pub.shutdown();
  system_update_pub.shutdown();
  planning_area_pub.shutdown();
  obstacles_pub.shutdown();
  team_list_pub.shutdown();
  
  destroy_pose(robot_pose);
  destroy_pose(goal_pose);
  
  printf("master: exiting program \n");
  return 0;
}
