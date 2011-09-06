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

using namespace std;

#define LARGE 100000000
#define SMALL 0.00000001
#define PI 3.1415926535897 
//#define using_glut

 #define treev2            //options: treev3, treev3rho, treev2, treev2rho, rrt, rrtrho, rrtfast or nothing i.e. not defined
// #define using_smoothing   // when defined the rrt algorithms perform smoothing after each new solution is found
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
//#define drop_old_robots_from_teams
        
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
  
bool JOIN_ON_OVERLAPPING_AREAS = true; // if true, then we conservatively combine teams based on overlappingplanning areas. If false, then teams are only combined if paths intersect (or cause collisions)

float team_combine_dist = 20;  // distance robots have to be near to each other to combine teams
float team_drop_dist = 3;     // distance robots have to be away from each other to dissolve teams
float team_drop_time = 10;    // after this long without hearing from a robot we drop it from the team
        
bool robot_is_moving = false;
float change_plase_thresh = .001; // if start or goal change less than this, then we say they are the same

#ifdef pre_calculated_free_space
vector<vector<float> > free_space;  // holds freespace points to resolution of bitmap map
#endif

bool use_smart_plan_time = false; // if min_planning_time is input <= 0 then this gets set to true, and min_planning_time is adjusted based on circumstance
float plan_time_mult = 3;         // if(use_smart_plan_time) then plan for at least plan_time_mult X time it takes to compute first solution 
float smart_min_time_to_plan = 5; // if(use_smart_plan_time) then plan for at least smart_min_time_to_plan

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

  Globals.master_reset = true; // goal change, so reset
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
  while(!load_map(free_space) && ros::ok())
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
        
        
        
        
// 
//     int num = free_space.size();
//   if(num > 2000)
//       num = 2000;
// 
//   multi_robot_planner_cu::PolygonArray msg;
// 
//   msg.header.frame_id = "/map_cu";
//   msg.polygons.resize(1);
// 
//   int num_points = num;
//   msg.polygons[0].points.resize(num_points);
//     
// 
//   for(int j = 0; j < num_points; j++)
//   {
//     msg.polygons[0].points[j].x = free_space[j][0];
//     msg.polygons[0].points[j].y = free_space[j][1];
//     msg.polygons[0].points[j].z = 0;        
//   }
// 
//   obstacles_pub.publish(msg); 
// 
//   
//   
  
    
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
  
  // each of the following loops is a complete planning cycle
  clock_t start_time, now_time, last_time;  
  Globals.Populate(total_agents);  // sets master_reset to true, so globals are not used as they are being changed
      
  // enter into communication with other robots to get their start and goal positions
  
  // set up globals used by communication threads
  Globals.agent_number = agent_number;
  Globals.InTeam[agent_number] = true;
  Globals.local_ID[agent_number] = 0;
  Globals.global_ID.push_back(agent_number);
  Globals.team_size = 1;  
  Globals.robot_pose = robot_pose;
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
  pthread_create( &Listener_thread, NULL, Robot_Listner_Ad_Hoc, &Globals);         // listens for incomming messages, will sleep when and as long as master_reset = true but not terminate
 
  Globals.default_map_x_size = default_map_x_size;
  Globals.default_map_y_size = default_map_y_size;

  while(!Globals.kill_master)
  {   
    // at this point master_reset is true because globals may be changing and being reset, it will only be set to false lower down in this loop

    // if using smart plan time then reset min time to plan
    printf("-----------------------start of planning phase -----------------------\n");
 
    if(use_smart_plan_time)
    {
      min_clock_to_plan = smart_min_time_to_plan;
      printf("using smart plan time : %f \n", min_clock_to_plan);
    }

    Globals.done_planning = false;
    if(robot_is_moving) // master_reset = true while robot is moving
    {
      unused_result = system(system_call);  // remove old files
      start_time = clock();
      now_time = clock(); 
      while(difftime_clock(now_time,start_time) < 2.0) // wait for two seconds so that robot can stop moving
      {
        publish_system_update(1); // if we've reset, then tell the controller
        ros::spinOnce();  
        now_time = clock(); 
      }
    }  
    robot_is_moving = false;  
    
    Globals.planning_iteration[Globals.agent_number]++;
    printf("increasing planning_iteration_single_solutions of me \n");
    Globals.planning_iteration_single_solutions[Globals.agent_number] = Globals.planning_iteration[Globals.agent_number];        

    Globals.have_info.resize(0);
    Globals.have_info.resize(Globals.number_of_agents, 0);   // gets set to 1 when we get an agent's info
        
    Globals.agent_ready.resize(0); 
    Globals.agent_ready.resize(Globals.number_of_agents, 0); // gets set to 1 when we get an agent's info
    
    Globals.have_info[0] = 1;
    Globals.agent_ready[0] = 1;
          
    Globals.last_update_time.resize(0);
    timeval temp_time;
    gettimeofday(&temp_time, NULL);
    Globals.last_update_time.resize(Globals.number_of_agents, temp_time);

    printf("resetting planning start time\n");
    Globals.start_time_of_planning = temp_time;
    Globals.min_clock_to_plan = min_clock_to_plan;

    Globals.planning_time_remaining.resize(0);
    Globals.planning_time_remaining.resize(Globals.number_of_agents, LARGE);
    
    #ifdef drop_old_robots_from_teams
    // get rid of outdated team members
    for(int j = 1; j < Globals.team_size; j++) // start at 1 because this agent is 0
    {
      int j_global = Globals.global_ID[j];
      
      if(difftime_clock(now_time, Globals.last_known_time[j_global]) > Globals.drop_time || Globals.last_known_dist[j_global] > Globals.drop_dist)
      {
        // either the drop distance or time has been reached, so drop this agent from our team
          
        Globals.InTeam[j_global] = false;
        Globals.local_ID[j_global] = -1; 
          
        // swap local index with the last one
        Globals.global_ID[j] = Globals.global_ID[Globals.team_size-1]    ;         
        Globals.local_ID[Globals.global_ID[j]] = j;
   
        Globals.team_size--;
        Globals.global_ID.resize(Globals.team_size);
      }
    }
    #endif
    
    Globals.MAgSln = NULL;
    Globals.not_planning_yet = true;  
    Globals.master_reset = false;  // now set master_reset to false indicating that "perminate" robot data in globals is stable
                                   // this should be the only place in the code where master_reset is set to false
      
    Globals.have_calculated_start_and_goal = false;

    // kick off sender threads
    pthread_create( &Sender_thread, NULL, Robot_Data_Sync_Sender_Ad_Hoc, &Globals);  // this is used for startup, to send data to other robots, will terminate on master_reset

    // reset start and goal data
    Globals.start_coords.resize(0);
    Globals.start_coords.resize(Globals.number_of_agents);

    Globals.goal_coords.resize(0);
    Globals.goal_coords.resize(Globals.number_of_agents);

    // need to calculate own start and goals based on limited sub-region if there are other robots in team
    if(Globals.team_size > 1)
    {
      // do message passing (in sender thread) until we have everybody's updated prefered single robot paths
      while(!Globals.have_all_team_single_paths() && !Globals.master_reset)
      {
        printf("waiting for other member's single paths\n");
        sleep(1);
      }
      if(Globals.master_reset)
      {
        printf("restarting planning -1\n");
        continue;
      }

      // look at this agent's single path vs other agent's single paths and 
      // calculate start and goal for this agent based on intersections with other agents

      vector<float> sub_start;
      vector<float> sub_goal;
      Globals.other_robots_single_solutions[Globals.agent_number] = Globals.single_robot_solution;
      float preferred_min_planning_area_side_length = 1;
      float preferred_max_planning_area_side_length = 1.5;
      float accuracy_resolution = .05;
      float time_resolution = .05;

      printf("calculating sub_start and sub_goals \n");

      bool no_conflicts_between_sub_paths = false;
      if(calculate_sub_goal(Globals.other_robots_single_solutions, Globals.InTeam, Globals.agent_number, 
                            preferred_min_planning_area_side_length,  preferred_max_planning_area_side_length, 
                            robot_radius, accuracy_resolution, time_resolution, sub_start, sub_goal, no_conflicts_between_sub_paths) )
      {
        // if here then sub_start and sub_goal have changed
        printf("new sub area \n");

        Globals.start_coords[0].resize(3, 0); 
        Globals.start_coords[0][0] = sub_start[0];
        Globals.start_coords[0][1] = sub_start[1];
        Globals.start_coords[0][2] = 0;

        Globals.goal_coords[0].resize(3,0); 
        Globals.goal_coords[0][0] = sub_goal[0];
        Globals.goal_coords[0][1] = sub_goal[1];
        Globals.goal_coords[0][2] = 0;

        Globals.use_sub_sg = true;
      }
      else if(no_conflicts_between_sub_paths)
      {
        printf("no conflicts between sub-paths\n");

        // reset single robot path 
        Globals.use_sub_sg = false;
        Globals.revert_to_single_robot_path = true;
      }
      else if(Globals.use_sub_sg)
      {
        printf("old sub area \n");

        // continue using old sub_start and sub_goal (which we have saved)
        Globals.start_coords[0].resize(3, 0); 
        Globals.start_coords[0][0] = sub_start[0];
        Globals.start_coords[0][1] = sub_start[1];
        Globals.start_coords[0][2] = 0;

        Globals.goal_coords[0].resize(3,0); 
        Globals.goal_coords[0][0] = sub_goal[0];
        Globals.goal_coords[0][1] = sub_goal[1];
        Globals.goal_coords[0][2] = 0;
      }

      printf("Done calculating sub_start and sub_goals \n");

      // NOTE adjust bounds to take care of min planning region size based on huristic when we actually calculate min bounds and size
      // NOTE remember to change single robot path to reflect group solution once a group solution is found
      // NOTE also need to get this data to the controller (path-follower)
    }

    if(!Globals.use_sub_sg) // are not using sub area
    {
      printf("default sub area \n");

      Globals.goal_coords[0].resize(3,0); 
      Globals.goal_coords[0][0] = goal_pose->x;
      Globals.goal_coords[0][1] = goal_pose->y;
      Globals.goal_coords[0][2] = goal_pose->alpha;

      Globals.start_coords[0].resize(3,0); 
      Globals.start_coords[0][0] = robot_pose->x;
      Globals.start_coords[0][1] = robot_pose->y;
      Globals.start_coords[0][2] = robot_pose->alpha;
    }

    printf("My IP: %s\n",Globals.my_IP);
    printf("My start: %f %f %f\n", Globals.start_coords[0][0], Globals.start_coords[0][1], Globals.start_coords[0][2]);
    printf("My goal: %f %f %f\n", Globals.goal_coords[0][0], Globals.goal_coords[0][1], Globals.goal_coords[0][2]);

    printf("---------- all planning iterations: ");
    for(uint i = 0; i < Globals.planning_iteration.size(); i++)
      printf("%d, ", Globals.planning_iteration[i]);
    printf("----------\n ");
    
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
      while(Globals.not_planning_yet && !Globals.master_reset)
      {
        // wait until we have everybody in this team's address info
        printf("planner thread not planning yet\n");
    
        // broadcast tf
        //broadcast_map_tf();
    
        usleep(Globals.sync_message_wait_time*1000000);
 
      }
  
      if(Globals.master_reset)
      {
        printf("restarting planning 0\n");
        continue;  // a team member has been added, need to restart planning with more dimensions
      }
    
      printf("--- start of actual planning ---\n");
  
      // this is where the planning stuff starts
      // set up the scene based on all start/goal locations and any maps
      if(!Scene.LoadFromGlobals(Globals))
        return 0;
    
      char map_file[] = "../lab.txt";
      if(!Scene.LoadMapFromFile(map_file))
      {
        printf("unable to load map file: %s\n", map_file);
        return 0;
      }
  
      Scene.PrintSceneInfo(); 
      publish_planning_area(Scene);
      publish_obstacles(Scene);
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
        Globals.master_reset = true;

        printf("!!!!!!!!!!!!! INVALID START OR GOAL !!!!!!!!!!!!!!\n");
        sleep(1);
        continue;
      }
    
      MultAgSln.Populate(total_agents, agent_number, &Globals, world_dims);
      MultAgSln.obstacles_pub = &obstacles_pub;
      Globals.MAgSln = &MultAgSln;

      int iterations_left = -1; // negative means that time is used instead
  
      clock_t phase_two_start_t = clock();
    
      printf("resetting planning start time\n");
      gettimeofday(&temp_time, NULL);
      Globals.start_time_of_planning = temp_time;
      Globals.last_update_time[Globals.agent_number] = Globals.start_time_of_planning; 
      Globals.min_clock_to_plan = min_clock_to_plan;


      // find at least one solution (between all robots), also does one round of message passing per loop
      found_path = false;
      while(!found_path && (!Globals.master_reset || !Globals.found_single_robot_solution))
      {  
        // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second second case



        //data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose);
        // broadcast tf
        //broadcast_map_tf();  
      
        last_time = clock();
        if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0)) // a planning agent
        {
          now_time = clock();
        
          if(Globals.master_reset)
            printf("pulse num_ag=%d  reset:  ", Globals.number_of_agents);
          else
            printf("pulse num_ag=%d  -----:  ", Globals.number_of_agents);      

          for(int a = 0; a < Globals.number_of_agents ; a++)
          {
            if(!Globals.InTeam[a])
              printf("--- ");
            else
            {
              printf("%d(%d,%d) ", a, Globals.have_info[Globals.local_ID[a]], Globals.agent_ready[Globals.local_ID[a]]);
            } 
          }
          printf(" \n");
        
          #ifdef treev2
          if(Cspc.BuildTreeV2(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
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
           
        ros::spinOnce();
      }
      float actual_solution_time = difftime_clock(now_time,start_time); // time for first solution
    
      printf("found first path \n");
              

      if(use_smart_plan_time)
      {
        if(actual_solution_time*plan_time_mult > smart_min_time_to_plan)
          min_clock_to_plan = actual_solution_time*plan_time_mult;
        printf("using smart plan time : %f \n", min_clock_to_plan);
      }

      if(Globals.master_reset && Globals.found_single_robot_solution) // master reset and have a single robot solution
      {
        // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second case

        printf("restarting planning 1\n");
        continue;  // a team member has been added, need to restart planning with more dimensions
      }
    
      // record how much time left there is for planning (on this agent)
      Globals.planning_time_remaining[agent_number] = min_clock_to_plan - actual_solution_time;
      timeval temp_time;
      gettimeofday(&temp_time, NULL);
      Globals.last_update_time[agent_number] = temp_time;
  
      if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0))   // a planning agent
      {
        // do anytime until a better solution is found, or we run out of time, also does one round of message passing per loop
        
        float time_left_to_plan = Globals.calculate_time_left_for_planning();  // this also considers when other robots are expected to move
      
        // we want to keep planning for the minimum of time_left_to_plan or message_wait_time
        float this_time_to_plan = message_wait_time;
        if(time_left_to_plan < message_wait_time)
          this_time_to_plan = time_left_to_plan;
      
        int last_time_left_floor = (int)time_left_to_plan;
        while(this_time_to_plan > 0 && (!Globals.master_reset  || !Globals.found_single_robot_solution))
        {    
          // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second second case
 
 

          //data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose);  
          
          if(last_time_left_floor != (int)time_left_to_plan)
          {
            printf("\ntime left to plan: %f\n", time_left_to_plan);
            last_time_left_floor = (int)time_left_to_plan;
        
            //for(int k = 0; k < Globals.number_of_agents; k++)
            //  printf("agent %d: %f \n", k, Globals.planning_time_remaining[k]);
            //printf("\n");
          }
        
          now_time = clock();
     
          #ifdef treev2
          if(Cspc.BuildTreeV2(now_time, this_time_to_plan, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
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
          ros::spinOnce();
      
      
          time_left_to_plan = Globals.calculate_time_left_for_planning();  // this also considers when other robots are expected to move
      
          // we want to keep planning for the minimum of time_left_to_plan or message_wait_time
          this_time_to_plan = message_wait_time;
          if(time_left_to_plan < message_wait_time)
            this_time_to_plan = time_left_to_plan;
        }
      
        now_time = clock(); 
            
        if(Globals.master_reset && Globals.found_single_robot_solution)
        {
          // Note: want a good solution for single robot, so force to find one using all planning time before allow reset by second case

 
          printf("restarting planning 2\n");
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

        printf("single robot solution:\n");
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

 

        if(Globals.master_reset)
        {
          printf("restarting planning 2.5\n");
          continue;  // a team member has been added, need to restart planning with more dimensions
        }
      }

      printf("--- Done with actual path planning --- \n");
  
      // record that this agent has reach the end of path planning
      MultAgSln.FinalSolutionSent[agent_number] = 1;
  
      if(mode == 2) // just tell the clients to start moving, and server also starts moving 
        MultAgSln.moving = true;
  
      // now we try to get consensus among the agents as to who's solution to use
      if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0))
        phase_two_start_t = clock();

      while(!MultAgSln.StartMoving() && !Globals.master_reset)
      {    
          
        data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose);

        usleep(sync_message_wait_time*1000000);

        if(mode == 2 && agent_number != 0)
          break;
    
        // while we cannot move, we broadcaset the best solution to everybody, and recieve thier best solutions
        MultAgSln.GetMessages(startc, goalc);  
        MultAgSln.SendMessageUDP(prob_success);
   
        printf(" waiting, not moving\n");
    
      
        printf("final solution sent:  -->");
        for(uint i = 0; i < MultAgSln.FinalSolutionSent.size(); i++)
        {
          if(Globals.InTeam[i])
            printf("%d, ", MultAgSln.FinalSolutionSent[i]);
          else
            printf("-, ");
        }
        printf("  <---\n");
      
      
      
        publish_planning_area(Scene);
       // publish_obstacles(Scene);
            
        ros::spinOnce();
      }
      now_time = clock();
    
      if(Globals.master_reset)
      {
        printf("restarting planning 3\n");
        continue;  // a team member has been added, need to restart planning with more dimensions
      }

      float phase_two_time = difftime_clock(now_time,phase_two_start_t); // time since planning ended until this robot is aware of an agreement
      //float total_time = difftime_clock(now_time,start_time);
      printf("Done with communication phase, (which took %f secs)\n", phase_two_time); 
      printf("agent #%d found the best overall solution, with length %f \n", MultAgSln.best_solution_agent, MultAgSln.best_solution_length);
  
      // save this info to a file so we can look at stats later
      //if(total_agents > 1)
      //  data_dump(experiment_name, prob_success, min_clock_to_plan, phase_two_time, Cspc, MultAgSln,actual_solution_time,total_time);
  
    
      //data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose);
    
      // now we just broadcast the final solution, in case other robots didn't get it
  
//       printf("ending clean \n");
//       Globals.kill_master = true; // shutdown listener thread
//       sleep(1);
//       return 0;
    }
    else // (Globals.revert_to_single_robot_path)
    {
      printf("using old single robot path \n");
    }

    Globals.done_planning = true;
    bool need_to_calculate_path_to_broadcast = true;
    while(!display_path && !Globals.master_reset) // if we want to display the path then we ignore this part, otherwise loop here until goal or path conflict
    {       
      //printf("%f %f %f \n", lookup_sum/n_lookup, out_collision/n_collision, out_sum/n_out );
      //printf("%f %f %f \n", elookup_sum/en_lookup, eout_collision/en_collision, eout_sum/en_out );
   
      //printf("----------------------\n");
  
      //data_dump_dynamic_team(experiment_name, Cspc, MultAgSln, Globals, robot_pose);
      
      start_wait_t = clock();
      now_time = clock();

      usleep(sync_message_wait_time*1000000);

      if(mode == 2 && agent_number != 0) 
        continue; 

      MultAgSln.GetMessages(startc, goalc);
    
      MultAgSln.SendMessageUDP(prob_success);

      
      // now extract this robot's path and calculate times 

      if(Globals.revert_to_single_robot_path) // reverting to old path
      {
        // do nothing

        Globals.revert_to_single_robot_path = false;
        need_to_calculate_path_to_broadcast = false;
      }
      else if(need_to_calculate_path_to_broadcast)
      {
        printf("calculating path to broadcast \n");

        // extract ThisAgentsPath from the multi_solution where each point is [x y rotation]

        vector<vector<float> > mult_agent_bst_sln_doubled;
        double_up_points(MultAgSln.BestSolution, mult_agent_bst_sln_doubled);  // double points since we need rotation at either end of each edge
        calculate_rotation(mult_agent_bst_sln_doubled);                        // calculate the rotation (of all robots in solution)

        // calculate times of each point from multi solution, this accounts for speed and rotation speed of each robot
        calculate_times(Parametric_Times, mult_agent_bst_sln_doubled, target_mps, target_rps); 

        // extract this robots path from the multi solution, each point in ThisAgentsPath will be [x y rotation]
        extract_and_translate_solution(ThisAgentsPath, mult_agent_bst_sln_doubled, Scene.translation, Globals.local_ID[agent_number], world_dims);

        printf("done extracting path from planning area\n");

        if(Globals.use_sub_sg) // if we were planning in a sub area
        {
          printf("sandwitching new path between valid old path parts\n");

          // if this is a multi-agent solution that used sub-area selection, then need to account for getting to and from the sub area
          // assume that the time at wich the teams starts moving through the sub-ara is 0, so adjust time to be negative before entering the sub-area

          // extract paths to sub-area from the old single robot solution (note points in old solution are [x y old_time angle]
          printf("extracting path to planning sub area from old path\n");
           
          // find the edge of the single robot path that contains the sub_start point, and the corresponding point as calculated from the path
          vector<float> path_sub_start;
          int ind_with_sub_start = find_edge_containing_point(Globals.single_robot_solution, ThisAgentsPath[0], path_sub_start); // based on [x y]

          if(ind_with_sub_start < 0)
            printf("problems finding point at sub_start \n");

          // store portion of path to the sub area
          vector<vector<float> > single_robot_solution_to_sub_start(ind_with_sub_start+2);  // make empty path of the correct length

          // the following loop get us up to the start of the last edge in the path to the sub area (but times are incorrect)
          for(int p = 0; p <= ind_with_sub_start; p++)
          {
            single_robot_solution_to_sub_start[p] = Globals.single_robot_solution[p];
          }
  
          // now we add the end of the last edge in the path to the sub area
          single_robot_solution_to_sub_start[ind_with_sub_start+1].resize(4);
          single_robot_solution_to_sub_start[ind_with_sub_start+1][0] = ThisAgentsPath[0][0];  // x
          single_robot_solution_to_sub_start[ind_with_sub_start+1][1] = ThisAgentsPath[0][1];  // y
          single_robot_solution_to_sub_start[ind_with_sub_start+1][3] = ThisAgentsPath[0][2];  // angle

          // calculate the time that we would have reached sub_start given old path and set the last point in the path to sub area to be that    
          float dist_of_entire_old_edge = euclid_dist(Globals.single_robot_solution[ind_with_sub_start], 
                                                      Globals.single_robot_solution[ind_with_sub_start+1]);
          float dist_of_new_edge = euclid_dist(Globals.single_robot_solution[ind_with_sub_start], ThisAgentsPath[0]);

          if(dist_of_entire_old_edge <= SMALL) // no movement from start of edge
            single_robot_solution_to_sub_start[ind_with_sub_start+1][2] = Globals.single_robot_solution[ind_with_sub_start][2];
          else                                 // movement from start of edge
            single_robot_solution_to_sub_start[ind_with_sub_start+1][2] = Globals.single_robot_solution[ind_with_sub_start][2] + dist_of_new_edge/dist_of_entire_old_edge*(Globals.single_robot_solution[ind_with_sub_start+1][2] - Globals.single_robot_solution[ind_with_sub_start][2]);

          // calculate parametric time of path to the sub area
          uint size_to_sub_start = single_robot_solution_to_sub_start.size();

          // need to make the final time of the path to sub area = -sub_start_rotate_time_adjust, 
          // and the rest negative but with the same delta time that they had before
 
          float final_time = single_robot_solution_to_sub_start[size_to_sub_start-1][2]; // + sub_start_rotate_time_adjust;

          for(uint i = 0; i < size_to_sub_start; i++)
            single_robot_solution_to_sub_start[i][2] -= final_time;

          printf("Done extracting path to planning sub area from old path\n");



          // extract path from sub-area to global goal from the old path

          printf("extracting path from planning sub area to global goal\n");
           
          // find the edge of the single robot path that contains the sub_goal point, and the corresponding point as calculated from the path
          vector<float> path_sub_goal;
          int ind_with_sub_goal = find_edge_containing_point(Globals.single_robot_solution, ThisAgentsPath[ThisAgentsPath.size()-1], path_sub_goal); // based on [x y]

          if(ind_with_sub_goal < 0)
            printf("problems finding point at sub_goal \n");

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

          printf("Done extracting path from planning sub area to global goal\n");


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

          printf("concatonating sub paths\n");
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

          printf("Done concatonating sub paths\n");
        }
        else // are not using a sub area
        {

          // reset single robot solution [x y time angle] based on the new ThisAgentsPath [x y angle] and Parametric_Times [time]
          vector<float> temp(4, -1);   
          vector<vector<float> > new_single_robot_solution(ThisAgentsPath.size(), temp);
          for(uint p = 1; p < ThisAgentsPath.size(); p++)
          {
            new_single_robot_solution[p][0] = ThisAgentsPath[p][0];  // x
            new_single_robot_solution[p][1] = ThisAgentsPath[p][1];  // y
            new_single_robot_solution[p][2] = Parametric_Times[p];   // time
            new_single_robot_solution[p][3] = ThisAgentsPath[p][2];  // angle
          }
          Globals.single_robot_solution = new_single_robot_solution;
        }
        need_to_calculate_path_to_broadcast = false;
      }
      else // update single robot solution to reflect the robot's current location (shorten path to reflect distance traveled
      {

        // find closest point on path to robot's current position
        //printf("robot position : [%f %f]\n", robot_pose->x, robot_pose->y);
        vector<float> temp_robot_position(2);
        temp_robot_position[0] = robot_pose->x;
        temp_robot_position[1] = robot_pose->y;

        vector<float> temp_best_point_found;


        int first_i_of_edge = find_edge_containing_point(Globals.single_robot_solution, temp_robot_position, temp_best_point_found);
        if(first_i_of_edge >= 0) //found an edge
        {
          if(euclid_dist(temp_robot_position, temp_best_point_found) < Globals.robot_radius) // robot is within radius of the closest point on the path
          {      
            if(first_i_of_edge == 0) // just repace first point in path with temp_best_point_found
            {
              printf("adjusting an edge\n");
              // calculate the time associated with that point
              if(Globals.single_robot_solution.size() > 1) // ... if there is at least one edge
              {
                float old_edge_dist = euclid_dist(Globals.single_robot_solution[0], Globals.single_robot_solution[1]);
                float dist_moved_since_last = euclid_dist(Globals.single_robot_solution[0], temp_best_point_found);
                if(dist_moved_since_last > 0)
                {
                  float diff_time = dist_moved_since_last/old_edge_dist * (Globals.single_robot_solution[1][2] - Globals.single_robot_solution[0][2]);
                  Globals.single_robot_solution[0][0] = temp_best_point_found[0];               // x
                  Globals.single_robot_solution[0][1] = temp_best_point_found[1];               // y
                  Globals.single_robot_solution[0][2] += diff_time;                             // time
                  //Globals.single_robot_solution[0][4] = Globals.single_robot_solution[0][4];  // angle remains unchanged
                }
              }
            }
            else // need to remove at least one edge (take care of removing more of the next edge on next loop)
            {
              printf("removing an edge\n");
              uint orig_size = Globals.single_robot_solution.size();
              uint new_size = orig_size - first_i_of_edge;
              vector<vector<float> > new_single_robot_solution(new_size);

              uint p = first_i_of_edge;
              for(uint q = 0; p < Globals.single_robot_solution.size(); q++, p++)
              {
                new_single_robot_solution[q] = Globals.single_robot_solution[p];
              }

              Globals.single_robot_solution = new_single_robot_solution;
            }
          }
        }

        for(uint p = 0; p < Globals.single_robot_solution.size(); p++)
          printf("%f %f %f %f\n", Globals.single_robot_solution[p][0], Globals.single_robot_solution[p][1], Globals.single_robot_solution[p][2], Globals.single_robot_solution[p][3]);
        printf("\n");
      }

      publish_global_path(ThisAgentsPath, Parametric_Times); 
      robot_is_moving = true;
              
      publish_planning_area(Scene);
      publish_obstacles(Scene);

      ros::spinOnce(); ///////////////// error only happens when spinning
    
      //printf("sleeping \n");
      loop_rate.sleep();
      // printf("moving\n");

      if(Globals.master_reset)
      {
        break;  // problem has been changed
      }
    }

    if(Globals.master_reset)
    {
      printf("restarting planning 4\n");
      continue;  // a team member has been added, need to restart planning with more dimensions
    }
    
    #ifdef using_glut
      //  Pass control to GLUT so it can interact with the user
      glutMainLoop();
    #endif     
   
    ros::spinOnce();
    loop_rate.sleep();
    
    Globals.kill_master = true; // shutdown listener thread
  }
  
  // shutdown ros stuff
  pose_sub.shutdown();
  goal_sub.shutdown();
  global_path_pub.shutdown();
  system_update_pub.shutdown();
  planning_area_pub.shutdown();
  obstacles_pub.shutdown();
  
  destroy_pose(robot_pose);
  destroy_pose(goal_pose);
  
  printf("exiting program \n");
  return 0;
}
