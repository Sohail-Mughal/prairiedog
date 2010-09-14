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

using namespace std;

#define LARGE 100000000
#define SMALL 0.00000001
#define PI 3.1415926535897 
//#define using_glut

 #define treev2            //options: treev3, treev3rho, treev2, treev2rho, rrt, rrtrho, rrtfast or nothing i.e. not defined
// #define using_smoothing   // when defined the rrt algorithms perform smoothing after each new solution is found
// #define save_time_data    // when defined stats [time, best, path_length, nodes_in_tree] are saved (for entire run) after the final solution is found
// #define use_kd_tree       // when defined a kd_tree is used for finding nearest neighbors (only useful with trees where this is allowed)  rrtrho, rrtfast


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
 
int min_num_agents = -1; // waits until this many agents have contact before planning (this number includes this agent), -1 means wait for all

float map_resolution = -1; // reset later to proper value
float map_width = 0;
float map_height = 0;
        
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
GlobalVariables Globals;
vector<vector<float> > ThisAgentsPath; // holds this agents path of the best solution
vector<float> Parametric_Times; // holds time parametry of best solution
        
#include "helper_functions.cpp"
#include "NavScene.cpp"
#include "MultiRobotWorkspace.cpp"  
#include "Workspace.cpp"
#include "Cspace.cpp"
#include "MultiAgentSolution.cpp"
#include "MultiRobotComs.cpp" 
#include "glut_functions.cpp"

struct POSE;
typedef struct POSE POSE;

struct UPDATE;
typedef struct UPDATE UPDATE;


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
        
// globals for robot and goal
POSE* robot_pose = NULL;
POSE* goal_pose = NULL;


bool new_goal = false;       
bool change_token_used = false;
bool reload_map = false; 


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
        

/*--------------------------- ROS callbacks -----------------------------*/
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
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

  change_token_used = false;
}


/*---------------------- ROS Publisher Functions ------------------------*/
void publish_global_path(vector<vector<float> > path, vector<float> times_p)
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


/*----------------------- ROS service functions -------------------------*/

bool load_pose()
{
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
   
  ros::init(argc, argv, "base_planner_cu");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
    
  // wait until the map service is provided (we need its tf /world_cu -> /map_cu to be broadcast)
  //if(want_clean_start)
  //  ros::service::waitForService("/cu/get_map_cu", -1);
  
  // set up ROS topic subscriber callbacks
  pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
  goal_sub = nh.subscribe("/cu/goal_cu", 1, goal_callback);
    
  // set up ROS topic publishers
  global_path_pub = nh.advertise<nav_msgs::Path>("/cu/global_path_cu", 1);
  system_update_pub = nh.advertise<std_msgs::Int32>("/cu/system_update_cu", 10);
  
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
  
  printf("I am agent #%d of %d, and can plan for %f seconds\nmessages sent with probability %f\nmessage_wait_time: %f, sync_message_wait_time: %f\n", agent_number, total_agents, min_clock_to_plan, prob_success, message_wait_time, sync_message_wait_time);
  
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
      if(goal_pose == NULL)
      {
        printf("waiting for goal\n");
      
        //load_goal();
        if(goal_pose != NULL)
        {
          printf("recieved goal: \n");
          print_pose(goal_pose);
        }
      }
      
      if(robot_pose == NULL)
      {
        printf("waiting for pose\n"); 
      
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
  //print_pose(robot_pose);
  //print_pose(goal_pose);
  
  // spin ros one more time to check callbacks
  if(ros::ok())
  {
    new_goal = false; 
    ros::spinOnce();  // check in on callbacks
  }
  
  // remember the start time
  clock_t start_time, now_time, last_time;  
  start_time = clock();
  now_time = clock();
  last_chop_t = clock();
     
  // enter into communication with other robots to get their start and goal positions
  
  // set up globals used by communication threads
  Globals.Populate(total_agents);
  Globals.agent_number = agent_number;
  if(min_num_agents < 0)
    Globals.min_number_of_agents = total_agents;
  else
    Globals.min_number_of_agents = min_num_agents;

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
  
  if(Globals.start_coords[agent_number].size() < 3)
     Globals.start_coords[agent_number].resize(3); 
  Globals.start_coords[agent_number][0] = robot_pose->x;
  Globals.start_coords[agent_number][1] = robot_pose->y;
  Globals.start_coords[agent_number][2] = robot_pose->alpha;
  
  if(Globals.goal_coords[agent_number].size() < 3)
     Globals.goal_coords[agent_number].resize(3); 
  Globals.goal_coords[agent_number][0] = goal_pose->x;
  Globals.goal_coords[agent_number][1] = goal_pose->y;
  Globals.goal_coords[agent_number][2] = goal_pose->alpha;
  
  Globals.have_info[agent_number] = 1;
  Globals.agent_ready[agent_number] = 1;
          
  Globals.robot_radius = robot_radius;
  Globals.prob_at_goal = prob_at_goal;
  Globals.move_max = move_max;
  Globals.theta_max = theta_max;
  Globals.resolution = resolution;
  Globals.angular_resolution = angular_resolution;         
  Globals.planning_border_width = planning_border_width;
  
  printf("My IP: %s\n",Globals.my_IP);
  printf("My start: %f %f %f\n", Globals.start_coords[agent_number][0], Globals.start_coords[agent_number][1], Globals.start_coords[agent_number][2]);
  printf("My goal: %f %f %f\n", Globals.goal_coords[agent_number][0], Globals.goal_coords[agent_number][1], Globals.goal_coords[agent_number][2]);
  
  // kick off communication threads
  pthread_t Listener_thread, Sender_thread;
  pthread_create( &Listener_thread, NULL, Robot_Listner_Ad_Hoc, &Globals); // gets incomming messages, is also used in startup
  pthread_create( &Sender_thread, NULL, Robot_Sender_Ad_Hoc, &Globals);    // this is used for startup
  
  // start-up phase loop (wait until we have min number of agents start and goal locations)
  clock_t start_wait_t;
  while(Globals.non_planning_yet)
  {
    // wait until we have everybody's address info
    printf("planner thread not planning yet\n");
    
    // broadcast tf
    //broadcast_map_tf();
    
    start_wait_t = clock();
    now_time = clock();
    while(difftime_clock(now_time, start_wait_t) < Globals.sync_message_wait_time)
      now_time = clock(); 
  }
  
  printf("here in main loop \n");
  
  // this is where the planning stuff starts
  // set up the scene based on all start/goal locations and any maps
  if(!Scene.LoadFromGlobals(Globals))  ////////////////////////////////////////still need to ad functionality that reads in map file (and correct sub-piece of it)
    return 0;
  Scene.PrintScenceInfo(); 
  
  int num_robots = Scene.num_robots;
  int world_dims = Scene.world_dims;
  float robot_rad = Scene.robot_rad[0];      
  map_resolution = Scene.resolution*num_robots; //////////////////// this is kind of wierd, someday it may be a good idea to go down the rabbit hole and figure out if this actually affects anything, or if it is now ignorred by stuff
  startc = Scene.startC;
  goalc = Scene.goalC;
  dist_threshold = map_resolution;
  Cspc.W.Populate(num_robots, robot_rad, Scene.dim_max);
  //Cspc.W.dim_max = Scene.dim_max;
  Cspc.Populate(startc, goalc, num_robots*world_dims);
    
  MultAgSln.Populate(total_agents, agent_number, &Globals);
  
  int iterations_left = -1; // negative means that time is used instead
  
  clock_t phase_two_start_t = clock();
    
  // find at least one solution (between all robots), also does one round of message passing per loop
  while(!found_path)
  {  
    // broadcast tf
    //broadcast_map_tf();  
      
    last_time = clock();
    if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0)) // a planning agent
    {
      #ifdef treev2
      if(Cspc.BuildTreeV2(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
      #elif defined(treev3)
      if(Cspc.BuildTreeV3(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))   
      #elif defined(treev4)
      if(Cspc.BuildTreeV4(now_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))  
      #elif defined(treev2rho)
      if(Cspc.BuildTreeV2rho(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))   
      #elif defined(treev3rho)
      if(Cspc.BuildTreeV3rho(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))  
      #elif defined(rrt)
      if(Cspc.BuildRRT(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
      #elif defined(rrtrho)
      if(Cspc.BuildRRTRho(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
      #elif defined(rrtfast)
      if(Cspc.BuildRRTFast(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
      #else
      if(Cspc.BuildTree(last_time, message_wait_time, iterations_left, prob_at_goal, move_max, theta_max, resolution, angular_resolution))
      #endif  
      {
        found_path = true;
        MultAgSln.ExtractSolution(Cspc);
      }
      
      if(mode == 0) // share partial solutions
      {
        if(MultAgSln.GetMessages())
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
      
      if(MultAgSln.GetMessages())
      {
        MultAgSln.AddSolution(Cspc);  // add the path from the message to our cspace
        found_path = true;
      } 
    }
    now_time = clock();
  }
  float actual_solution_time = difftime_clock(now_time,start_time); // time for first solution
  
  if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0))   // a planning agent
  {
    // do anytime until a better solution is found, or we run out of time, also does one round of message passing per loop
    now_time = clock();
    while(difftime_clock(now_time,start_time) < min_clock_to_plan)
    {   
      // broadcast tf
      //broadcast_map_tf();  
     
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
        if(MultAgSln.GetMessages())
        {  
          MultAgSln.AddSolution(Cspc);  // add the path from the message to our cspace
          found_path = true;
        }
        MultAgSln.SendMessageUDP(prob_success);
      }
      now_time = clock(); 
    }
    now_time = clock(); 
  }
  printf("Done with path planning phase\n");
  
  // record that this agent has reach the end of path planning
  MultAgSln.FinalSolutionSent[agent_number] = 1;
  
  if(mode == 2) // just tell the clients to start moving, and server also starts moving 
    MultAgSln.moving = true;
  
  // now we try to get consensus among the agents as to who's solution to use
  if(mode == 0 || mode == 1 || (mode == 2 && agent_number == 0))
    phase_two_start_t = clock();

  while(!MultAgSln.StartMoving())
  {
    // while we cannot move, we could refine our own solution, in case everybody votes to use our solution
      
    // broadcast tf
    //broadcast_map_tf();  
      
    start_wait_t = clock();
    now_time = clock();
    while(difftime_clock(now_time, start_wait_t) < sync_message_wait_time)
    { 
      now_time = clock();
    }
      
    if(mode == 2 && agent_number != 0)
        break;
    
    // while we cannot move, we broadcaset the best solution to everybody, and recieve thier best solutions
    MultAgSln.GetMessages();  
    MultAgSln.SendMessageUDP(prob_success);
   
    printf(" waiting, not moving\n");
  }
  now_time = clock();
      
  float phase_two_time = difftime_clock(now_time,phase_two_start_t); // time since planning ended until this robot is aware of an agreement
  float total_time = difftime_clock(now_time,start_time);
  printf("Done with communication phase, (which took %f secs)\n", phase_two_time); 
  printf("agent #%d found the best overall solution, with length %f \n", MultAgSln.best_solution_agent, MultAgSln.best_solution_length);
  
  // save this info to a file so we can look at stats later
  if(total_agents > 1)
    data_dump(experiment_name, prob_success, min_clock_to_plan, phase_two_time, Cspc, MultAgSln,actual_solution_time,total_time);
  
  // now we just broadcast the final solution, in case other robots didn't get it
  
  
  while(!display_path) // if we want to display the path then we ignore this part
  {       
    //printf("%f %f %f \n", lookup_sum/n_lookup, out_collision/n_collision, out_sum/n_out );
    //printf("%f %f %f \n", elookup_sum/en_lookup, eout_collision/en_collision, eout_sum/en_out );
   
      
    start_wait_t = clock();
    now_time = clock();
    while(difftime_clock(now_time, start_wait_t) < sync_message_wait_time)
    { 
      now_time = clock();
    }

    if(mode == 2 && agent_number != 0)
      continue; 

    MultAgSln.GetMessages();
    MultAgSln.SendMessageUDP(prob_success);

    // now extract this robot's path and send on to the controller
    
    calculate_rotation(MultAgSln.BestSolution);
    
    extract_and_translate_solution(ThisAgentsPath, MultAgSln.BestSolution, Scene.translation, agent_number, world_dims);
    
    calculate_times(Parametric_Times, MultAgSln.BestSolution, 0.2, PI/2);

    publish_global_path(ThisAgentsPath, Parametric_Times); 

    printf("moving\n");
  }

  #ifdef using_glut
    //  Pass control to GLUT so it can interact with the user
    glutMainLoop();
  #endif     
   
  ros::spinOnce();
  loop_rate.sleep();
    
  // shutdown ros stuff
  pose_sub.shutdown();
  goal_sub.shutdown();
  global_path_pub.shutdown();
  system_update_pub.shutdown();
    
  destroy_pose(robot_pose);
  destroy_pose(goal_pose);
}