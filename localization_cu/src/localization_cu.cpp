/*  Copyright Michael Otte, University of Colorado, 9-9-2009
 *
 *  This file is part of Localization_CU.
 *
 *  Localization_CU is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Localization_CU is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Localization_CU. If not, see <http://www.gnu.org/licenses/>.
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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GridCells.h"

#include "localization_cu/GetPose.h"

#ifndef PI
  #define PI 3.1415926535897
#endif

struct POSE;
typedef struct POSE POSE;        
        
// globals that can be reset using parameter server, see main();
float MOVEMULT = 2;                  // if pose jumps globally more than this mult by the local movement, global pose is dropped (used to prune error gps data), this can be reset with the paramiter server see main()
bool USING_GPS = true;               // true if there is gps data available (causes node to wait until GPS is received to broadcast position, gps data includes user defined pose i.e. from visualization node)
bool using_tf = true;                // when set to true, use the tf package
float odometer_pose_x_init = 0;      // odometer_pose is the pose returned by the robot
float odometer_pose_y_init = 0;
float odometer_pose_z_init = 0;      // note z coord is basically unused 
float odometer_pose_theta_init = 0;
float posterior_pose_x_init = 0;     // posterior_pose is the best estimate of where the robot is now in the global coordinate frame
float posterior_pose_y_init = 0;
float posterior_pose_z_init = 0;
float posterior_pose_theta_init = 0; // note z coord is basically unused 
        
// publisher handles
ros::Publisher pose_pub;

// subscriber handles
ros::Subscriber odometer_pose_sub;
ros::Subscriber user_pose_sub;
ros::Subscriber stargazer_pose_sub;

// global ROS provide service server handles
ros::ServiceServer get_pose_srv;

// globals
POSE* posterior_pose = NULL; // our best idea of where we are in the global coordinate system
POSE* odometer_pose = NULL;  // pose as determined by on-board odometry

float local_offset[] = {0, 0, 0}; // x,y,theta  
float global_offset[] = {0, 0, 0}; // x,y,theta 
float accum_movement = -1;
bool received_gps = false;
bool setup_tf_1 = true; // flag used to indicate that required transforms are available
bool setup_tf_2 = true; // flag used to indicate that required transforms are available
bool setup_tf_3 = true; // flag used to indicate that required transforms are available

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

  if(pose->qw < 1)
  {
    pose->qw *= -1;   
    pose->qz *= -1; 
  }
  
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

/*---------------------------- ROS tf functions -------------------------*/
void broadcast_robot_tf()
{ 
  static tf::TransformBroadcaster br;  
   
  if(USING_GPS && !received_gps)
    return;
  
  tf::Transform transform;   
  transform.setOrigin(tf::Vector3(posterior_pose->x, posterior_pose->y, posterior_pose->z));
  
  //transform.setRotation(tf::Quaternion(posterior_pose->alpha, 0, 0)); // this is currently ypr, but being depreciated and then changed to rpy
  tf::Quaternion Q;
  Q.setRPY(0, 0, posterior_pose->alpha);
  transform.setRotation(Q);
  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world_cu", "/robot_cu"));  
}

bool get_robot_map_tf(tf::StampedTransform &transform)
{
  static tf::TransformListener listener;
    
  
  while(setup_tf_1)  // there is usually a problem looking up the first transform, so do this to avoid that
  {
    setup_tf_1 = false;  
    try
    {    
      listener.waitForTransform("/world_cu", "/map_cu", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/world_cu", "/map_cu", ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    { 
      //printf("attempt failed \n");
      ROS_ERROR("localization 1: %s",ex.what());
      setup_tf_1 = true;  
    }   
  } 
  
  try
  {
    listener.lookupTransform("/world_cu", "/map_cu", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("localization 1: %s",ex.what());
    return false;
  }
  return true;
}

/*------------------------ ROS Callbacks --------------------------------*/

void odometer_pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{       
  if(USING_GPS && !received_gps)
    return;
  
  // remember old posterior x and y
  float old_x = posterior_pose->x;
  float old_y = posterior_pose->y;
  
  // note this assumes robot rotation is only in the 2D plane (and also constant velocity = average velocity between updates)
  odometer_pose->x = msg->x;
  odometer_pose->y = msg->y;
  odometer_pose->z = 0;
  odometer_pose->alpha = msg->theta;
    
  // update global pose based on this local pose
  
  // transform #1, to local coordinate frame where robot was last time we had both global and local pose info
  float xl = odometer_pose->x - local_offset[0];
  float yl = odometer_pose->y - local_offset[1];
  
  // transform #2, to global coordinate frame where robot was last time we had both global and local pose info
  float angle = global_offset[2] - local_offset[2];
  float xg = xl*cos(angle) - yl*sin(angle);
  float yg = xl*sin(angle) + yl*cos(angle);
  
  // transform #3, to global coordinate frame
  posterior_pose->x = xg + global_offset[0];
  posterior_pose->y = yg + global_offset[1];
  posterior_pose->z = 0;
  
  posterior_pose->alpha = odometer_pose->alpha + angle;
  posterior_pose->cos_alpha = cos(posterior_pose->alpha);
  posterior_pose->sin_alpha = sin(posterior_pose->alpha);
  
  float r = sqrt(2-(2*cos(posterior_pose->alpha)));

  if(r == 0)
    posterior_pose->qw = 1;
  else
    posterior_pose->qw = sin(posterior_pose->alpha)/r; 
  
  posterior_pose->qx = 0;
  posterior_pose->qy = 0;
  posterior_pose->qz = r/2; 
     
  // normalize
  //float magnitude = sqrt(posterior_pose->qw*posterior_pose->qw + posterior_pose->qx*posterior_pose->qx + posterior_pose->qy*posterior_pose->qy + posterior_pose->qz*posterior_pose->qz);
  float magnitude = sqrt(posterior_pose->qw*posterior_pose->qw + posterior_pose->qz*posterior_pose->qz);
  if(posterior_pose->qw < 0)
    magnitude *= -1;    
  posterior_pose->qw /= magnitude;
  //posterior_pose->qx /= magnitude;
  //posterior_pose->qy /= magnitude;
  posterior_pose->qz /= magnitude;
  
  if(posterior_pose->qw > 1)
    posterior_pose->qw = 1;
  
  //while(posterior_pose->alpha > PI)
  //  posterior_pose->alpha -= 2*PI;   
  //while(posterior_pose->alpha < -PI)
  //  posterior_pose->alpha += 2*PI;
   
  // remember accumulated movement
  if(accum_movement == -1)
      accum_movement = 0;
  accum_movement += sqrt((old_x - posterior_pose->x)*(old_x - posterior_pose->x) + (old_y - posterior_pose->y)*(old_y - posterior_pose->y));
}

void user_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
  if(msg->header.frame_id != "/world_cu")
    ROS_INFO("received a message that is not for the map_cu frame");
 
  posterior_pose->x = msg->pose.position.x;
  posterior_pose->y = msg->pose.position.y;
  posterior_pose->z = msg->pose.position.z;
  posterior_pose->qw = msg->pose.orientation.w;
  posterior_pose->qx = msg->pose.orientation.x;
  posterior_pose->qy = msg->pose.orientation.y;
  posterior_pose->qz = msg->pose.orientation.z; 
         
  // normalize
  float magnitude = sqrt(posterior_pose->qw*posterior_pose->qw + posterior_pose->qx*posterior_pose->qx + posterior_pose->qy*posterior_pose->qy + posterior_pose->qz*posterior_pose->qz);
  if(posterior_pose->qw < 0)
    magnitude *= -1; 
  posterior_pose->qw /= magnitude;
  posterior_pose->qx /= magnitude;
  posterior_pose->qy /= magnitude;
  posterior_pose->qz /= magnitude;
  
  if(posterior_pose->qw > 1)
    posterior_pose->qw = 1;
  
  float qw = posterior_pose->qw;
  float qx = posterior_pose->qx;
  float qy = posterior_pose->qy;
  float qz = posterior_pose->qz; 
  
  posterior_pose->cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
  posterior_pose->sin_alpha = 2*qw*qz + 2*qx*qy; 
  posterior_pose->alpha = atan2(posterior_pose->sin_alpha, posterior_pose->cos_alpha);
  
  //while(posterior_pose->alpha > PI)
  //  posterior_pose->alpha -= 2*PI;   
  //while(posterior_pose->alpha < -PI)
  //  posterior_pose->alpha += 2*PI;
      
  // save most recent data from both local and global pose to use in transformation matrices
  
  local_offset[0] = odometer_pose->x;
  local_offset[1] = odometer_pose->y;
  local_offset[2] = odometer_pose->alpha;
  
  global_offset[0] = posterior_pose->x;
  global_offset[1] = posterior_pose->y;
  global_offset[2] = posterior_pose->alpha;
  
  accum_movement = -1;
  received_gps = true;
}

void stargazer_pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{   
  // check to make sure that this could be correct given accumulated local pose based movement since last global update
  float pose_diff = sqrt((msg->x - posterior_pose->x)*(msg->x - posterior_pose->x) + (msg->y - posterior_pose->y)*(msg->y - posterior_pose->y));
  if(accum_movement*MOVEMULT < pose_diff && accum_movement != -1) // ignore because the global pose jump is too far 
    return;
  
  // currently, we use the raw psudolite data as ground truth
  posterior_pose->x = msg->x;
  posterior_pose->y = msg->y;
  posterior_pose->z = 0;
  posterior_pose->alpha = -msg->theta; // note that stargazer uses left-handed convention

  float r = sqrt(2-(2*cos(posterior_pose->alpha)));

  if(r == 0)
    posterior_pose->qw = 1;
  else
    posterior_pose->qw = sin(posterior_pose->alpha)/r; 
  
  posterior_pose->qx = 0;
  posterior_pose->qy = 0;
  posterior_pose->qz = r/2;
 
  // normalize
  //float magnitude = sqrt(posterior_pose->qw*posterior_pose->qw + posterior_pose->qx*posterior_pose->qx + posterior_pose->qy*posterior_pose->qy + posterior_pose->qz*posterior_pose->qz);
  float magnitude = sqrt(posterior_pose->qw*posterior_pose->qw + posterior_pose->qz*posterior_pose->qz);
  if(posterior_pose->qw < 0)
    magnitude *= -1; 
  posterior_pose->qw /= magnitude;
  //posterior_pose->qx /= magnitude;
  //posterior_pose->qy /= magnitude;
  posterior_pose->qz /= magnitude;
  
  if(posterior_pose->qw > 1)
    posterior_pose->qw = 1;
  
  posterior_pose->cos_alpha = cos(posterior_pose->alpha);
  posterior_pose->sin_alpha = sin(posterior_pose->alpha);
  
  //while(posterior_pose->alpha > PI)
  //  posterior_pose->alpha -= 2*PI;   
  //while(posterior_pose->alpha < -PI)
  //  posterior_pose->alpha += 2*PI;
    
  // save most recent data from both local and global pose to use in transformation matrices
  local_offset[0] = odometer_pose->x;
  local_offset[1] = odometer_pose->y;
  local_offset[2] = odometer_pose->alpha;
  
  global_offset[0] = posterior_pose->x;
  global_offset[1] = posterior_pose->y;
  global_offset[2] = posterior_pose->alpha;
  
  // reset accumulated movement
  accum_movement = -1;
  received_gps = true;
}

/*-------------------------- ROS Publisher functions --------------------*/
//int num_its = 0;
void publish_pose()
{ 
  if(USING_GPS && !received_gps)
    return;
  
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "/map_cu";
  
  // if we are using tf, then we need to transform from the global coordinate system to the map coordiante system
  if(using_tf)
  { 
    geometry_msgs::PoseStamped world_msg;
    world_msg.header.frame_id = "/world_cu";
    world_msg.pose.position.x = posterior_pose->x;
    world_msg.pose.position.y = posterior_pose->y;
    world_msg.pose.position.z = posterior_pose->z;
    world_msg.pose.orientation.w = posterior_pose->qw; 
    world_msg.pose.orientation.x = posterior_pose->qx;
    world_msg.pose.orientation.y = posterior_pose->qy;
    world_msg.pose.orientation.z = posterior_pose->qz;
    
    static tf::TransformListener listener;

    while(setup_tf_2)  // there is usually a problem looking up the first transform, so do this to avoid that
    {
      setup_tf_2 = false;  
      try
      {    
        listener.waitForTransform("/world_cu", "/map_cu", ros::Time(0), ros::Duration(3.0));
        listener.transformPose(std::string("/map_cu"), world_msg, msg);  
      }
      catch(tf::TransformException ex)
      { 
        //printf("attempt failed \n");
        ROS_ERROR("mapper 2: %s",ex.what());
        setup_tf_2 = true;  
      }   
    }    
    
    try
    {  
      listener.transformPose(std::string("/map_cu"), world_msg, msg);   
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("mapper 2: %s",ex.what());
      return;
    }
  }
  else // just send the raw data
  {
    msg.pose.position.x = posterior_pose->x;
    msg.pose.position.y = posterior_pose->y;
    msg.pose.position.z = posterior_pose->z;  
    msg.pose.orientation.w = posterior_pose->qw; 
    msg.pose.orientation.x = posterior_pose->qx;
    msg.pose.orientation.y = posterior_pose->qy;
    msg.pose.orientation.z = posterior_pose->qz;
  }
  
  pose_pub.publish(msg);
  
  //printf(" published pose %d\n",num_its++);
  //print_pose(posterior_pose);
}

// service that provides pose
bool get_pose_callback(localization_cu::GetPose::Request &req, localization_cu::GetPose::Response &resp)
{  
  if(USING_GPS && !received_gps)
    return false;
  
  geometry_msgs::PoseStamped msg;
  resp.pose.header.frame_id = "/map_cu";
  
  if(using_tf)
  {
    tf::StampedTransform transform;
    
    if(!get_robot_map_tf(transform))
      return false; // don't send if there is an error getting the transform
    
    geometry_msgs::PoseStamped world_msg;
    world_msg.header.frame_id = "/world_cu";
    world_msg.pose.position.x = posterior_pose->x;
    world_msg.pose.position.y = posterior_pose->y;
    world_msg.pose.position.z = posterior_pose->z;
    world_msg.pose.orientation.w = posterior_pose->qw; 
    world_msg.pose.orientation.x = posterior_pose->qx;
    world_msg.pose.orientation.y = posterior_pose->qy;
    world_msg.pose.orientation.z = posterior_pose->qz;
    
    static tf::TransformListener listener;

    while(setup_tf_3)  // there is usually a problem looking up the first transform, so do this to avoid that
    {
      setup_tf_3 = false;  
      try
      {    
        listener.waitForTransform("/world_cu", "/map_cu", ros::Time(0), ros::Duration(3.0));
        listener.transformPose(std::string("/map_cu"), world_msg, resp.pose);
      }
      catch(tf::TransformException ex)
      { 
        //printf("attempt failed \n");
        ROS_ERROR("mapper 3: %s",ex.what());
        setup_tf_3 = true;  
      }   
    } 
    
    try
    {
      listener.transformPose(std::string("/map_cu"), world_msg, resp.pose);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("mapper 3: %s",ex.what());
      return false;
    }
    
  }
  else // just send the raw data
  { 
    resp.pose.pose.position.x = posterior_pose->x;
    resp.pose.pose.position.y = posterior_pose->y;
    resp.pose.pose.position.z = posterior_pose->z;  
    resp.pose.pose.orientation.w = posterior_pose->qw; 
    resp.pose.pose.orientation.x = posterior_pose->qx;
    resp.pose.pose.orientation.y = posterior_pose->qy;
    resp.pose.pose.orientation.z = posterior_pose->qz; 
  }
  
  //printf(" published pose via service %d\n",num_its++);
  //print_pose(posterior_pose);
  
  return true;
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "localization_cu");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
    
  // load globals from parameter server
  double param_input;
  bool bool_input;
  if(ros::param::get("localization_cu/max_pose_jump_ratio", param_input)) 
    MOVEMULT = (float)param_input;                                          // if pose jumps globally more than this mult by the local movement, global pose is dropped (used to prune error gps data), this can be reset with the paramiter server see main()
  if(ros::param::get("localization_cu/using_gps", bool_input)) 
    USING_GPS = bool_input;                                                 // true if there is gps data available (causes node to wait until GPS is received to broadcast position, gps data includes user defined pose i.e. from visualization node)
  if(ros::param::get("prairiedog/using_tf", bool_input)) 
    using_tf = bool_input;                                                  // when set to true, use the tf package
  if(ros::param::get("localization_cu/odometer_pose_x_init", param_input)) 
    odometer_pose_x_init = (float)param_input;                              // odometer_pose is the pose returned by the robot
  if(ros::param::get("localization_cu/odometer_pose_y_init", param_input)) 
    odometer_pose_y_init = (float)param_input;
  if(ros::param::get("localization_cu/odometer_pose_z_init", param_input)) 
    odometer_pose_z_init = (float)param_input;
  if(ros::param::get("localization_cu/odometer_pose_theta_init", param_input)) 
    odometer_pose_theta_init = (float)param_input;
  if(ros::param::get("localization_cu/posterior_pose_x_init", param_input)) 
    posterior_pose_x_init = (float)param_input;                             // posterior_pose is the best estimate of where the robot is now in the global coordinate frame
  if(ros::param::get("localization_cu/posterior_pose_y_init", param_input)) 
    posterior_pose_y_init = (float)param_input;
  if(ros::param::get("localization_cu/posterior_pose_z_init", param_input)) 
    posterior_pose_z_init = (float)param_input;
  if(ros::param::get("localization_cu/posterior_pose_theta_init", param_input)) 
    posterior_pose_theta_init = (float)param_input;

  // print basic info about parameters
  printf("movemult: %f \n", MOVEMULT);
  if(USING_GPS)
    printf("using gps\n");
  else
    printf("not using gps\n");
  if(using_tf)
    printf("using tf\n");
  else
    printf("not using tf\n");
  printf("odometer_pose_init: [%f, %f, %f] \n", odometer_pose_x_init, odometer_pose_y_init, odometer_pose_theta_init);
  printf("posterior_pose_init: [%f, %f, %f] \n", posterior_pose_x_init, posterior_pose_y_init, posterior_pose_theta_init);
  
  // initialize pose structs
  odometer_pose = make_pose(odometer_pose_x_init, odometer_pose_y_init, odometer_pose_z_init, odometer_pose_theta_init);
  posterior_pose = make_pose(posterior_pose_x_init, posterior_pose_y_init, posterior_pose_z_init, posterior_pose_theta_init);
    
  // set up publisher
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/pose_cu", 1);
    
  // set up subscribers
  odometer_pose_sub = nh.subscribe("/cu/odometer_pose_cu", 1, odometer_pose_callback);
  user_pose_sub = nh.subscribe("/cu/user_pose_cu", 1, user_pose_callback);
  stargazer_pose_sub = nh.subscribe("/cu/stargazer_pose_cu", 1, stargazer_pose_callback);

  // set up service servers
  get_pose_srv = nh.advertiseService("/cu/get_pose_cu", get_pose_callback);
    
  while (ros::ok()) 
  {
    //printf(" This is the localization system \n");
    //print_pose(posterior_pose);
    if(using_tf)
      broadcast_robot_tf();
            
    publish_pose();
      
    ros::spinOnce();
    loop_rate.sleep();
  }
    
    
  // destroy publisher
  pose_pub.shutdown();
    
  odometer_pose_sub.shutdown();
  user_pose_sub.shutdown();
  get_pose_srv.shutdown();
        
  destroy_pose(odometer_pose);
  destroy_pose(posterior_pose);
}
