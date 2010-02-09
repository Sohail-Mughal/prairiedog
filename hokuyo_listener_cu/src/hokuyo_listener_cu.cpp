/*  
 *  Copyrights:
 *  Stephen, Greg, Vijeth Sept. 2009
 *  Michael Otte Sept. 2009
 *
 *  This file is part of hokuyo_listener_cu.
 *
 *  hokuyo_listener_cu is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  hokuyo_listener_cu is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with hokuyo_listener_cu. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 *
 *
 *  This node listens to the hokuyo laser scanner node provided with ros
 *  and also the localization system. It then cleans and transforms the 
 *  raw laser scan data before sending it out for other nodes to use, along
 *  with the origin of the laser scanner.
 */


#include <math.h>
#include <vector>
#include <list>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

#include "hokuyo_listener_cu/PointCloudWithOrigin.h"

#ifndef PI
  #define PI 3.1415926535897
#endif
    
#define FORCED_MAX_RANGE // if defiend, this causes readings > max range to be reset to max range
        
using namespace std;

struct POSE;
typedef struct POSE POSE;

struct POINT;
typedef struct POINT POINT;

// globals for transform, these can be reset via the parameter server, see main()
float LASER_SCANNER_X_OFFSET = 0.15;     // m in robot coordinate system 
float LASER_SCANNER_Y_OFFSET = 0.0;      // m in robot coordinate system 
float LASER_SCANNER_THETA_OFFSET = PI/6; // rad in laser coordinate system
bool using_tf = true;                    // when set to true, use the tf package

// global ROS subscriber handles
ros::Subscriber scan_sub;
ros::Subscriber pose_sub;

// global ROS publisher handles
ros::Publisher laser_scan_pub;

// globlas for robot pose
POSE* robot_pose = NULL;

// other globals
bool setup_tf = true; // flag used to indicate that required transforms are available

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
POSE* make_pose(float x, float y, float z)
{
  POSE* pose = (POSE*)calloc(1, sizeof(POSE));
  pose->x = 0;
  pose->y = 0;
  pose->z = 0;
  pose->alpha = 0;

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

/* -------------------------- POINT -------------------------------------*/
struct POINT
{
    float x;
    float y;
    float z;
};

/*---------------------------- ROS tf functions -------------------------*/
void broadcast_scanner_tf()
{
 
  static tf::TransformBroadcaster br;  
    
  tf::Transform transform;   
  transform.setOrigin(tf::Vector3(LASER_SCANNER_X_OFFSET, LASER_SCANNER_Y_OFFSET, 0));
  
  //transform.setRotation(tf::Quaternion(LASER_SCANNER_THETA_OFFSET, 0, 0));    // this is currently pyr, but being depreciated and then changed to rpy
  tf::Quaternion Q;
  Q.setRPY(0, 0, LASER_SCANNER_THETA_OFFSET);
  transform.setRotation(Q);
  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_cu", "/scanner_cu"));  
}

/*---------------------- ROS Publisher Functions ------------------------*/
// NOTE publish_point_cloud() has been combined with scan_callback()


/*--------------------------- ROS callbacks -----------------------------*/
void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO_STREAM("Received time: "<<msg->scan_time<<" with header:"<<msg->header.frame_id);
  //ROS_INFO("%f, %f, %f, %f, %f", msg->angle_min, msg->angle_max, msg->angle_increment, msg->range_min, msg->range_max);


  //other info in LaserScan msg
  // 	msg.angle_min
  // 	msg.angle_max
  // 	msg.angle_increment
  // 	msg.time_increment
  // 	msg.scan_time
  // 	msg.range_min
  // 	msg.range_max
  // 	msg.ranges[]
  // 	msg.intensities[]
    
    
  if(robot_pose == NULL)
    return;
    
  int length = msg->ranges.size();
  //printf(" here \n\n\n\n %d \n\n\n\n", length);
    
  float inc = msg->angle_increment;
    
  // first pass, see how many values are actually in range
  float range_min = msg->range_min;
  float range_max = msg->range_max;
  int num_valid = 0;
  for(int i = 0; i < length; i++)
  {
    #ifdef FORCED_MAX_RANGE
      if(msg->ranges[i] >= range_min || msg->ranges[i] == 0) // zero means no reading, which could happen in a large open area
        num_valid++;      
    #else
      if(msg->ranges[i] >= range_min && msg->ranges[i] <= range_max)
        num_valid++;
    #endif
  }
    
  if(num_valid <= 0)
    return;
   
  // second pass, populate outgoing message with scan data transformed into the map coordinate system
  hokuyo_listener_cu::PointCloudWithOrigin outgoing_msg;
  
  float alpha = -120/180*PI;
  if(using_tf)
  {
    outgoing_msg.cloud.points.resize(num_valid+1);    // +1 is for a hack to get an easy origin transform (reset to num_valid afterward)
    outgoing_msg.cloud.header.frame_id = "/map_cu";    
      
    sensor_msgs::PointCloud temp_message;
    temp_message.points.resize(num_valid+1);
    temp_message.header.frame_id = "/scanner_cu";  
            
    int j = 0;
    for(int i = 0; i < length; i++)
    {
      #ifdef FORCED_MAX_RANGE  
        if(msg->ranges[i] >= range_min || msg->ranges[i] == 0)
        {
          // transform 1, from range and degree in laser scanner to x and y in robot local coordinate frame  
          if(msg->ranges[i] <= range_max && msg->ranges[i] != 0)
          {
            temp_message.points[j].x = msg->ranges[i]*sin(alpha);
            temp_message.points[j].y = msg->ranges[i]*cos(alpha);
            temp_message.points[j].z = 0;
          }
          else
          {
            temp_message.points[j].x = range_max*sin(alpha);
            temp_message.points[j].y = range_max*cos(alpha);
            temp_message.points[j].z = 0;    
          }
          j++;
        }
      #else
        if(msg->ranges[i] >= range_min && msg->ranges[i] <= range_max)
        {
          // transform 1, from range and degree in laser scanner to x and y in robot local coordinate frame
          temp_message.points[j].x = msg->ranges[i]*sin(alpha);
          temp_message.points[j].y = msg->ranges[i]*cos(alpha);
          temp_message.points[j].z = 0;
          j++;
        }            
      #endif
      alpha += inc;
    }
  
    // save origin
    temp_message.points[j].x = 0;
    temp_message.points[j].y = 0;
    temp_message.points[j].z = 0;
        
    // actually do the transfrom int the map frame
    static tf::TransformListener listener;

    while(setup_tf)  // there is usually a problem looking up the first transform, so do this to avoid that
    {
      setup_tf = false;  
      try
      {    
        broadcast_scanner_tf();  // make sure that we've broadcast /scanner_cu at least once
        listener.waitForTransform("/scanner_cu", "/map_cu" , ros::Time(0), ros::Duration(1.0));
        listener.transformPointCloud(std::string("/map_cu"), temp_message, outgoing_msg.cloud);
      }
      catch(tf::TransformException ex)
      { 
        //printf("attempt failed \n");
        setup_tf = true;  
      }   
    }    
    
    try
    {
      listener.transformPointCloud(std::string("/map_cu"), temp_message, outgoing_msg.cloud);
    }
    catch(tf::TransformException ex)
    { 
      ROS_ERROR("%s",ex.what());
      return;
    }
    
    // extract origin
    outgoing_msg.origin.x = outgoing_msg.cloud.points[num_valid].x;
    outgoing_msg.origin.y = outgoing_msg.cloud.points[num_valid].y;
    outgoing_msg.origin.z = outgoing_msg.cloud.points[num_valid].z;
    outgoing_msg.cloud.points.resize(num_valid);
  }
  else // not using tf
  {
    outgoing_msg.cloud.points.resize(num_valid);  
    outgoing_msg.cloud.header.frame_id = "/map_cu";    
      
    float x,y,z;
    int j = 0; 
    for(int i = 0; i < length; i++)
    {
      #ifdef FORCED_MAX_RANGE  
        if(msg->ranges[i] >= range_min || msg->ranges[i] == 0)
        {
          // transform 1, from range and degree in laser scanner to x and y in robot local coordinate frame  
          if(msg->ranges[i] <= range_max && msg->ranges[i] != 0)
          {
            x = msg->ranges[i]*sin(alpha-LASER_SCANNER_THETA_OFFSET) + LASER_SCANNER_X_OFFSET;
            y = msg->ranges[i]*cos(alpha-LASER_SCANNER_THETA_OFFSET) + LASER_SCANNER_Y_OFFSET;
            z = 0;
          }
          else
          {
            x = range_max*sin(alpha-LASER_SCANNER_THETA_OFFSET) + LASER_SCANNER_X_OFFSET;
            y = range_max*cos(alpha-LASER_SCANNER_THETA_OFFSET) + LASER_SCANNER_Y_OFFSET;
            z = 0;    
          }
        
          // transform 2 from local x, y to global x, y      
          outgoing_msg.cloud.points[j].x = x*robot_pose->cos_alpha - y*robot_pose->sin_alpha + robot_pose->x;
          outgoing_msg.cloud.points[j].y = x*robot_pose->sin_alpha + y*robot_pose->cos_alpha + robot_pose->y;
          outgoing_msg.cloud.points[j].z = z;
          j++;
        }
      #else
        if(msg->ranges[i] >= range_min && msg->ranges[i] <= range_max)
        {
          // transform 1, from range and degree in laser scanner to x and y in robot local coordinate frame
          x = msg->ranges[i]*sin(alpha-LASER_SCANNER_THETA_OFFSET) + LASER_SCANNER_X_OFFSET;
          y = msg->ranges[i]*cos(alpha-LASER_SCANNER_THETA_OFFSET) + LASER_SCANNER_Y_OFFSET;
          z = 0;
        
          // transform 2 from local x, y to global x, y      
          outgoing_msg.cloud.points[j].x = x*robot_pose->cos_alpha - y*robot_pose->sin_alpha + robot_pose->x;
          outgoing_msg.cloud.points[j].y = x*robot_pose->sin_alpha + y*robot_pose->cos_alpha + robot_pose->y;
          outgoing_msg.cloud.points[j].z = z;
          j++;
        }            
      #endif
      alpha += inc;
    }
    
    // save origin
    
    // transform 1, from range and degree in laser scanner to x and y in robot local coordinate frame
    x = LASER_SCANNER_X_OFFSET;
    y = LASER_SCANNER_Y_OFFSET;
    z = 0;
        
    // transform 2 from local x, y to global x, y      
    outgoing_msg.origin.x = x*robot_pose->cos_alpha - y*robot_pose->sin_alpha + robot_pose->x;
    outgoing_msg.origin.y = x*robot_pose->sin_alpha + y*robot_pose->cos_alpha + robot_pose->y;
    outgoing_msg.origin.z = z;    
  }
  
  laser_scan_pub.publish(outgoing_msg);
  
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
    
  // init pose
  if(robot_pose == NULL)
    robot_pose = make_pose(0,0,0);
    
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
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hokuyo_listener_cu");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
        
  // load globals from parameter server
  double param_input;
  bool bool_input;
  if(ros::param::get("hokuyo_listener_cu/laser_scanner_x", param_input)) 
    LASER_SCANNER_X_OFFSET = (float)param_input;                             // m in robot coordinate system 
  if(ros::param::get("hokuyo_listener_cu/laser_scanner_y", param_input)) 
    LASER_SCANNER_Y_OFFSET = (float)param_input;                             // m in robot coordinate system 
  if(ros::param::get("hokuyo_listener_cu/laser_scanner_theta", param_input)) 
    LASER_SCANNER_THETA_OFFSET = (float)param_input;                         // rad in laser coordinate syst
  if(ros::param::get("prairiedog/using_tf", bool_input)) 
    using_tf = bool_input;                                                   // when set to true, use the tf package
  
  // print data about parameters
  printf("Laser Scanner Local Offset (x, y, theta): [%f, %f, %f] \n", LASER_SCANNER_X_OFFSET, LASER_SCANNER_Y_OFFSET, LASER_SCANNER_THETA_OFFSET);
  
  // wait until the map service is provided (we need its tf /world_cu -> /map_cu to be broadcast)
  ros::service::waitForService("/cu/get_map_cu", -1);
  // wait until localization service is provided (we need its tf /world_cu -> /robot_cu to be broadcast)
  ros::service::waitForService("/cu/get_pose_cu", -1);
  
  // set up subscribers
  scan_sub = nh.subscribe("scan", 100, scan_callback);
  pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
        
  // set up publishers
  laser_scan_pub = nh.advertise<hokuyo_listener_cu::PointCloudWithOrigin>("/cu/laser_scan_cu", 1);
    
  while (nh.ok()) 
  {   
    if(using_tf)
      broadcast_scanner_tf();  
        
    ros::spinOnce();
    loop_rate.sleep();
  }
    
  // clean up subscribers and publishers
  scan_sub.shutdown();
  pose_sub.shutdown();
  laser_scan_pub.shutdown();
    
  destroy_pose(robot_pose);
}

