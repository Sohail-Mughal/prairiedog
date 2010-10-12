/*  
 *  Copyrights:
 *  Ben Pearre, Patrick Mitchell, Marek, Jason Durrie Sept. 2009
 *  Michael Otte Sept. 2009
 *
 *  This file is part of irobot_create_cu.
 *
 *  irobot_create_cu is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  irobot_create_cu is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with irobot_create_cu. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 *
 *
 *  This node interacts with Brown's irobot create package, extracting pose
 *  and publishing it and providing drive commands based on a path that it 
 *  receives from an incoming topic.
 */


#include <assert.h>
#include <errno.h>
#include <math.h>
#include <netdb.h>
#include <sched.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include <irobot_create_rustic/irobot_create_controller.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"

#include "nav_msgs/Path.h"

#include "std_msgs/Int32.h"

#ifndef PI
  #define PI 3.1415926535897
#endif
      
#ifndef SQRT2
  #define SQRT2 1.414213562373095
#endif
 
#ifndef LARGE
  #define LARGE 1000000000
#endif
        
using namespace std;

struct POSE;
typedef struct POSE POSE;
        
// globals that can be reset using parameter server, see main();
float BUMPER_BACKUP_DIST = .015;   //(m) after a bumper hit, the robot backs up this much before going again
float BACKUP_SPEED = -.2;
float DEFAULT_SPEED = 0.2;
float DEFAULT_TURN = 0.2;
float BUMPER_THETA_OFFSET = PI/6;  // rad in robot coordinate system
float BUMPER_OFFSET = .35;         // (m) distance in robot coordinate system
bool using_tf = true;              // when set to true, use the tf package

bool path_has_orientation = true;  // if path contains orientation, set this to true
bool multi_robot_mode = true;      // set to true if doing multi_robot mode
ros::Time move_start_time;         // holds the time that we started moving, used for multi_robot_mode


// set up Brown's Controller
IRobotCreateController * controller = (IRobotCreateController*) NULL;

// publisher handles
ros::Publisher odometer_pose_pub;
ros::Publisher bumper_pose_pub;
ros::Publisher system_state_pub;
ros::Publisher target_pose_pub;
ros::Publisher turn_circle_pub;

// subscriber handles
ros::Subscriber user_control_sub;
ros::Subscriber pose_sub;
ros::Subscriber goal_sub; // global goal
ros::Subscriber global_path_sub;
ros::Subscriber system_update_sub;
ros::Subscriber user_state_sub;

// globals for defining robot velocity
float speed = 0;
float turn = 0;

float max_speed = 1;
float max_turn = 1;

float use_input_speed_increment = .1;
float use_input_turn_increment = .1;

int safe_path_exists = 0;

int new_global_path = 1;

int system_state = 0; // as advertised by this node
                      // 0 = initial state (havn't recieved enough info to start moving)
                      // 1 = planning and moving normally, 
                      // 2 = bumper hit, backing up
                      // 3 = no path to goal exists
                      // 4 = manual stop
                      // 5 = manual control

int  user_state = 0;  // as advertised by the visualization node
                      // 0 = passive, doing nothing
                      // 1 = manual stop
                      // 2 = manual control

bool backing_up = false; // set to true if robot gets a bumper hit
int state_prior_to_bumper_hit;
bool setup_tf = true; // flag used to indicate that required transforms are available
bool recieved_first_path = false; // flag used to indicate if we recieved the first path


// globals for robot and goal
POSE* robot_pose = NULL;
POSE* local_goal_pose = NULL;
POSE* global_goal_pose = NULL;

vector<POSE> global_path;

vector<vector<float> > first_path; // used in multi robot mode
vector<vector<float> > trajectory; // used in multi robot mode
int t_target_ind = -1;             // used in multi robot mode

float TARGET_SPEED = DEFAULT_SPEED;
float TARGET_TURN = DEFAULT_TURN;

/* Print the current state info to the console
void print_state() 
{
    controller->getX();
    controller->getY();
    
    controller->isBumpedLeft();
    controller->isBumpedRight();
    controller->getVolts();
    controller->getWatts();
    controller->getPercent();
    controller->isCharging();
}
*/

/* ------------------- a few function headers ---------------------------*/
float setSpeed(float newspeed);
float setTurn(float x);
// given [x, y, alpha, time] path p, this extracts [x, y, alpha, time] trajectory t at higher granularity defined by time_granularity
void extract_trajectory(vector<vector<float> >& t, vector<vector<float> >& p, float time_granularity);
void print_2d_float_vector(vector<vector<float> >& V);


/* ------------------- Misc functions -----------------------------------*/
float determinant(float a, float b, float c, float d)    // calculates the determinant of [a b; c d]
{
  return (a*d) - (c*b);
}

bool find_intersect(float a1x, float a1y, float a2x, float a2y, float b1x, float b1y, float b2x, float b2y, float& cx, float& cy) // given lines a and b defined by the 4 points, finds their intersection and puts the point in c
{
  float a = determinant(a1x, a1y, a2x, a2y);
  float b = a1x - a2x;
  float c = a1y - a2y;

  float d = determinant(b1x, b1y, b2x, b2y);
  float e = b1x - b2x;
  float f = b1y - b2y;

  float h = determinant(b, c, e, f);

  cx = determinant(a, b, d, e)/h;
  cy = determinant(a, c, d, f)/h;    
  
  if(-LARGE < cx && cx < LARGE && -LARGE < cy && cy < LARGE)
    return true;
  return false;
}


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

/*------------------------ ROS Callbacks --------------------------------*/
void user_control_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  if(msg->x == 0 && msg->theta == 0) // two zeros means stop the robot, error on the side of caution
  {
    setSpeed(0);
    setTurn(0);   
    
    //ROS_INFO_STREAM("speed: " << speed << "turn: " << turn );
    controller->setSpeeds(speed, turn);
  }
  else if(system_state == 5) // in the user control state
  {
  	if(msg->x > 0)
    {
        speed += use_input_speed_increment;
        if(speed > max_speed)
            speed = max_speed;
    }
    else if(msg->x < 0)
    {
        speed -= use_input_speed_increment;
        if(speed < -max_speed)
            speed = -max_speed;
    }
    else
    {
        speed = 0;
    }

    if(msg->theta > 0)
    {
        turn += use_input_turn_increment;
        if(turn > max_turn)
            turn = max_turn;
    }
    else if(msg->theta < 0)
    {
        turn -= use_input_turn_increment;
        if(turn < -max_turn)
            turn = -max_turn;  
    }
    else
    {
        turn = 0;
    } 
    //ROS_INFO_STREAM("speed: " << speed << "turn: " << turn );
    controller->setSpeeds(speed, turn);
  }
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

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) // global goal
{ 
  global_goal_pose->x = msg->pose.position.x;
  global_goal_pose->y = msg->pose.position.y;
  global_goal_pose->z = msg->pose.position.z;
  global_goal_pose->qw = msg->pose.orientation.w;
  global_goal_pose->qx = msg->pose.orientation.x;
  global_goal_pose->qy = msg->pose.orientation.y;
  global_goal_pose->qz = msg->pose.orientation.z; 
  
  float qw = global_goal_pose->qw;
  float qx = global_goal_pose->qx;
  float qy = global_goal_pose->qy;
  float qz = global_goal_pose->qz; 
  
  global_goal_pose->cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
  global_goal_pose->sin_alpha = 2*qw*qz + 2*qx*qy;
  global_goal_pose->alpha = atan2(global_goal_pose->sin_alpha, global_goal_pose->cos_alpha);
}

void global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{      
  int length = msg->poses.size();
  global_path.resize(length);
  
  for(int i = 0; i < length; i++)
  {
    global_path[i].x = msg->poses[i].pose.position.x;
    global_path[i].y = msg->poses[i].pose.position.y;
    global_path[i].z = msg->poses[i].pose.position.z;  
    
    if(path_has_orientation)
    {
      global_path[i].qw = msg->poses[i].pose.orientation.w;
      global_path[i].qx = msg->poses[i].pose.orientation.x;
      global_path[i].qy = msg->poses[i].pose.orientation.y;
      global_path[i].qz = msg->poses[i].pose.orientation.z;
    
      float qw = global_path[i].qw;
      float qx = global_path[i].qx;
      float qy = global_path[i].qy;
      float qz = global_path[i].qz;
  
      global_path[i].cos_alpha = qw*qw + qx*qx - qy*qy - qz*qz;
      global_path[i].sin_alpha = 2*qw*qz + 2*qx*qy;
      global_path[i].alpha = atan2(global_path[i].sin_alpha, global_path[i].cos_alpha);
      
      //printf("path theta: %f (%f, %f, %f, %f)\n", global_path[i].alpha, qw, qx, qy, qz);
    }
  } 
  
  if(multi_robot_mode) 
  {     
    if(!recieved_first_path || safe_path_exists == 0)
    {
      move_start_time = ros::Time::now();
      recieved_first_path = true;

      // save into first_path
      first_path.resize(length);
      
      for(int i = 0; i < length; i++)
      {
        first_path[i].resize(4);  // (x, y, alpha, time)
        first_path[i][0] = global_path[i].x;
        first_path[i][1] = global_path[i].y;
        first_path[i][2] = global_path[i].alpha;
        first_path[i][3] = global_path[i].z;
        
        //if(global_path[i].z > 1000)
        //{
        //  printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ \n");
        //  printf("sent in a large time :  global_path[%d].z= %d, (%u total or %d) \n", i, global_path[i].z, global_path.size(), length);   
        //  getchar(); 
        //}
        
      }
      extract_trajectory(trajectory, first_path, .1); 

      //print_2d_float_vector(first_path);
      //print_2d_float_vector(trajectory);
      //printf("(%u points) \n", trajectory.size());
      //getchar();
    }
    else // check to make sure this is the same as the first path we recieved
    {
      bool is_the_same = true;
      
      if((int)first_path.size() != length)
        is_the_same = false;
      
      for(int i = 0; i < length; i++)
      {
        if(first_path[i][0] != global_path[i].x || first_path[i][1] != global_path[i].y || first_path[i][2] != global_path[i].alpha || first_path[i][3] != global_path[i].z)
          is_the_same = false;
      }
      if(!is_the_same)
      {
        printf(" warning: recieved different multi_robot path than first path \n");
        return;
      }
    }
  }
  
  //ros::Duration elapsed_time = ros::Time::now() - move_start_time;
  //printf("time: %f \n", elapsed_time.toSec());
  
  new_global_path = 1;
  safe_path_exists = 1;
}

void system_update_callback(const std_msgs::Int32::ConstPtr& msg)
{      
  int data = msg->data;
 
  if(data == 1) // there is no safe path to the goal
  {
    safe_path_exists = 0;
  }
}

void user_state_callback(const std_msgs::Int32::ConstPtr& msg)
{      
  user_state = msg->data;
 
  if(user_state == 1)
  {
    backing_up = false;
    setSpeed(0);
    setTurn(0);   
  }
}

/*------------------------- ROS publisher functions ---------------------*/

void publish_odometer_pose(float x, float y, float theta)
{
  if(theta < -10 || theta > 10) // i.e. 10 is a stand-in for > 2*pi
    return;
    
  geometry_msgs::Pose2D msg;
  msg.x = x;
  msg.y = y;
  msg.theta = theta;
  
  odometer_pose_pub.publish(msg);       
}

void publish_bumper(bool left, bool right)
{
  float theta;  
  if(left && right)
    theta = 0;
  else if(left)
    theta = BUMPER_THETA_OFFSET;
  else
    theta = -BUMPER_THETA_OFFSET;
  
  
  if(using_tf)
  {   
    geometry_msgs::PoseStamped msg_temp;  
    msg_temp.header.frame_id = "/robot_cu";
    
    // transform 1, from range and degree in robot local to x and y in robot local coordinate frame
    msg_temp.pose.position.x = BUMPER_OFFSET*cos(theta);
    msg_temp.pose.position.y = BUMPER_OFFSET*sin(theta);
    msg_temp.pose.position.z = 0;
    msg_temp.pose.orientation.w = 1/SQRT2; 
    msg_temp.pose.orientation.x = 0;
    msg_temp.pose.orientation.y = 0;
    msg_temp.pose.orientation.z = 1/SQRT2;
    
    // transform 2, from local robot frame to /map_cu
    geometry_msgs::PoseStamped msg;  
    msg.header.frame_id = "/map_cu";
    
    static tf::TransformListener listener;

    while(setup_tf)  // there is usually a problem looking up the first transform, so do this to avoid that
    {
      setup_tf = false;  
      try
      {    
        listener.waitForTransform("/robot_cu", "/map_cu" , ros::Time(0), ros::Duration(3.0));
        listener.transformPose(std::string("/map_cu"), msg_temp, msg);   
      }
      catch(tf::TransformException ex)
      { 
        printf("attempt failed \n");
        ROS_ERROR("irobot_create: %s",ex.what());
        setup_tf = true;  
      } 
    } 

    try
    {
      listener.transformPose(std::string("/map_cu"), msg_temp, msg);   
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("irobot_create: %s",ex.what());
      return;
    }
    
    bumper_pose_pub.publish(msg); 
  }   
  else
  {
    // transform 1, from range and degree in robot local to x and y in robot local coordinate frame
    float y = BUMPER_OFFSET*sin(theta);
    float x = BUMPER_OFFSET*cos(theta);
      
    // transform 2 from local x, y to global x, y
    geometry_msgs::Pose2D msg;
    msg.x = x*robot_pose->cos_alpha - y*robot_pose->sin_alpha + robot_pose->x;
    msg.y = x*robot_pose->sin_alpha + y*robot_pose->cos_alpha + robot_pose->y;
    bumper_pose_pub.publish(msg); 
  } 
}

void publish_system_state(int state)
{
  // state: 0 = initial state (havn't recieved enough info to start moving)
  //        1 = planning and moving normally, 
  //        2 = bumper hit, backing up
  //        3 = no path to goal exists
  //        4 = manual stop
  //        5 = manual control
  
    
  std_msgs::Int32 msg;  
  
  msg.data = state;
  system_state_pub.publish(msg); 
}

void publish_target_pose(float x, float y, float theta)
{  
  geometry_msgs::Pose2D msg;
  msg.x = x;
  msg.y = y;
  msg.theta = theta;
  
  target_pose_pub.publish(msg);       
}

void publish_turn_circle(float x, float y, float theta)
{  
  geometry_msgs::Pose2D msg;
  msg.x = x;
  msg.y = y;
  msg.theta = theta;
  
  turn_circle_pub.publish(msg);       
}
/* ---------------------- navigation functions --------------------------*/

// given [x, y, alpha, time] path p, this extracts [x, y, alpha, time] trajectory t at higher granularity defined by time_granularity
void extract_trajectory(vector<vector<float> >& t, vector<vector<float> >& p, float time_granularity)
{
    
  if(p.size() < 1)
    printf("trying to extract trajectory from a path of length < 1 \n");
    
  t.resize(0);
  t.push_back(p[0]);
  for(int i = 1; i < (int)p.size(); i++)
  {    
    if(p[i][3] - p[i-1][3] < time_granularity) // just use next point
    {
      t.push_back(p[i]);
    }
    else // need to break path section up into multiple lengths
    {
      float start_time = p[i-1][3];
      float end_time = p[i][3];
      
      float alpha_start = p[i-1][2]; // should already be between -PI and PI
      float alpha_end = p[i][2];     // should already be between -PI and PI
      
      // in case it is not
      
      while(alpha_start < 0)
        alpha_start += 2*PI;
      while(alpha_start > 2*PI)
        alpha_start -= 2*PI;
      
      while(alpha_end < 0)
        alpha_end += 2*PI;
      while(alpha_end > 2*PI)
        alpha_end -= 2*PI;
      
      // now start and goal are both between 0 and 2 PI
      
      float delta_alpha = alpha_end - alpha_start;

      if(delta_alpha <= PI && delta_alpha >= 0) // end is ahead of start, but the difference is less than PI, so nothing needs to be done
      {
          
      }
      else if(delta_alpha >= -PI &&  delta_alpha <= 0) // start is ahead of end, but the magnitude of the difference is less than PI, so nothing needs to be done
      {
      
      }
      else if(delta_alpha >= 0) // (otherwise) it is better to go in the other direction and currently end is ahead of start, so we want to make end behind start
      {
        alpha_end -= 2*PI;  
      }
      else // delta_alpha <= 0 // (otherwise) it is better to go in the other direction and currently start is ahead of end, so we want to make end ahead of start
      {
        alpha_end += 2*PI;    
      }
          
//       while(delta_alpha < -PI)
//         delta_alpha += 2*PI;
//       while(delta_alpha > PI)
//         delta_alpha -= 2*PI;
//       
//       // now delta alpha is between -PI and PI
//       printf("delta alpha: %f \n", delta_alpha);
//       
//       if(delta_alpha < 0) // should have decreasing coords
//       {
//         while(alpha_start < alpha_end)
//           alpha_end -= 2*PI; 
//           
//       }
//       else // should have increasing coords
//       {
//         while(alpha_start > alpha_end)
//           alpha_end += 2*PI;  
//           
//       }
      
      
//       if(alpha_end > alpha_start && alpha_end - alpha_start > 0) // it is easier to go in the other direction
//         alpha_end -= 2*PI;
//       else if(alpha_start > alpha_end && alpha_start - alpha_end > 0) // it is easier to go in the other direction
//         alpha_start -= 2*PI;
          
      vector<float> this_path_point(4);
      for(float this_time = start_time + time_granularity; this_time < end_time; this_time += time_granularity)
      {
        
        float percentage_through =  (this_time - start_time)/(end_time - start_time);
        //printf("percentage_through: %f, this_time=%f, end_time=%f, start_time=%f, p[i].size()=%u, p[i][3]=%f \n", percentage_through, this_time, end_time, start_time, p[i].size(), p[i][3]);
        
        
        this_path_point[0] = p[i-1][0] + (p[i][0] - p[i-1][0])*percentage_through;  // x
        this_path_point[1] = p[i-1][1] + (p[i][1] - p[i-1][1])*percentage_through;  // y
        
        if(this_path_point[0] != t[t.size()-1][0] || this_path_point[1] != t[t.size()-1][1]) // location has changed
          this_path_point[2] = p[i-1][2];  // assuming that alpha of point i points the correct direction along segment {i,i+1}
        else // location has not changed, so need to worry about alpha     
        {
          this_path_point[2] = alpha_start + (alpha_end - alpha_start)*percentage_through;  // alpha
//           while(this_path_point[2] < -PI)
//             this_path_point[2] += 2*PI;
//           while(this_path_point[2] > PI)
//             this_path_point[2] -= 2*PI;
        }
        
        //printf("alpha %f \n", this_path_point[2]);
        
        this_path_point[3] = this_time;  // time
        
        t.push_back(this_path_point);
      }
      t.push_back(p[i]);
    }
  }
}


/* Move at a speed [0-1] */
float setSpeed(float newspeed) 
{
    speed = newspeed;
    controller->setSpeeds(speed, turn);
    return newspeed;
}

/* turn [0-1] */
float setTurn(float x) 
{
    turn = x;
    controller->setSpeeds(speed, turn);
    return x;
}

// this will cause the robot to rotate toward the specified (x,y) of the target_pose from its current position,
// it will not turn if it is within tolerance radians of the correct direction, in which case it returns 1
int turn_toward_location(POSE* target_pose, float tolerance)
{
  float desired_direction = atan2(target_pose->y-robot_pose->y, target_pose->x-robot_pose->x);
  float current_direction = robot_pose->alpha;
  
  float diff_direction = desired_direction - current_direction;
  
  // find on range of -PI to PI
  while(diff_direction > PI)
      diff_direction -= 2*PI;
  while(diff_direction < -PI)
      diff_direction += 2*PI;
  
  if(diff_direction > tolerance)
     setTurn(TARGET_TURN);
  else if(diff_direction < -tolerance)
     setTurn(-TARGET_TURN);
  else
  {
     setTurn(0);
     return 1;
  }
  return 0;     
}

// same as above but pose is in a float vector [x y alpha (time)]
int turn_toward_location(vector<float>& target_pose, float tolerance)
{
  float desired_direction = atan2(target_pose[1]-robot_pose->y, target_pose[0]-robot_pose->x);
  float current_direction = robot_pose->alpha;
  
  float diff_direction = desired_direction - current_direction;
  
  // find on range of -PI to PI
  while(diff_direction > PI)
      diff_direction -= 2*PI;
  while(diff_direction < -PI)
      diff_direction += 2*PI;
  
  if(diff_direction > tolerance)
     setTurn(TARGET_TURN);
  else if(diff_direction < -tolerance)
     setTurn(-TARGET_TURN);
  else
  {
     setTurn(0);
     return 1;
  }
  return 0;     
}

// same as (two) above, but will also drive forward when robot is within tolerance_fast radians of the correct direction
int turn_toward_location_fast(POSE* target_pose, float tolerance, float tolerance_fast)
{
  float desired_direction = atan2(target_pose->y-robot_pose->y, target_pose->x-robot_pose->x);
  float current_direction = robot_pose->alpha;
  
  float diff_direction = desired_direction - current_direction;
  
  // find on range of -PI to PI
  while(diff_direction > PI)
      diff_direction -= 2*PI;
  while(diff_direction < -PI)
      diff_direction += 2*PI;
  
  if(diff_direction >= 0 && diff_direction <= tolerance_fast)
     setSpeed(TARGET_SPEED);
  else if(diff_direction <= 0 && diff_direction >= -tolerance_fast)
     setSpeed(TARGET_SPEED);
  else
     setSpeed(0);
  
  if(diff_direction > tolerance)
     setTurn(TARGET_TURN);
  else if(diff_direction < -tolerance)
     setTurn(-TARGET_TURN);
  else
  {
     setTurn(0);
     return 1;
  }
  return 0;     
}

// same as above but pose is in a float vector [x y alpha (time)]
int turn_toward_location_fast(vector<float>& target_pose, float tolerance, float tolerance_fast)
{
  float desired_direction = atan2(target_pose[1]-robot_pose->y, target_pose[0]-robot_pose->x);
  float current_direction = robot_pose->alpha;
  
  float diff_direction = desired_direction - current_direction;
  
  // find on range of -PI to PI
  while(diff_direction > PI)
      diff_direction -= 2*PI;
  while(diff_direction < -PI)
      diff_direction += 2*PI;
  
  if(diff_direction >= 0 && diff_direction <= tolerance_fast)
     setSpeed(TARGET_SPEED);
  else if(diff_direction <= 0 && diff_direction >= -tolerance_fast)
     setSpeed(TARGET_SPEED);
  else
     setSpeed(0);
  
  if(diff_direction > tolerance)
     setTurn(TARGET_TURN);
  else if(diff_direction < -tolerance)
     setTurn(-TARGET_TURN);
  else
  {
     setTurn(0);
     return 1;
  }
  return 0;     
}

// this will cause the robot to rotate toward the specified orientation of the target_pose from its current position,
// it will not turn if it is within tolerance radians of the correct direction, in which case it returns 1
int turn_toward_heading(POSE* target_pose, float tolerance)
{
  float desired_direction = target_pose->alpha;
  float current_direction = robot_pose->alpha;
  
  float diff_direction = desired_direction - current_direction;
  
  // find on range of -PI to PI
  while(diff_direction > PI)
      diff_direction -= 2*PI;
  while(diff_direction < -PI)
      diff_direction += 2*PI;
  
  if(diff_direction > tolerance)
     setTurn(TARGET_TURN);
  else if(diff_direction < -tolerance)
     setTurn(-TARGET_TURN);
  else
  {
     setTurn(0);
     return 1;
  }
  return 0; 
}

// same as above but pose is in a float vector [x y alpha (time)]
int turn_toward_heading(vector<float>& target_pose, float tolerance)
{
  float desired_direction = target_pose[2];
  float current_direction = robot_pose->alpha;
  
  float diff_direction = desired_direction - current_direction;
  
  // find on range of -PI to PI
  while(diff_direction > PI)
      diff_direction -= 2*PI;
  while(diff_direction < -PI)
      diff_direction += 2*PI;
  
  if(diff_direction > tolerance)
     setTurn(TARGET_TURN);
  else if(diff_direction < -tolerance)
     setTurn(-TARGET_TURN);
  else
  {
     setTurn(0);
     return 1;
  }
  return 0; 
}


// this will cause the robot to move toward the target_pose from its current position,
// it will not move if it is within tolerance_radians and tolerance_distance of the correct direction and position, in which case it returns 1
int move_toward_pose(POSE* target_pose, float tolerance_distance, float tolerance_radians)
{
  float x_dist = target_pose->x - robot_pose->x;
  float y_dist = target_pose->y - robot_pose->y;
  float dist_to_goal = sqrt(x_dist*x_dist + y_dist*y_dist);
     
  //printf("dist to goal: %f, tolerance: %f \n", dist_to_goal, tolerance_distance);
    
  if(dist_to_goal > tolerance_distance) // the robot is not close enough
  {
    if(turn_toward_location(target_pose, tolerance_radians) == 1)   // the robot is facing in the correct direction to move closer
      setSpeed(TARGET_SPEED);
    else
      setSpeed(0); // not facing in the correct direction to move closer
  }
  else // the robot is close enough
  {
    setSpeed(0);  
    if(turn_toward_heading(target_pose, tolerance_radians) == 1) // the robot is facing in the correct direction
    {
      //printf(" at goal \n");
      return 1;
    }
  }
  return 0;
}

// same as above but pose is in a float vector [x y alpha (time)]
int move_toward_pose(vector<float>& target_pose, float tolerance_distance, float tolerance_radians)
{
  float x_dist = target_pose[0] - robot_pose->x;
  float y_dist = target_pose[1] - robot_pose->y;
  float dist_to_goal = sqrt(x_dist*x_dist + y_dist*y_dist);
     
  //printf("dist to goal: %f, tolerance: %f \n", dist_to_goal, tolerance_distance);
    
  if(dist_to_goal > tolerance_distance) // the robot is not close enough
  {
    if(turn_toward_location(target_pose, tolerance_radians) == 1)   // the robot is facing in the correct direction to move closer
      setSpeed(TARGET_SPEED);
    else
      setSpeed(0); // not facing in the correct direction to move closer
  }
  else // the robot is close enough
  {
    setSpeed(0);  
    if(turn_toward_heading(target_pose, tolerance_radians) == 1) // the robot is facing in the correct direction
    {
      //printf(" at goal \n");
      return 1;
    }
  }
  return 0;
}


// same as (two) above, except that if the robot is within tolerance_radians_fast of desired heading, then it moves forward
int move_toward_pose_fast(POSE* target_pose, float tolerance_distance, float tolerance_radians, float tolerance_radians_fast)
{
  float x_dist = target_pose->x - robot_pose->x;
  float y_dist = target_pose->y - robot_pose->y;
  float dist_to_goal = sqrt(x_dist*x_dist + y_dist*y_dist);
     
  //printf("dist to goal: %f, tolerance: %f \n", dist_to_goal, tolerance_distance);
    
  if(dist_to_goal > tolerance_distance) // the robot is not close enough
  {       
    turn_toward_location_fast(target_pose, tolerance_radians, tolerance_radians_fast);  // the robot is facing in the correct direction to move closer
  }
  else // the robot is close enough
  {
    setSpeed(0);  
    if(turn_toward_heading(target_pose, tolerance_radians) == 1) // the robot is facing in the correct direction
    {
      //printf(" at goal \n");
      return 1;
    }
  }
  return 0;
}

// same as above but pose is in a float vector [x y alpha (time)]
int move_toward_pose_fast(vector<float>& target_pose, float tolerance_distance, float tolerance_radians, float tolerance_radians_fast)
{
  float x_dist = target_pose[0] - robot_pose->x;
  float y_dist = target_pose[1] - robot_pose->y;
  float dist_to_goal = sqrt(x_dist*x_dist + y_dist*y_dist);
     
  //printf("dist to goal: %f, tolerance: %f \n", dist_to_goal, tolerance_distance);
    
  if(dist_to_goal > tolerance_distance) // the robot is not close enough
  {       
    turn_toward_location_fast(target_pose, tolerance_radians, tolerance_radians_fast);  // the robot is facing in the correct direction to move closer
  }
  else // the robot is close enough
  {
    setSpeed(0);  
    if(turn_toward_heading(target_pose, tolerance_radians) == 1) // the robot is facing in the correct direction
    {
      //printf(" at goal \n");
      return 1;
    }
  }
  return 0;
}

// calculates the radius and center of the circle that is tangent to the robot_pose and the trajectory segment [s1 s2], populates the approperiate values with the center of the circle and its radius
// returns 0 if circle cannot be calulated, 1 if the circle is calculate, and 2 if the robot must move forward more before turning
int calculate_turn_circle(POSE* robot_pose, const vector<float>& s1, const vector<float>& s2, float& center_x, float& center_y, float& radius)
{
  float robot_x1 = robot_pose->x;
  float robot_y1 = robot_pose->y;
  float robot_theta = PI/2-robot_pose->alpha;

  float segment_x1 = s1[0];
  float segment_y1 = s1[1];

  float segment_x2 = s2[0];
  float segment_y2 = s2[1];

  float robot_x2 = robot_x1+sin(robot_theta);
  float robot_y2 = robot_y1+cos(robot_theta);

  // find where the robot's current direction intersects the line that the segment is on
  float intersect_x, intersect_y;
  if(!find_intersect(robot_x1, robot_y1, robot_x2, robot_y2, segment_x1, segment_y1, segment_x2, segment_y2, intersect_x, intersect_y))
  {
    //printf("cannot find intersection 1 \n");
    return 0;
  }

  // figure out if the circle will intersect the segment before it ends
  float dist_RI = sqrt(((robot_x1-intersect_x)*(robot_x1-intersect_x)) + ((robot_y1-intersect_y)*(robot_y1-intersect_y)));
  float dist_IS2 = sqrt(((segment_x2-intersect_x)*(segment_x2-intersect_x)) + ((segment_y2-intersect_y)*(segment_y2-intersect_y)));
  if(dist_RI > dist_IS2)
  {
    //printf("dist_RI > dist_IS2 \n");
    return 2; // must drive forward more before turning
  }
  
  // find the point on the segment's line where the circle is tangent
  float dist_S = sqrt(((segment_x2-segment_x1)*(segment_x2-segment_x1)) + ((segment_y2-segment_y1)*(segment_y2-segment_y1)));
  float rise_S = segment_y2-segment_y1;
  float run_S = segment_x2-segment_x1;
  float px = intersect_x + (run_S/dist_S)*dist_RI;
  float py = intersect_y + (rise_S/dist_S)*dist_RI;

  // calculate a point that is on robot's local y axis
  float robot_x3 = robot_x1+sin(robot_theta+(PI/2));
  float robot_y3 = robot_y1+cos(robot_theta+(PI/2));

  // calculate point that is on line perpindicular to segment that goes through p
  float pp_theta = atan2(segment_y2-segment_y1, segment_x2-segment_x1) + PI/2;
  float ppx = px + cos(pp_theta);
  float ppy = py + sin(pp_theta);

  // find the center of the circle
  if(!find_intersect(robot_x1, robot_y1, robot_x3, robot_y3, px, py, ppx, ppy, center_x, center_y))
  {
    //printf("cannot find intersection 2 \n");
    return 0;
  }
  // calculate the radius of the circle
  radius = sqrt(((center_x - robot_x1)*(center_x - robot_x1)) + ((center_y - robot_y1)*(center_y - robot_y1)));
  //radius = sqrt( (center_x - px)^2 + (center_y - py)^2)  // another way to calculate the radius

  //printf("found a circle of radius %f \n", radius);
  return 1;
    
}


// finds the best turn circle given the robots current pose and the trajectory (considering only path segments b to c-1)
// returns 0 if none can be found
int find_best_turn_circle(POSE* r_pose, const vector<vector<float> >& traj, int b, int c, float& center_x, float& center_y, float& radius)
{
  radius = LARGE;
  bool found_a_circle = false;
  int seg_num = traj.size() - 2;
  if(c < seg_num)
    seg_num = c;    
  
  if(r_pose == NULL)
    return 0;
  
  float this_center_x;
  float this_center_y;
  float this_radius;
  int ret_value;      
  
  // currently "best" means the one that gets us to the path in the smallest radius without pivoting on the spot, ties broken by the furtherest point
  for(int i = b; i < seg_num; i++)
  {
    if(traj[i][0] != traj[i+1][0] || traj[i][1] != traj[i+1][1])  // if the points are different
    {
      ret_value = calculate_turn_circle(r_pose, traj[i], traj[i+1], this_center_x, this_center_y, this_radius);
    
      //printf("ret value: %d \n", ret_value);
      
      if(ret_value == 1) // found a turn circle for this segment
      {
        if(this_radius <= radius)
        {
          center_x = this_center_x;  
          center_y = this_center_y;    
          radius = this_radius;  
          found_a_circle = true; 
          //getchar();
        }  
      }
    }
  }
  
  if(found_a_circle)      
    return 1;
  
  return 0;
}


// returns true if the robot is going away from the segment sq->s2
bool is_diverging(POSE* robot_pose, const vector<float>& s1, const vector<float>& s2)
{
  double robot_x1 = robot_pose->x;
  double robot_y1 = robot_pose->y;
  double robot_theta = PI/2-robot_pose->alpha;

  double segment_x1 = s1[0];
  double segment_y1 = s1[1];

  double segment_x2 = s2[0];
  double segment_y2 = s2[1];

  double robot_x2 = robot_x1+(.2*sin(robot_theta)); // for robot radius approx .2
  double robot_y2 = robot_y1+(.2*cos(robot_theta)); // for robot radius approx .2

  // find where the robot's current direction intersects the line that the segment is on
  float intersect_x, intersect_y;
  if(!find_intersect(robot_x1, robot_y1, robot_x2, robot_y2, segment_x1, segment_y1, segment_x2, segment_y2, intersect_x, intersect_y))
  {
    //printf("cannot find intersection 1 \n");
    return false;
  }
  
  // find distance between robot and the intersection point
  double t_x = robot_x1 - (double)intersect_x;
  double t_y = robot_y1 - (double)intersect_y;
  double d1_intersect = sqrt((t_x*t_x) + (t_y*t_y));
  
  // find distance between robot ray end and intersect
  t_x = robot_x2 - (double)intersect_x;
  t_y = robot_y2 - (double)intersect_y;
  double d2_intersect = sqrt((t_x*t_x) + (t_y*t_y)); 
  
  if(d1_intersect < d2_intersect) // the center of the robot is closer to the intersect point then the leading edge of the robot, so diverging
  {
    //printf("diverging\n");
    return true;
  }
  //printf("converging\n");
  return false;
}


// this will cause the robot to follow a path, where the path is given in tems of list<POSE>
// it returns 1 when the robot reaches the end of the path. Note that it will attempt to drive 
// toward the nearest point on the path, and considers a point reached whenver it gets within
// tolerance_distance of it. when new_path_flag == 0, a speed up is achieved by remembering 
// the index of the next point so that calculation of the nearest point can be minimized
int next_p_index = 0;
int follow_path(vector<POSE>& path, float tolerance_distance, int new_path_flag)
{
  if(robot_pose == NULL)
      return 0;
    
  int length = path.size();
  if(length <= next_p_index)
      return 0;
    
  float min_dist;
  float this_dist;
  float x, y;
            
  if(new_path_flag == 1)
  {
    // need to calculate which point is the closest to the robot 
      
    x = path[0].x - robot_pose->x;
    y = path[0].y - robot_pose->y;
    min_dist = sqrt(x*x + y*y);
    next_p_index = 0;
    
    for(int i = 1; i < length; i++)
    {
      x = path[i].x - robot_pose->x;
      y = path[i].y - robot_pose->y;
      this_dist = sqrt(x*x + y*y);
      
      if(this_dist < min_dist)
      {
        min_dist = this_dist;
        next_p_index = i;
      }   
    }
  }
  else
  {
    x = path[next_p_index].x - robot_pose->x;
    y = path[next_p_index].y - robot_pose->y;
    min_dist = sqrt(x*x + y*y);  
  }
      
  // while the current target is within the tolerance_distance, look further down the path
  this_dist = min_dist;   
  while(this_dist <= tolerance_distance)
  {
    next_p_index++;
      
    if(next_p_index >= length)
    {
      // all points toward the end of the path are within the tolerance_distance
      return 1;
    }
        
    x = path[next_p_index].x - robot_pose->x;
    y = path[next_p_index].y - robot_pose->y;
    this_dist = sqrt(x*x + y*y);      
  }
  
  //printf(" ind: %d \n",next_p_index);
  //print_pose(&(path[next_p_index]));
  
  return move_toward_pose_fast(&(path[next_p_index]), tolerance_distance/2, PI/24, PI/6);
}

// this causes the robot to follow the trajectory T, where point T[i] has the form [x, y, alpha, time]
// when behind schedual the robot will still follow the trajectory, as it may go around static obstacles, but
// it will speed up to try to get where it should currently be. new_path_flag = 1 means that a new path has been sent
// time_look_ahead is the ammount of time into the future we consider to be now.

int current_place_index = 0;   // index of closest point to robot currently
int current_time_index = 0;    // index of point where robot should be currently
int follow_trajectory(vector<vector<float> >& T, float time_look_ahead, int new_path_flag)
{
  if(robot_pose == NULL)
    return 0;
    
  int length = T.size();
  if(length <= current_place_index)
      return 0;
    
  float min_dist;
  float this_dist;
  float x, y;
  
  int previous_current_place_index = 0;
  int previous_current_time_index = 0;
  
  
//   if(new_path_flag != 1) // if a new path has not been recieved
//   {   
//     previous_current_place_index = current_place_index;
//     previous_current_time_index = current_time_index;
//       
//   }   
//   else
//     printf("whaaaaa???? \n");
  
  
  
  // calculate which point is the closest to the robot, starting with previous_current_place_index
      
  // start with 2D location
  x = T[previous_current_place_index][0] - robot_pose->x;
  y = T[previous_current_place_index][1] - robot_pose->y;
  min_dist = sqrt(x*x + y*y);
  current_place_index = previous_current_place_index;
    
  for(int i = previous_current_place_index+1; i < length; i++)
  {
    x = T[i][0] - robot_pose->x;
    y = T[i][1] - robot_pose->y;
    this_dist = sqrt(x*x + y*y);
      
    if(this_dist < min_dist)
    {
      min_dist = this_dist;
      current_place_index = i;
    }   
  }
    
  // now take orientation into account if there are multiple trajectory points for this location
  float min_orientation_diff = T[current_place_index][2] - robot_pose->alpha;
  float this_orientation_diff;
  while(min_orientation_diff < -PI) 
    min_orientation_diff += 2*PI;
  while(min_orientation_diff > PI)
    min_orientation_diff -= 2*PI; 
  if(min_orientation_diff < 0)
    min_orientation_diff *= -1;
    
  for(int i = current_place_index+1; i < length; i++)
  {
    if(T[i][0] != T[i-1][0] || T[i][1] != T[i-1][1]) // x and y are not the same
      break;
        
    this_orientation_diff = T[i][2] - robot_pose->alpha;
      
    while(this_orientation_diff < -PI)
      this_orientation_diff += 2*PI;
    while(this_orientation_diff > PI)
      this_orientation_diff -= 2*PI;
    if(this_orientation_diff < 0)
      this_orientation_diff *= -1;
        
    if(this_orientation_diff < min_orientation_diff)
    {
      min_orientation_diff = this_orientation_diff;
      current_place_index = i;    
    }
  }
       
  // calculate which point the robot should currently be at
    
  // find "now" in terms of the time path coordinate
  ros::Duration elapsed_time = ros::Time::now() - move_start_time;
  float time_now = elapsed_time.toSec() + time_look_ahead;
  current_time_index = 0;
  for(int i = 0; i < length; i++)
  {
    current_time_index = i;
    if(time_now < T[i][3])
      break;
  }    
  
  // set the target location so we can broadcast it for visualization
  t_target_ind = current_time_index;
  //printf("t_target_ind = %d of %d \n", t_target_ind, length);
  
  int carrot_index; // point we actually steer at
  
  #ifndef old_controller 
          
  float time_ahead = T[current_place_index][3] - T[current_time_index][3];
  //printf("%f secs ahead of schedule \n", time_ahead);
   
  // calculate flags to use later for figuring out what to do based on different situations
  bool ahead_of_schedual = false;
  if(time_ahead > 0) // we are ahead of schedual
    ahead_of_schedual = true;
    
  bool too_far_away = false;
  if(min_dist > 0.2) // we are significantly off the path
    too_far_away = true;
  
  bool on_a_point = false;  // this is set later depending on circumstance  
  bool on_the_goal = false; // this is set later depending on circumstance  
  
  bool near_the_goal = false;
  float t_x = robot_pose->x - T[length-1][0];
  float t_y = robot_pose->y - T[length-1][1];
  float d_diff = sqrt(t_x*t_x + t_y*t_y);
  if(d_diff < 0.2)
    near_the_goal = true;
  
  bool diverging = false; // this is set later depending on circumstance
  
  // calculate a factor for how far we are ahead or behind schedual
  float time_ahead_rad = 2; // sec, for control if we are further away then this in time, then we want behaviour to be the same as if we were this far away in time
  if(time_ahead > time_ahead_rad)
    time_ahead = time_ahead_rad; 
  if(time_ahead < -time_ahead_rad)
    time_ahead = -time_ahead_rad;
  float time_ahead_factor = time_ahead/time_ahead_rad; // gives number between -1 and 1
  

  // different control modalities start here
  if(ahead_of_schedual && !too_far_away)
  {
    carrot_index = current_time_index;  // set carrot as the current time index because we are close enough to where we should be in both time and distance
  }
  else // behind schedual or too far away
  {   
    // set carrot point based on a little ahead of where we are now
    float time_ahead = .5;
    carrot_index = current_place_index;
    while(carrot_index < length - 1)
    {    
      if(T[carrot_index][3] - T[current_place_index][3] >= time_ahead)
        break;
        
      carrot_index++;
    }
    while(carrot_index < length - 1)  // ignore rest of pivots
    {
      if(T[carrot_index][1] != T[carrot_index+1][1] || T[carrot_index][0] != T[carrot_index+1][0])
        break;
        
      carrot_index++;
    }
    if(carrot_index >= current_time_index)  // don't want to end up going too ahead of where we should be
        carrot_index = current_time_index;  
  }
    
  publish_turn_circle(T[carrot_index][0], T[carrot_index][1], .1); // publish for visualization
    
  if(T[carrot_index][0] == T[length-1][0] && T[carrot_index][1] == T[length-1][1])  // steering at the goal (not considering orientation)
  {
    //printf("goal -- %d:[%f, %f], %d:[%f, %f], %d \n", carrot_index, T[carrot_index][0], T[carrot_index][1], length-1, T[length-1][0], T[length-1][1], current_place_index);
    near_the_goal = true;
    on_the_goal = true;
    on_a_point = true; 
  }
  else if(T[carrot_index][1] == T[carrot_index+1][1] && T[carrot_index][0] == T[carrot_index+1][0])  // steering at a point
    on_a_point = true; 
    
  if(near_the_goal)
  {
    // use point navigation to get us the last little bit of the way there    
    
    // find distance to goal from robot
    float temp_x = robot_pose->x - T[length-1][0];
    float temp_y = robot_pose->y - T[length-1][1];
    float dist_diff = sqrt(temp_x*temp_x + temp_y*temp_y);
    
    // get a factor between 0 and 1 for how far away we are
    float dist_factor = dist_diff/0.2;
    if(dist_factor > 1)
      dist_factor = 1;
        
    //printf("moving toward goal, %f \n", dist_factor);  
    
    TARGET_SPEED = DEFAULT_SPEED*(1-sin(dist_factor*PI/2+PI/2));
    TARGET_TURN = DEFAULT_TURN;
        
    return move_toward_pose_fast(T[length-1], .05, PI/24, PI/24);
  }
  else if(on_a_point) // going toward a point
  {
    if(!too_far_away) // basically at point
    {
      TARGET_SPEED = 0;
    }
    else // not at point yet
    {
      // find distance to carrot from robot
      float temp_x = robot_pose->x - T[carrot_index][0];
      float temp_y = robot_pose->y - T[carrot_index][1];
      float dist_diff = sqrt(temp_x*temp_x + temp_y*temp_y);
    
      // get a factor between 0 and 1 for how far away we are
      float dist_factor = dist_diff/0.2;
      if(dist_factor > 1)
        dist_factor = 1;
        
      printf("moving toward point, %f \n", dist_factor);  
    
      TARGET_SPEED = DEFAULT_SPEED*(1-sin(dist_factor*PI/2+PI/2)); 
    }
  }
  else // going toward a segment
  {
    float delta_time = T[carrot_index+1][3] -  T[carrot_index][3];
      
    float temp_x = T[carrot_index+1][0] - T[carrot_index][0];
    float temp_y = T[carrot_index+1][1] - T[carrot_index][1];
    float dist_diff = sqrt(temp_x*temp_x + temp_y*temp_y);
            
    TARGET_SPEED = (dist_diff/delta_time)*(1-sin(time_ahead_factor*PI/2));   // first factor is expected speed, second factor adjusts based on if we are ahead or behind schedual
  
    diverging = is_diverging(robot_pose, T[carrot_index], T[carrot_index+1]); 
  }
    
  // find approperiate heading
  float desired_direction;
  if(too_far_away) // desired dirction is directly at carrot
    desired_direction = atan2(T[carrot_index][1] - robot_pose->y, T[carrot_index][0] - robot_pose->x);
  else // desired direction is along carrot
    desired_direction = T[carrot_index][2];
   
  float current_direction = robot_pose->alpha;
  float diff_direction = desired_direction - current_direction;  
  // find diff_direction on range -PI to PI
  while(diff_direction > PI)
    diff_direction -= 2*PI;
  while(diff_direction < -PI)
    diff_direction += 2*PI;
    
  if(diff_direction < -PI/3 || diff_direction > PI/3) // if not facing kind of in the correct direction, just rotate
    TARGET_SPEED = 0;
  
  diff_direction = .5*atan(diff_direction); // scale a bit
    
  if(diverging && !too_far_away)  // we want to add add aditional turning based on how far the robot is away from the segment
  {
    float lateral_dist = min_dist;  // not exactly lateral, but sideways from the segment
    if(lateral_dist > 0.1)        
      lateral_dist = 0.1;
      
    // get on range 0 to 1
    lateral_dist = (lateral_dist)/.1;
        
    float diff_lateral = 1-sin(lateral_dist*PI/2+(PI/2));       
              
    //printf("diff_direction: %f, diff_lateral: %f \n", diff_direction, diff_lateral);
      
    if(diff_direction < 0)
      TARGET_TURN = diff_direction - diff_lateral;
    else
      TARGET_TURN = diff_direction + diff_lateral; 
  }
  else
  {
    TARGET_TURN = diff_direction; 
  }
    
  setSpeed(TARGET_SPEED);
  setTurn(TARGET_TURN);   
  return 0;  

  
  
  
  
  
//   // while carrot_index is within 0.2, look further down the path
//   carrot_index = current_place_index;
//  
//   
//   
//   
//    this_dist = min_dist; 
//   
//   float this_rad = 0;
//   float th;
//   
//   while(this_dist <= 0.2)
//   { 
//     if(carrot_index >= length - 1)  
//       break;
//     
//     carrot_index++;
// 
//     x = T[carrot_index][0] - robot_pose->x;
//     y = T[carrot_index][1] - robot_pose->y;
//     this_dist = sqrt(x*x + y*y);  
//     
//     th = T[carrot_index][2] - robot_pose->alpha;
//     
//     // find th on range -PI to PI
//     while(th > PI)
//       th -= 2*PI;
//     while(th < -PI)
//       th += 2*PI;
//     
//     // now take abs(th)
//     if(th < 0)
//       th *= -1;
//     
//     this_rad = th;
//     
//     if(T[carrot_index][0] == T[carrot_index-1][0] && T[carrot_index][1] == T[carrot_index-1][1])
//       break;
//   }    
//   
//   
//   
//   //float center_x = 0;
//   //float center_y = 0;
//   //float radius = 0;
//   //find_best_turn_circle(robot_pose, T, current_place_index, current_time_index+1, center_x, center_y, radius);
//   //publish_turn_circle(center_x, center_y, radius);
//   
//   if(carrot_index >= length-1)  // on a point (the goal)
//   {
//     TARGET_SPEED = 0;
//   }
//   if(T[carrot_index][1] == T[carrot_index+1][1] && T[carrot_index][0] == T[carrot_index+1][0])  // on a point
//   {
//     TARGET_SPEED = 0;  
//   }
//   else // going along a path segment
//   {
//     DEFAULT_SPEED - DEFAULT_SPEED*sin(time_ahead_factor*PI/2); 
//     
//     // think about setting speed to 0 here if abs(T[carrot_index+1][2] - T[carrot_index][2]) > pi/6
//   }
//   
//   TARGET_TURN = 0 ; //DEFAULT_TURN - DEFAULT_TURN*sin(time_ahead_factor*PI/2);
//       
//  
//   
//  // else if(this_dist <= .2 || on_a_point) // we want to rotate to the orientation of the carrot
//  // {
//       
//     
//   
//  // }
//   //else
//   //  move_toward_pose_fast(T[carrot_index], 0, PI/6, PI/6); 
//   
// 
//   
  
  
  
  
  #else
  
  if(current_time_index == current_place_index) // we are on schedual
  {   
    carrot_index = current_time_index;
    
    // use default speeds
    TARGET_SPEED = DEFAULT_SPEED;
    TARGET_TURN = DEFAULT_TURN;
    
    printf("on schedule \n");
  }
  else if(current_time_index < current_place_index) // we are ahead of schedual
  {
    carrot_index = current_time_index;  
      
    // calcuklate how far we are from carrot
    float d_x = T[carrot_index][0] - robot_pose->x;
    float d_y = T[carrot_index][1] - robot_pose->y;
    float d_dist = sqrt(d_x*d_x + d_y*d_y); 
    
    if(d_dist > .2)
    {
      // stop and wait, maybe change this to just a slow down if faster speeds are used
      setSpeed(0); 
      setTurn(0); 
    }
    else
    {
      // stop but keep turning toward the orientation we want, maybe change this to just a slow down if faster speeds are used
      setSpeed(0); 
      TARGET_TURN = DEFAULT_TURN;  
    }
        
    float time_ahead = T[current_place_index][3] - T[current_time_index][3];
    
    printf("%f secs ahead of schedule \n", time_ahead-time_look_ahead);
  }
  else // we are behind schedule
  {
    // while carrot_index is within 0.2, look further down the path
    carrot_index = current_place_index;  
    this_dist = min_dist;   
    while(this_dist <= 0.2)
    { 
      if(carrot_index >= current_time_index)
      {
        carrot_index = current_time_index;
        break;
      }

      carrot_index++;

      x = T[carrot_index][0] - robot_pose->x;
      y = T[carrot_index][1] - robot_pose->y;
      this_dist = sqrt(x*x + y*y);  
    }    

    // speed up based on how far behind we are
    float time_behind = T[current_time_index][3] - T[current_place_index][3];
    
    float second_order_speed_increase_paramiter = .3;
    float second_order_turn_increase_paramiter = 1.0;
    float speed_increase = time_behind*time_behind*second_order_speed_increase_paramiter;
    if(speed_increase > .1)
      speed_increase = .1;
            
    float turn_increase = time_behind*time_behind*second_order_turn_increase_paramiter;
    if(turn_increase > .1)
      turn_increase = .1;
    
    if(current_time_index < current_place_index) // ahead of schedual
    {
      speed_increase *= -1.0;
      turn_increase *= -1.0;
      printf("%f secs ahead of schedule \n", -time_behind-time_look_ahead);
    }
    else
      printf("%f secs behind schedule \n", time_behind-time_look_ahead);

    TARGET_SPEED = DEFAULT_SPEED + speed_increase;
    TARGET_TURN = DEFAULT_TURN + turn_increase; 
  }
  
  
  if(carrot_index == (int)T.size()-1) // the carrot is at the goal
  {
    TARGET_SPEED = 0; 
    printf("near to goal \n");
  } 
  
  printf("%d %d %f %f \n", current_time_index, current_place_index, T[current_time_index][3], T[current_place_index][3]);
  
  if(carrot_index > current_place_index && (T[carrot_index][0] != T[current_place_index][0] || T[carrot_index][1] != T[current_place_index][1]))
  {
     // if there is a location difference between current_place_index and carrot_index we need to turn at the new place we want to go
      
    move_toward_pose_fast(T[carrot_index], 0, PI/6, PI/6); 
printf("case 1 \n");
  }
  else if(carrot_index > current_place_index)
  {
printf("case 2 \n");

    // othersise, we just want to turn to face the heading of carrot_index
    turn_toward_heading(T[carrot_index],  PI/24);
  }
  else // we are at the carrot index already
  {
    printf("case 3 \n");

    // keep going at previously set speed
    turn_toward_heading(T[carrot_index],  PI/24);
  }
  
  if(current_place_index == length-1)
    return turn_toward_heading(T[carrot_index],  PI/24);
  
  #endif
  
  return 0;
}


// returns 1 if the robot is less than dist to the goal
int within_dist_of_goal(float dist)
{
  float x_dist = global_goal_pose->x - robot_pose->x;
  float y_dist = global_goal_pose->y - robot_pose->y;
  float dist_to_goal = sqrt(x_dist*x_dist + y_dist*y_dist);

  if(dist_to_goal < dist)
    return 1;
  else
    return 0;
}

void print_2d_float_vector(vector<vector<float> >& V)
{
  for(int i = 0; i < (int)V.size(); i++)
  {
    for(int j = 0; j < (int)V[i].size(); j++)
      printf("%f, ", V[i][j]);
    printf("\n");
  }
  printf("\n");
}


int main(int argc, char * argv[]) 
{    
  // init ROS
  ros::init(argc, argv, "irobot_create_cu");
  controller = new IRobotCreateController();
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
   
  // load globals from parameter server
  double param_input;
  bool bool_input;
  if(ros::param::get("irobot_create_cu/bumper_backup_distance", param_input)) 
    BUMPER_BACKUP_DIST = (float)param_input;                                   //(m) after a bumper hit, the robot backs up this much before going again
  if(ros::param::get("irobot_create_cu/backup_speed", param_input)) 
    BACKUP_SPEED = (float)param_input;
  if(ros::param::get("irobot_create_cu/bumper_theta_offset", param_input)) 
    BUMPER_THETA_OFFSET = (float)param_input;                                  // rad in robot coordinate system
  if(ros::param::get("irobot_create_cu/bumper_distance", param_input)) 
    BUMPER_OFFSET = (float)param_input;                                        // (m) distance in robot coordinate system
  if(ros::param::get("prairiedog/using_tf", bool_input)) 
    using_tf = bool_input;                                                     // when set to true, use the tf package
  
  // print data about parameters
  printf("backup distance: %f, backup_speed: %f, bumper location (distance, theta): [%f %f]\n", BUMPER_BACKUP_DIST, BACKUP_SPEED, BUMPER_OFFSET, BUMPER_THETA_OFFSET);
  
  // wait until the map service is provided (we need its tf /world_cu -> /map_cu to be broadcast)
  ros::service::waitForService("/cu/get_map_cu", -1);
  
  // set up publishers
  odometer_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/odometer_pose_cu", 1);
  if(using_tf)
    bumper_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cu/bumper_pose_cu", 10);
  else
    bumper_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/bumper_pose_cu", 10);
  system_state_pub = nh.advertise<std_msgs::Int32>("/cu/system_state_cu", 10);
  if(multi_robot_mode)  
  {
    target_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/target_pose_cu", 1);
    turn_circle_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/turn_circle_cu", 1);
  }
  // set up subscribers
  user_control_sub = nh.subscribe("/cu/user_control_cu", 1, user_control_callback);
  pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
  goal_sub = nh.subscribe("/cu/goal_cu", 1, goal_callback);
  global_path_sub = nh.subscribe("/cu/global_path_cu", 1, global_path_callback);
  system_update_sub = nh.subscribe("/cu/system_update_cu", 10, system_update_callback);
  user_state_sub = nh.subscribe("/cu/user_state_cu", 1, user_state_callback);
    
  // init pose and local goal
  robot_pose = make_pose(0,0,0);
  local_goal_pose = make_pose(0,0,0);
  global_goal_pose = make_pose(0,0,0);
   
  float backup_start_x = 0, backup_start_y = 0; // remembers where a bumper hit occoured
  while (nh.ok()) 
  {   
    //printf("system state: %d \n", system_state);  
      
    // check for bumper hits
    if (controller->isBumpedLeft() || controller->isBumpedRight()) 
    {        
      setSpeed(BACKUP_SPEED);
      setTurn(0);
          
      backup_start_x = controller->getX();
      backup_start_y = controller->getY();
      backing_up = true;
          
      // there is a bumper hit, so send estimated position of obstacle and back up a little bit  
      publish_bumper(controller->isBumpedLeft(), controller->isBumpedRight());
      
      safe_path_exists = 0; // wait for a new path to come
          
      state_prior_to_bumper_hit = system_state;
      system_state = 2;
    }   
    else if(backing_up)
    { 
      setSpeed(BACKUP_SPEED);
      // already backing up based on a prior bumper hit
      if(system_state != 2)  // this may happen if something changes the state while the robot is already backing up
      {
        state_prior_to_bumper_hit = system_state;
        system_state = 2;     
      }
    }  
    else if(user_state == 1)
    {
      // user is doing a manual stop  
      system_state = 4;    
    }
    else if(user_state == 2)
    {
      // user is doing manual movement  
      system_state = 5;    
    }
    else if(safe_path_exists == 1)
    {
      // we have a path to the goal and are in autonomous mode  
      system_state = 1;
    }
    else if(system_state == 0)
    {
      // still in initial state
    }
    else if(safe_path_exists == 0)
    {
      // there is not a safe path to the goal
      system_state = 3;
    }
    else
    {
      printf("didn't count on this case \n");
    }
    
    
    
//     if(trajectory.size() > 1 && robot_pose!= NULL)
//     {
//       float center_x = 0;
//       float center_y = 0;
//       float radius = 0;
//       find_best_turn_circle(robot_pose, trajectory, 0, current_time_index+1, center_x, center_y, radius);
//       publish_turn_circle(center_x, center_y, radius);
//     }
    
    // printf("user state: %d \n", user_state);
    
    if(system_state == 0) // initial state (havn't recieved enough info to start moving)    
    {
      setSpeed(0);
      setTurn(0);   
    }
    else if(system_state == 1) // planning and moving normally, 
    {
      if(multi_robot_mode)
      {
        //if(within_dist_of_goal(.2) == 0)
          follow_trajectory(trajectory, 0, new_global_path); // last argument should be changed to 1 the first time and 0 after that for speed
        //else
        //{
        //  move_toward_pose(global_goal_pose, .3, PI/12);
          ///printf("here ---\n");

        //}
      }
      else
      { 
        if(within_dist_of_goal(.2) == 0)
          follow_path(global_path, .3, new_global_path);
        else
          move_toward_pose(global_goal_pose, .3, PI/12);
      }
      
      new_global_path = 0;    
    }
    else if(system_state == 2) // bumper hit, backing up
    {
      float current_x = controller->getX();
      float current_y = controller->getY();
          
      float dx = current_x - backup_start_x;
      float dy = current_y - backup_start_y;
          
      if(sqrt(dx*dx + dy*dy) > BUMPER_BACKUP_DIST) // backed up enough after a bumper hit
      {
        setSpeed(0);
        setTurn(0);   
        backing_up = false;
          
        system_state = state_prior_to_bumper_hit;
      }  
    }
    else if(system_state == 3) // no path to goal exists
    {
      //printf("irobot_create_cu: no safe path to goal exists \n"); 
      
      setSpeed(0);
      setTurn(0);
      
      new_global_path = 0;
    }
    else if(system_state == 4) // manual stop
    { 
      setSpeed(0);
      setTurn(0);    
    }
    else if(system_state == 5) // manual control
    {
       // manual navigation handled by callback functions     
        
       //printf("speed: %f, turn: %f \n", speed, turn);      
    }
 
    
    // broadcast odometer based pose
    publish_odometer_pose(controller->getX(), controller->getY(), controller->getRot());
     
    // broadcast system state
    publish_system_state(system_state);
       
    // broadcast target pose
    if(multi_robot_mode && t_target_ind != -1)
      publish_target_pose(trajectory[t_target_ind][0],trajectory[t_target_ind][1], trajectory[t_target_ind][2]);      
    
    ros::spinOnce();
    loop_rate.sleep();
  } // end main control loop
    
  // destroy subscribers and publishers
  odometer_pose_pub.shutdown();
  bumper_pose_pub.shutdown();
  system_state_pub.shutdown();
  user_control_sub.shutdown();  
  target_pose_pub.shutdown();
  turn_circle_pub.shutdown();
  pose_sub.shutdown();
  goal_sub.shutdown();
  global_path_sub.shutdown();
  system_update_sub.shutdown();
  user_state_sub.shutdown(); 
  
  destroy_pose(robot_pose);
  destroy_pose(local_goal_pose);
  destroy_pose(global_goal_pose);
  
  return 0;
}
