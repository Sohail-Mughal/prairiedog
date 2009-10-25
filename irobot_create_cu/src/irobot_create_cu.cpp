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

#include <irobot_create/irobot_create_controller.h>
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"

#include "nav_msgs/Path.h"

#ifndef PI
  #define PI 3.1415926535897
#endif
      
#define BUMPER_BACKUP_DIST .015 //(m) after a bumper hit, the robot backs up this much before going again
#define BACKUP_SPEED -.2
#define BUMPER_THETA_OFFSET PI/6  // rad in robot coordinate system
#define BUMPER_OFFSET .25  // (m) distance in robot coordinate system
        
using namespace std;

struct POSE;
typedef struct POSE POSE;
        
// set up Brown's Controller
IRobotCreateController * controller = (IRobotCreateController*) NULL;

// publisher handles
ros::Publisher odometer_pose_pub;
ros::Publisher bumper_pose_pub;

// subscriber handles
ros::Subscriber user_control_sub;
ros::Subscriber pose_sub;
ros::Subscriber goal_sub; // global goal
ros::Subscriber global_path_sub;

// globals for defining robot velocity
float speed = 0;
float turn = 0;

float max_speed = 1;
float max_turn = 1;

float use_input_speed_increment = .2;
float use_input_turn_increment = .2;

int user_e_stop = 0;

int new_global_path = 1;

// globals for robot and goal
POSE* robot_pose = NULL;
POSE* local_goal_pose = NULL;
POSE* global_goal_pose = NULL;

vector<POSE> global_path;

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

/*------------------------ ROS Callbacks --------------------------------*/
void user_control_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    
  if(msg->y > 0)
  {
    speed += use_input_speed_increment;
    if(speed > max_speed)
        speed = max_speed;
  }
  else if(msg->y < 0)
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

  if(msg->y == 0 && msg->theta == 0)
  {
    if(user_e_stop == 0)
      user_e_stop = 1;
    else
      user_e_stop = 0;   
  }
      
  //ROS_INFO_STREAM("speed: " << speed << "turn: " << turn );
  controller->setSpeeds(speed, turn);
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
  } 
  new_global_path = 1;
}

/*------------------------- ROS publisher functions ---------------------*/

void publish_bumper(bool left, bool right)
{
  float theta;  
  if(left && right)
    theta = 0;
  else if(left)
    theta = BUMPER_THETA_OFFSET;
  else
    theta = -BUMPER_THETA_OFFSET;
  
  // transform 1, from range and degree in robot local to x and y in robot local coordinate frame
  float y = BUMPER_OFFSET*sin(theta);
  float x = BUMPER_OFFSET*cos(theta);
  
  // transform 2 from local x, y to global x, y     
  geometry_msgs::Pose2D msg;
  msg.x = x*robot_pose->cos_alpha - y*robot_pose->sin_alpha + robot_pose->x;
  msg.y = x*robot_pose->sin_alpha + y*robot_pose->cos_alpha + robot_pose->y;
  
  bumper_pose_pub.publish(msg); 
}



/* ---------------------- navigation functions --------------------------*/


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
     setTurn(.2);
  else if(diff_direction < -tolerance)
     setTurn(-.2);
  else
  {
     setTurn(0);
     return 1;
  }
  return 0;     
}

// same as above, but will also drive forward when robot is within tolerance_fast radians of the correct direction
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
     setSpeed(.2);
  else if(diff_direction <= 0 && diff_direction >= -tolerance_fast)
     setSpeed(.2);
  else
     setSpeed(0);
  
  if(diff_direction > tolerance)
     setTurn(.2);
  else if(diff_direction < -tolerance)
     setTurn(-.2);
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
     setTurn(.2);
  else if(diff_direction < -tolerance)
     setTurn(-.2);
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
      setSpeed(.2);
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

// same as above, except that if the robot is within tolerance_radians_fast of desired heading, then it moves forward
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
    next_p_index ++;
      
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


int main(int argc, char * argv[]) 
{    
    // init ROS
    ros::init(argc, argv, "irobot_create_cu");
    controller = new IRobotCreateController();
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    
    // set up publishers
    odometer_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/odometer_pose_cu", 1);
    bumper_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/bumper_pose_cu", 10);
    
    // set up subscribers
    user_control_sub = nh.subscribe("/cu/user_control_cu", 1, user_control_callback);
    pose_sub = nh.subscribe("/cu/pose_cu", 1, pose_callback);
    goal_sub = nh.subscribe("/cu/goal_cu", 1, goal_callback);
    global_path_sub = nh.subscribe("/cu/global_path_cu", 1, global_path_callback);
   
    // init pose and local goal
    robot_pose = make_pose(0,0,0);
    local_goal_pose = make_pose(0,0,0);
    global_goal_pose = make_pose(0,0,0);
   
    float backup_start_x = 0, backup_start_y = 0; // remembers where a bumper hit occoured
    bool backing_up = false; // set to true if robot gets a bumper hit
    while (nh.ok()) 
    {   
        // control robot
        if(backing_up)
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
          }
        }
        else if(user_e_stop == 0)
        {
          if(within_dist_of_goal(.2) == 0)
            follow_path(global_path, .3, new_global_path);
          else
            move_toward_pose(global_goal_pose, .3, PI/12);
           
          new_global_path = 0;
        }
        
        if (controller->isBumpedLeft() || controller->isBumpedRight()) 
        {
          // there is a bumper hit, so send estimated position of obstacle and back up a little bit  
            
          publish_bumper(controller->isBumpedLeft(), controller->isBumpedRight());
            
          setSpeed(BACKUP_SPEED);
          setTurn(0);
          
          backup_start_x = controller->getX();
          backup_start_y = controller->getY();
          backing_up = true;
        }
        
        // broadcast odometer based pose
        geometry_msgs::Pose2D msg;
        msg.x = controller->getX();
        msg.y = controller->getY();
        msg.theta = controller->getRot();
        odometer_pose_pub.publish(msg);   
  
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // destroy subscribers and publishers
    odometer_pose_pub.shutdown();
    bumper_pose_pub.shutdown();
    user_control_sub.shutdown();  
    pose_sub.shutdown();
    goal_sub.shutdown();
    global_path_sub.shutdown();
    
    destroy_pose(robot_pose);
    destroy_pose(local_goal_pose);
    destroy_pose(global_goal_pose);
    
    return 0;
}
