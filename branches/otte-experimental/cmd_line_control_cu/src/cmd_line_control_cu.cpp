/*  Copyright Michael Otte, University of Colorado, 4-21-2009
 *
 *  This file is part of cmd_line_control_cu.
 *
 *  cmd_line_control_cu is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  cmd_line_control_cu is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with cmd_line_control_cu. If not, see <http://www.gnu.org/licenses/>.
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





#include <assert.h>
#include <curses.h>
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



#include <ros/ros.h>

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point32.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GridCells.h"

#include "std_msgs/Int32.h"

#include <list>

// global ROS publisher handles
ros::Publisher user_control_pub;
ros::Publisher user_state_pub;
        
// globals used to set user robot control
double change_speed = 0; // 0 = no, -1 = decrease speed, 1 = increase speed
double change_turn = 0; // 0 = no, -1 = decrease turn, 1 = increase turn

int user_control_state = 0; // as broadcast by this node
                            // 0 = passive, doing nothing
                            // 1 = manual stop
                            // 2 = manual control


/*----------------------- ROS Publisher Functions -----------------------*/
void publish_user_control(int x, int theta)
{
  geometry_msgs::Pose2D msg;
  msg.x = x;
  msg.theta = theta;
  user_control_pub.publish(msg); 
}

void publish_user_state(int u_state)
{
  std_msgs::Int32 msg;  
  
  msg.data = u_state;
  user_state_pub.publish(msg); 
}

int main(int argc, char *argv[])
{   
  // init ROS stuff  
  ros::init(argc, argv, "cmd_line_control_cu");  
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
 
  // set up ROS topic publishers
  user_control_pub = nh.advertise<geometry_msgs::Pose2D>("/cu/user_control_cu", 1);
  user_state_pub = nh.advertise<std_msgs::Int32>("/cu/user_state_cu", 1);
  

  // init curses
  initscr();
  //keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  //nonl();
  cbreak();
  noecho();

  int cmdchar;

  while (ros::ok()) 
  {
    change_turn = 0;
    change_speed = 0;
    cmdchar = getch();

    if(cmdchar != ERR) 
    {
      if(cmdchar == 121 || cmdchar == 110 || cmdchar == 103 || cmdchar == 106)
      {
        if(cmdchar == 121)      // y, forward
        {
          change_speed = 1;
          printf("\n/\\ \n");
        }  
        else if(cmdchar == 110) // n, reverse
        {
          change_speed = -1;
          printf("\n\\/ \n");
        }
        else if(cmdchar == 103) // g, turn left
        {
          change_turn = 1;
          printf("\n<--\n");
        }
        else if(cmdchar == 106) // j, turn right
        {
          change_turn = -1;
          printf("\n-->\n");
        }

        publish_user_control(change_speed, change_turn);
        user_control_state = 2;
        publish_user_state(user_control_state);  
      }
      else if(cmdchar == 104) // j, stop
      {        
        change_turn = 0;
        change_speed = 0;
        publish_user_control(change_speed, change_turn);
        user_control_state = 1;
        publish_user_state(user_control_state);
        printf("\n-X-\n");
      }
      else if(cmdchar == 97) // a, auto-mode
      {
        user_control_state = 0;
        publish_user_state(user_control_state);  
        printf("\n-A-\n");
      }
    } 
    fflush(stdout);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // exit curses
  endwin();

  user_control_pub.shutdown();
  user_state_pub.shutdown();
 
}

    
