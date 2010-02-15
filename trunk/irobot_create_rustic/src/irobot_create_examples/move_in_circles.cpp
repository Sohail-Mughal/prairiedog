/*
 *  move_in_circles.cpp
 *  
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/15/09.
 *
 */

//#include "move_in_circles.h"

#include <ros/ros.h>
#include <create_comms.h>
#include <irobot_create/Speeds.h>
#include <irobot_create/irobot_create_controller.h>

int main(int argc, char** argv) 
{
	ros::init(argc, argv,"move_in_circles");
	ros::NodeHandle n;
	printf("hi!\n");
	
	IRobotCreateController controller = IRobotCreateController();
	
	usleep(5*1000000);
	
	controller.setSpeeds(1.0, 2.0);
	
	double forward = 1.0;
	double rotate = 3.0;
	
	controller.setSpeeds(forward, rotate);
	controller.setSpeeds(forward, rotate);
	
	usleep(5*1000000);
	
	forward = 0.0;
	rotate = 0.0;
	
	controller.setSpeeds(forward, rotate);
}