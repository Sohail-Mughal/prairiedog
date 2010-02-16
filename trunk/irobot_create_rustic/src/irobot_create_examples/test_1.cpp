/*
 *  client.cpp
 *  
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/16/09.
 *
 *  Modified by Michael Otte University of Colorado at Boulder 2010 for irobot_create_rustic 
 *
 */

#include <ros/ros.h>
#include <irobot_create_rustic/irobot_create_controller.h>
#include <signal.h>


bool endnow = false;
void stopper(int x)
{
	endnow = true;
}

int main(int argc, char** argv) 
{

	ros::init(argc, argv, "test_1");
	// initialize ROS
	ros::NodeHandle n;
	// create a handle to this process node	
	IRobotCreateController controller = IRobotCreateController();
	// create an instance of controller
	
	usleep(5*1000000);
	// allow for a 5 second break
	
	double forward;
	double rotate;
	
	signal(SIGINT,stopper);
	while(!endnow)//n.ok())
	{
		//printf("in Loop\n");
		
		forward = 0.5;
		rotate = 0.0;
		// set the speed for forward and rotating motion
			
		if(controller.isBumpedLeft())
		{
			printf("leftBump\n");
			forward = -0.1;
			rotate = 1.5;
		} else if (controller.isBumpedRight()) {
			printf("rightBump\n");
			forward = -0.1;
			rotate = - 1.5;
		}
		
		controller.setSpeeds(forward, rotate);
		// pass in the values & display
		
		ros::spinOnce();
	}
	printf("out of loop\n");
	
	
	// stop the create from moving
	forward = 0.0;
	rotate = 0.0;
	controller.setSpeeds(forward, rotate);
	
	printf("done\n");
	
	
	return(0);
}
