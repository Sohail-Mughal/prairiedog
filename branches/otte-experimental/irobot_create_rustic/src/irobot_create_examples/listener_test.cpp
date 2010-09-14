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

	ros::init(argc, argv, "listener_test");
	// initialize ROS
	ros::NodeHandle n;
	// create a handle to this process node	
	IRobotCreateController controller = IRobotCreateController();
	// create an instance of controller
	
	usleep(5*1000000);
	// allow for a 5 second break
	
	ros::spin();
	
	/*
	signal(SIGINT,stopper);
	while(!endnow)//n.ok())
	{
			
		if(controller.isBumpedLeft())
		{
			ROS_INFO("Left Bump!!");
		} else if (controller.isBumpedRight()) {
			ROS_INFO("Right Bump!!");
		}
		
		ros::spinOnce();
	}*/
	
	return(0);
}
