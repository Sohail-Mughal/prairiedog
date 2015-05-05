/*  
 * stargazer_cu_node.cpp
 */
#include <ros/ros.h>
//#include <stargezer.h>

int main ( int argc, char ** argv ) 
{
	ros::init(argc, argv, "stargazer_node");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(1000);
}

