#include <stdio.h>
#include "port_setup.h"
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Int8.h>
#include <string.h>
#include "geometry_msgs/Pose2D.h"
#include "stargazer_cu/Pose2DTagged.h"
#include "../include/pseudo_obj.h"
#include "../include/tinyxml/tinyxml.h"
#include <iostream>

void setup_stargazer(const char * command_file);

int main ( int argc, char ** argv ) {

  // Set the default values
  const char * command_file = "/home/user/ros_ws/indigo/catkin_ws/src/prairiedog/stargazer_cu/example1.xml";

	ros::init(argc, argv, "stargazer_name");
	ros::NodeHandle n;

    // load configuration file to load things up
    setup_stargazer(command_file);
    return 0;
}
/*
 * Set up the stargazer with parameters read in
 * from a config file currently hard-coded below
 * and perhaps later read in from command prompt
 */
void setup_stargazer(const char * command_file)
{
	// ****** Settings specific to pseudolite configuration **********************
	// ****** You may need to edit these based on pseudolite version *************
	// ****** or startup-command set *********************************************
	TiXmlDocument doc( command_file ); // config file directory
        
  if (!doc.LoadFile()) // if we couldn't successfully load the config file:
	{
		ROS_INFO("Pseudolite config settings file could not be loaded. Hopefully stuff works anyway...\n");
		return; 

	}

	
	ROS_INFO("Pseudolite config settings file was loaded successfully.\n");
	const char * cmd_init_set = "standard_init"; // name for set of initialization commands in xml file.
	const char * cmd_start = "~#"; // Special symbols for pseudolite as a start command sequence
	const char * cmd_end = "`"; // special end command sequence
	const char * response_start = "~";
	TiXmlElement *parent = doc.RootElement();
	TiXmlElement *cmd_category = NULL;
	TiXmlElement *cmd_child = NULL;

	printf("step1\n");
	if(parent==NULL)
		return;
	const char* test;
	do
	{
		cmd_category = (TiXmlElement *)(parent->IterateChildren( cmd_category ) );

		if(cmd_category!=NULL)
			printf("step2\n");
		test = cmd_category->Attribute("title");
		printf("category name: %s \n", test);
	} while ( strcmp( test, cmd_init_set ) != 0 );

	cmd_child = (TiXmlElement *)(cmd_category->IterateChildren( cmd_child ));

	const char *command, *value;
	printf("step3\n");

	
	char * send_msg, * rec_msg; 
	send_msg = (char *)malloc( 40 * sizeof(char) );
	rec_msg = (char *)malloc( 40 * sizeof(char) );
	int i=0; // index variable, used mainly for managing  strings
    
    command = "CalcStop";
	strcpy(send_msg, cmd_start);
	strcat(send_msg, command); 
	strcat(send_msg, cmd_end);
	 do {
		    ROS_INFO("Setup-Stargazer: Sending command %s\n",send_msg);
			i++;
			if(!strcmp(send_msg, cmd_end)) break;
		} while (1);

		return;
}

