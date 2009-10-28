/*  
 *  Copyrights:
 *  Nikos Arechiga 2009
 *  Maciej Stachura, Apretim Shaw, Tony Carfang, Jaeheon Jeong, Sept. 2009
 *  Michael Otte Sept. 2009
 *
 *  This file is part of Stargazer_CU.
 *
 *  Stargazer_CU is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Stargazer_CU is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Stargazer_CU. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *  If you require a different license, contact Michael Otte at
 *  michael.otte@colorado.edu
 *
 * Initialize the robot with proper parameters for the proper environment
 * This involves reading in the global world XML file that represents
 * pseudolite positions, and setting the configuration parameters defined in the configuration xml file. The expected parameters from the config file are as follows:
 *   ThrAlg: To determine how to get ThrVal, Auto or Manual. We are assigning Auto
 *   MarkType: To set up landmark type by use, Home (3x3 pseudolites, 31 ids), or Office (4x4, 4095 IDs)
 *   MarkDim: To set up landmark type by height. HDL2-2 for our case, which is between 1.1 and 2.9 meters, this means MarkDim 1
 *   MapMode: Determine whether map building mode is executed or not. There are start and stop. We are setting up the world map, so default 'Stop'
 *   MarkMode: To determine whether landmarks are used independently under Alone Modoe or dependently under Map Mode, we are fine with default 'Alone'
 *   ThrVal: Threshold level to reject external turbulence, recommended value between 210 and 240. - Not needed for Auto
 *   MarkHeight: Distance from a StarGazer to a landmark, in our case, height of the ceiling from the floor minus the mounted height of the StarGazer - 2140 mm
 *
 *   Created by: Nikos Arechiga 2009
 *   Edited by: Maciej Stachura, Apretim Shaw, Tony Carfang, and JJ, Sept. 2009
 *   Edited by: Michael Otte Sept. 2009 to send messages to a different topic  
 *
 *   Note: this file includes code from tinyxml
 */



#include <stdio.h>
#include "port_setup.h"
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <string.h>
#include "geometry_msgs/Pose2D.h"
#include "../include/pseudo_obj.h"
#include "../include/tinyxml/tinyxml.h"

void process_and_send_data ( char * input_data, ros::Publisher * data_pub, Pseudolite p_data );
geometry_msgs::Pose2D convert2global ( geometry_msgs::Pose2D meas, Pseudolite p_data );
void setup_stargazer( int port, ros::NodeHandle n, ros::Rate loop_rate, const char * command_file, const char * cmd_init_set);

int main ( int argc, char ** argv ) {

  // Set the default values
  const char * pseudo_file = "pseudolites.xml";
  const char * command_file = "command_sets.xml";
  const char * cmd_init_set = "standard_init"; // name for set of initialization commands in xml file.
  const char * stargazer_name = "new_stargazer";
  const char * port_name = "/dev/stargazer";

  char * sensor_data;
	sensor_data = (char *)malloc( 40 * sizeof(char) );
	int i = 0;

  // Read in command line arguments
  for(int in = 1; in < argc; in = in+2){
    if (!strcmp("-n",argv[in]))
      stargazer_name = argv[in+1];
    else if(!strcmp("-c",argv[in]))
      command_file = argv[in+1];
    else if(!strcmp("-s",argv[in]))
      cmd_init_set = argv[in+1];
    else if(!strcmp("-f",argv[in]))
      pseudo_file = argv[in+1];
    else if(!strcmp("-p",argv[in]))
      port_name = argv[in+1];
    else if(!strcmp("--help",argv[in])){
      ROS_INFO("\n\tInput arguments:\n\t-n <name>: name of this ROS service\n\t-c <name>: Stargazer init xml file\n\t-s <name>: name of command set within the command file\n\t-p <name>: the portname of the Stargazer Module\n\t-f <name>: the name of the pseudolite location xml file\n");
      return 0;}
    else
      ROS_INFO("Unknown argument: %s. Try --help next time. Continuing with default settings\n", argv[in]);
  }
//  if (help) {break;}

  // Initialize the ROS service, and the sensor data to be published
	ros::init(argc, argv, stargazer_name);
	ros::NodeHandle n;
	ros::Publisher data_pub = n.advertise<geometry_msgs::Pose2D>("/cu/stargazer_pose_cu",1);
	ros::Rate loop_rate(1000);

  // Load the xml files
  Pseudolite pseudo_1b50(pseudo_file);

  // Setup the serial port
	enum state_counter { waiting_for_STX, reading_data, processing };
	enum state_counter STATE;
	int port = setup_serial_port(port_name);

  // load configuration file to load things up
  setup_stargazer( port, n, loop_rate, command_file, cmd_init_set);
  
	STATE = waiting_for_STX;
	while ( n.ok() ) {
		switch ( STATE ) {

			case waiting_for_STX:
				sensor_data[0] = '\0';
				while ( sensor_data[0] != '~' ) {
					read ( port, sensor_data, 1 );
					loop_rate.sleep();
				}
				STATE = reading_data;
				break;

			case reading_data:
				i = 0;
				while ( sensor_data[i] != '`' ) {
					++i;
					read ( port, sensor_data + i, 1 );
				}
				++i;
				if ( read( port, sensor_data + i, 1) == '~' ) { STATE = reading_data; }
				else { STATE = processing; }
				break;

			case processing:
				++i;
				sensor_data[i] = '\0';
				process_and_send_data ( sensor_data, &data_pub, pseudo_1b50 );
				STATE = waiting_for_STX;
				break;

			default:
				ROS_INFO("onoz!: System has reached an unknown state\n");
				break;
			}
	}
	free( sensor_data );

}


void process_and_send_data ( char * input_data, ros::Publisher * data_pub, Pseudolite p_data) {

	char mode;
	float angle, x, y;
	int ID;
	geometry_msgs::Pose2D meas; // The position measurement to be published

	// Read in the data from the serial port.

	if ( strcmp( input_data, "~DeadZone`" ) == 0 )
	{
		// could not detect position
		ROS_INFO("...dead zone...\n");
	}else {
		sscanf(input_data, "~^%c%i|%f|%f|%f`", &mode, &ID, &angle, &x, &y );

		// Convert the measurement to radians and metres.
		meas.theta = angle*M_PI/180;
		meas.x = x/100;
		meas.y = y/100;
        
		// Find the pseudolite data based on the ID. Implement this with a hash table.
    if (p_data.getPseudoliteById(ID)){

      // Convert to the global coordinate system.
      meas = convert2global(meas, p_data);

      // Keep the angles in [0, 2*pi].
		  while ( meas.theta < 0 ) meas.theta += 2*M_PI;
		  while ( meas.theta >= 2*M_PI ) meas.theta -= 2*M_PI;
	
		  // Output the data
		  ROS_INFO("Mode: %c, ID: %i, x: %f, y: %f, Angle: %f", mode, ID, meas.x*100, meas.y*100, meas.theta*180/M_PI);
		  data_pub->publish(meas);
    }else{ // If the measured Pseudolite ID was not found.
      //ROS_INFO("Error: Measured ID not in list of candidate Pseudo-lites: \n %c, ID: %i, x: %f, y: %f, Angle: %f", mode, ID, meas.x*100, meas.y*100, meas.theta*180/M_PI);
    }
	}

}
	
geometry_msgs::Pose2D convert2global ( geometry_msgs::Pose2D meas, Pseudolite p_data ) {
	// This function takes in the measurement and pseudolite and converts to global coordinates
	geometry_msgs::Pose2D meas_g;

	// Transform from the pseudolites local coordinate frame to the global
	meas_g.theta = meas.theta + p_data.angle;	
	meas_g.x = meas.x*cos(p_data.angle) + meas.y*sin(p_data.angle) + p_data.x;
	meas_g.y = meas.y*cos(p_data.angle) - meas.x*sin(p_data.angle) + p_data.y;
	return meas_g;
}



/*
 * Set up the stargazer with parameters read in
 * from a config file currently hard-coded below
 * and perhaps later read in from command prompt
 */
void setup_stargazer( int port, ros::NodeHandle n, ros::Rate loop_rate, const char * command_file, const char * cmd_init_set)
{
	// ****** Settings specific to pseudolite configuration **********************
	// ****** You may need to edit these based on pseudolite version *************
	// ****** or startup-command set *********************************************
	TiXmlDocument doc( command_file ); // config file directory
//	const char * cmd_init_set = "standard_init"; // name for set of initialization commands in xml file.
	const char * cmd_start = "~#"; // Special symbols for pseudolite as a start command sequence
	const char * cmd_end = "`"; // special end command sequence

	// states for interacting with the hagisonic board
	enum state_counter { waiting_for_STX, reading_data, send_cmd };
	// message to send, received message, xml command, xml corresponding value
	char * send_msg, * rec_msg;
  	const char * command, * value; 
	send_msg = (char *)malloc( 40 * sizeof(char) );
	rec_msg = (char *)malloc( 40 * sizeof(char) );
	int i; // index variable, used mainly for managing  strings
	char check_cmd, check_value;
        
  	if (!doc.LoadFile()) // if we couldn't successfully load the config file:
	{
		ROS_INFO("Stargazer config settings file could not be loaded. %s We kind of need that for everything to work...\n", doc.ErrorDesc());
		return;
	}

  	ROS_INFO("Stargazer config settings file was loaded successfully.\n");

	// ***************** Methodology ***************************************
	// Find proper set of commands to issue, from xml file
	// Send CalcStop command to prepare the system to take some parameters.
	// Read in commands from xml 1 at a time and send them out
	// Send CalcStart to let system resume
	// ************************************************************************************
	
	// Stuff for xml config file
	TiXmlElement *parent = doc.RootElement();
  	TiXmlElement *cmd_category = 0;
  	TiXmlElement *cmd_child = 0;
  	const char *cmd_set = "\0"; 
	
	// start communicating with board
	//ROS_INFO("Initializing parameters for our world");
	enum state_counter STATE;

	// Start off in waiting for the node, to ensure we have communication.
	// This means that the first message will be a localization calculation.
	STATE = waiting_for_STX;
	
	// iterate through the command categories in the xml file, since there could be multiple sets of command categories.
	// The one we are looking for is stored in cmd_init_set
	do{
		cmd_category = (TiXmlElement *)(parent->IterateChildren( cmd_category ) );
		if (cmd_category){
			ROS_INFO("cmd_category = %s", cmd_category);
			if(!cmd_category->Attribute("title") ) {
				cmdset = "\0";
			}
			else cmdset = cmd_category->Attribute("title");
		}
	} while ( strcmp( cmdset, cmd_init_set ) != 0 && cmd_category);
	// TODO: need to add check to ensure we're not at the end of the file, otherwise we get a null cmd_category at some point

	// iterate through sending commands, almost all of which come with its own response
	// Make sure the node is ok to talk, and while there are still commands to send
	while ( n.ok() && ( cmd_child = (TiXmlElement *)(cmd_category->IterateChildren( cmd_child )) ) )
	{
		switch ( STATE )
		{
			case waiting_for_STX:
				// Waiting for response from card
				strcpy( rec_msg, '\0');
				// looking for a ~ to indicate the start of a response, as set at the top of this function
				while ( strcmp( rec_msg, cmd_start ) != 0 ) {
					read ( port, rec_msg, 1 );
					loop_rate.sleep();
				}
				// once a response is on its way, read it
				STATE = reading_data;
				break;

			case reading_data:
				i = 0;
				// read the incoming response. The response
				// ends with a ` character as set at the top of the function
				while ( strcmp(rec_msg, cmd_end) != 0 ) {
					++i;
					if ( read ( port, rec_msg + i, 1 ) )
        				{
						// Only want to read one response at a time
						// as part of initializing parameters
						// so go to next state
						STATE = send_cmd; 
					}
					else {
                        ROS_INFO("Uh oh, couldn't read properly. Trying again");
                    }
				}

	      // Check if the received data is a parameter response, starting with ~!
	      sscanf(rec_msg, "~!%s|%s`", &check_cmd, &check_value);
	      if ( strcmp(rec_msg, send_msg) ) // Make sure the sent command is same as received
	      	ROS_INFO("Correctly set %s to %s", &check_cmd, &check_value);
	      else ROS_INFO("Warning: Different commands! Sent %s, received %s", send_msg, rec_msg); 

				break;

			case send_cmd:
				// get command and value from xml
				command = cmd_child->Attribute("command");
				value = cmd_child->Attribute("value");
				
				// format the command message as "cmd_start command [|value] cmd_end" without spaces, where |value is optional
				strcpy(send_msg, cmd_start);
				strcat(send_msg, command); 
				if ( strcmp(value,"") != 0 ) // if there is a value to append, unlike CalcStop, etc...
				{	// add | and value
					strcat(send_msg, "|");
					strcat(send_msg, value);
				}
				strcat(send_msg, cmd_end);
				ROS_INFO("Setup-Stargazer: Sending command %s\n",send_msg);

				i=-1;
				do {
					++i;
					// send the command through the serial port
					if ( ! write ( port, send_msg + i, 1) )
            					ROS_INFO("Uh Oh, couldn't send command");
				} while ( strcmp( send_msg, cmd_end) != 0);

				STATE = waiting_for_STX;
				break;

			default:
				ROS_INFO("onoz!: System has reached an unknown state\n");
				break;
		}
	}

	free( send_msg );
	free( rec_msg );
	// Consts don't need to be freed? Lines below cause compile-time errors if uncommented
	// free( cmd_init_set );
	// free( cmd_start );
	// free( cmd_end );
	// free( command );
	// free( value );
}

