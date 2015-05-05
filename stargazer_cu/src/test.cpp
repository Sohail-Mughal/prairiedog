#include <stdio.h>
#include <serial_communication.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Int8.h>
#include <string.h>
#include "geometry_msgs/Pose2D.h"
#include "stargazer_cu/Pose2DTagged.h"
#include "../include/pseudo_obj.h"
#include "../include/tinyxml/tinyxml.h"
#include <time.h>
#include <iostream>
#include <string>

bool kill = false;

void process_and_send_data ( char * input_data, ros::Publisher * data_pub, ros::Publisher * marker_pub, Pseudolite p_data );
geometry_msgs::Pose2D convert2global ( geometry_msgs::Pose2D meas, Pseudolite p_data );
void setup_stargazer( int port, ros::NodeHandle n, ros::Rate loop_rate, const char * command_file);
void kill_stargazer(const std_msgs::Int8::ConstPtr& msg);

void sleepcp(int milliseconds);

void sleepcp(int milliseconds) // cross-platform sleep function
{
    clock_t time_end;
    time_end = clock() + milliseconds * CLOCKS_PER_SEC/1000;
    while (clock() < time_end)
    {
    }
}

int main ( int argc, char ** argv ) {

  // Set the default values
  const char * pseudo_file = "/home/user/ros_ws/indigo/catkin_ws/src/prairiedog/stargazer_cu/pseudolites.xml";
  const char * command_file = "/home/user/ros_ws/indigo/catkin_ws/src/prairiedog/stargazer_cu/example2.xml";
  const char * stargazer_name = "new_stargazer";
  const char * port_name = "/dev/stargazer";

  char * sensor_data;
	sensor_data = (char *)malloc( 256 * sizeof(char) );
	int i = 0;

  // Read in command line arguments
   /*
  for(int in = 1; in < argc; in = in+2){
    if (!strcmp("-n",argv[in]))
      stargazer_name = argv[in+1];
    else if(!strcmp("-c",argv[in]))
      command_file = argv[in+1];
    else if(!strcmp("-f",argv[in]))
      pseudo_file = argv[in+1];
    else if(!strcmp("-p",argv[in]))
      port_name = argv[in+1];
    else if(!strcmp("--help",argv[in])){
      ROS_INFO("\n\tInput arguments:\n\t-n <name>: name of this ROS service\n\t-c <name>: Stargazer init xml file\n\t-p <name>: the portname of the Stargazer Module\n\t-f <name>: the name of the pseudolite location xml file\n");
      return 0;}
    else
      ROS_INFO("Unknown argument: %s\n", argv[in]);
  }*/
//  if (help) {break;}

  // Initialize the ROS service, and the sensor data to be published
	ros::init(argc, argv, stargazer_name);
	ros::NodeHandle n;
	ros::Publisher data_pub = n.advertise<geometry_msgs::Pose2D>("/cu/stargazer_pose_cu",1);
	ros::Publisher marker_pub = n.advertise<stargazer_cu::Pose2DTagged>("/cu/stargazer_marker_cu",1);
        ros::Subscriber kill_sub = n.subscribe("/cu/killsg", 1, kill_stargazer);
	ros::Rate loop_rate(1000);

  	// Load the xml files
  	Pseudolite pseudo_1b50(pseudo_file);

  	// Setup the serial port
	enum state_counter { waiting_for_STX, reading_data, processing };
	enum state_counter STATE;
	int port = 0;
	//int port = setup_serial_port(port_name);

  	// load configuration file to load things up
  	setup_stargazer( port, n, loop_rate, command_file );
  
    /*
    STATE = waiting_for_STX;

	while ( n.ok() && !kill) {
		// waiting for STX
		ROS_INFO("Waiting for data."); 	
		memset(sensor_data, '\0', sizeof(sensor_data));
		while (n.ok()) {
			read ( port, sensor_data, 1 );
			printf("[%s]", sensor_data);
			if(strcmp(sensor_data, "~")) break;
			loop_rate.sleep();
		}
		// reading data:
		i = 0;
		while ( sensor_data[i] != '`' ) {
			++i;
			read ( port, sensor_data + i, 1 );
		}
		
		// processing:
		++i;
		sensor_data[i] = '\0';
		process_and_send_data ( sensor_data, &data_pub, &marker_pub, pseudo_1b50 );
		ros::spinOnce();
		
	}
	*/
	free( sensor_data );
}

void kill_stargazer(const std_msgs::Int8::ConstPtr& msg)
{
	if (msg->data == 1) kill = true;
        ROS_INFO("Kill Received");
}

void process_and_send_data ( char * input_data, ros::Publisher * data_pub, ros::Publisher * marker_pub, Pseudolite p_data) {

	ROS_INFO("Processing and publishing data.");
	char mode;
	float angle, x, y;
	int ID;
	int scanreturn;
	geometry_msgs::Pose2D meas; // The position measurement to be published
	stargazer_cu::Pose2DTagged marker; // The (relative?) position measurement of a marker

	// Read in the data from the serial port.

	if ( strcmp( input_data, "~*DeadZone`" ) == 0 )
	{
		// could not detect position
		ROS_INFO("...dead zone...\n");
	}else {
		scanreturn = sscanf(input_data, "~^%c%i|%f|%f|%f`", &mode, &ID, &angle, &x, &y );
		//ROS_INFO("%d, %s\n", scanreturn, input_data);

		// Convert the measurement to radians and metres.
		meas.theta = angle*M_PI/180;
		meas.x = x/100;
		meas.y = y/100;
        
        marker.x = meas.x;
        marker.y = meas.y;
        marker.theta = meas.theta;
        marker.tag = ID;
        marker_pub->publish(marker);
		// Find the pseudolite data based on the ID. Implement this with a hash table.
    if (p_data.getPseudoliteById(ID)){

      // Convert to the global coordinate system.
      meas = convert2global(meas, p_data);

      // Keep the angles in [0, 2*pi].
		  while ( meas.theta < 0 ) meas.theta += 2*M_PI;
		  while ( meas.theta >= 2*M_PI ) meas.theta -= 2*M_PI;
	
		  // Output the data
		  //ROS_INFO("Mode: %c, ID: %i, x: %f, y: %f, Angle: %f", mode, ID, meas.x*100, meas.y*100, meas.theta*180/M_PI);
		  data_pub->publish(meas);
    }else{ // If the measured Pseudolite ID was not found.
      //ROS_INFO("Error: Measured ID not in list of candidate Pseudo-lites: \n %c, ID: %i, x: %f, y: %f, Angle: %f", mode, ID, meas.x*100, meas.y*100, meas.theta*180/M_PI);
    }
	}

}
	
geometry_msgs::Pose2D convert2global ( geometry_msgs::Pose2D meas, Pseudolite p_data ) {
	// This function takes in the measurement and pseudolite and converts to global coordinates
	geometry_msgs::Pose2D meas_g;
	float pth, px, py;

	// Transform from the pseudolites local coordinate frame to the global
	meas_g.theta = meas.theta + p_data.angle;	
	meas_g.x = meas.x*cos(p_data.angle) + meas.y*sin(p_data.angle) + p_data.x;
	meas_g.y = meas.y*cos(p_data.angle) - meas.x*sin(p_data.angle) + p_data.y;

	//pth = meas_g.theta - meas.theta;
	//px = meas_g.x - meas.x*cos(pth) - meas.y*sin(pth);
	//py = meas_g.y - meas.y*cos(pth) + meas.x*sin(pth);
	//ROS_INFO("Tag is at: X = %f, Y = %f, TH = %f", px, py, pth);
	return meas_g;
}



/*
 * Set up the stargazer with parameters read in
 * from a config file currently hard-coded below
 * and perhaps later read in from command prompt
 */
void setup_stargazer( int port, ros::NodeHandle n, ros::Rate loop_rate, const char * command_file)
{
	// ****** Settings specific to pseudolite configuration **********************
	// ****** You may need to edit these based on pseudolite version *************
	// ****** or startup-command set *********************************************
	TiXmlDocument doc( command_file ); // config file directory
	const char * cmd_init_set = "standard_init"; // name for set of initialization commands in xml file.
	const char * cmd_start = "~#"; // Special symbols for pseudolite as a start command sequence
	const char * cmd_end = "`"; // special end command sequence
	const char * response_start = "~";
	std::string port_name("/dev/stargazer");
	SerialCommuniucation serial_comm(port_name);

	// states for interacting with the hagisonic board
	enum state_counter { waiting_for_STX, reading_data, send_cmd };
	// message to send, received message, xml command, xml corresponding value
	char * send_msg, * rec_msg;
	char *temp = NULL;
  	const char * command, * value;
  	temp = (char *)malloc( 2 * sizeof(char) );
	send_msg = (char *)malloc( 40 * sizeof(char) );
	rec_msg = (char *)malloc( 40 * sizeof(char) );
	int i=0; // index variable, used mainly for managing  strings
	char check_cmd, check_value;
        
  if (!doc.LoadFile()) // if we couldn't successfully load the config file:
	{
		ROS_INFO("Pseudolite config settings file could not be loaded. Hopefully stuff works anyway...\n");
		return; 

	}
	
	ROS_INFO("Pseudolite config settings file was loaded successfully.\n");

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
	
	// start communicating with board
	//ROS_INFO("Initializing parameters for our world");
	enum state_counter STATE;

	// Start off in waiting for the node, to ensure we have communication.
	// This means that the first message will be a localization calculation.
	STATE = send_cmd;
	
	// iterate through the command categories in the xml file, since there could be multiple sets of command categories.
	// The one we are looking for is stored in cmd_init_set
	ROS_INFO("Loading init command configuration.");
	do
	{
		cmd_category = (TiXmlElement *)(parent->IterateChildren( cmd_category ) );
	} while ( strcmp( (cmd_category->Attribute("title")), cmd_init_set ) != 0 );

	ROS_INFO("Setup stargazer started.");
	// iterate through sending commands, almost all of which come with its own response
	// Make sure the node is ok to talk, and while there are still commands to send
	bool is_loop_continue = true;

	if(!( cmd_child = (TiXmlElement *)(cmd_category->IterateChildren( cmd_child ))))
		is_loop_continue = false;

	while (is_loop_continue )
	{
		switch ( STATE )
		{
			case waiting_for_STX:
			    ROS_INFO("waiting_for_STX");
			    
				// Waiting for response from card
				//strcpy( rec_msg, "\0");
				memset(rec_msg, '\0', sizeof(rec_msg));
				
				// looking for a ~ to indicate the start of a response, as set at the top of this function
				while (n.ok()) {
					//read ( port, rec_msg, 1 );
					serial_comm.readSerial(rec_msg);
					//printf("[%s]\n", rec_msg);
					if(strcmp(rec_msg, response_start)) break;
					loop_rate.sleep();
				}
				
				STATE = reading_data;
				break;

			case reading_data:
				ROS_INFO("State:reading_data.");
				i = 0;
				// read the incoming response. The response
				// ends with a ` character as set at the top of the function
				
				while ( strcmp(rec_msg+i, cmd_end) != 0 ) {
					i++;
					//if ( read ( port, rec_msg+i, 1 ) )
					  if (serial_comm.readSerial(rec_msg+i))
        				{
						// Only want to read one response at a time
						// as part of initializing parameters
						// so go to next state
							//printf("[%s]:", rec_msg);
							STATE = send_cmd;
					}
					else {
                        ROS_INFO("Uh oh, couldn't read properly. Trying again");
                    }

				}
				//printf("\n");
				i++;
				strcpy(rec_msg+i, "\0");
				ROS_INFO("Response:[%s]", rec_msg);

		  		if(!( cmd_child = (TiXmlElement *)(cmd_category->IterateChildren( cmd_child ))))
		  			is_loop_continue = false;

	      		// Check if the received data is a parameter response, starting with ~!

	      		if ( strcmp(rec_msg, send_msg) ) // Make sure the sent command is same as received
	      			ROS_INFO("Correctly set %s to %s", send_msg, rec_msg);
	      		else 
	      			ROS_INFO("Warning: Different commands! Sent %s, received %s", send_msg, rec_msg);
	            sleep(1);
			break;

			case send_cmd:
			    ROS_INFO("State:send_cmd");
			    memset(send_msg, '\0', sizeof(send_msg));
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

				ROS_INFO("Command to send:[%s].",send_msg);
				i=0;
				do {  
            		if ( ! serial_comm.writeSerial(send_msg+i) )
            			ROS_INFO("Uh Oh, couldn't send command");
					if(!strcmp(temp, cmd_end)) break;
            		i++;
            		temp = send_msg+i;
            		sleepcp(60);
				} while (n.ok());
				STATE = waiting_for_STX;
				break;

			default:
			    printf("default\n");
				ROS_INFO("onoz!: System has reached an unknown state\n");
				break;
		}
	}

	free( send_msg );
	free( rec_msg );
	serial_comm.closeSerial();
	// Consts don't need to be freed? Lines below cause compile-time errors if uncommented
	// free( cmd_init_set );
	// free( cmd_start );
	// free( cmd_end );
	// free( command );
	// free( value );
	ROS_INFO("Setup stargazer done."); 
}

