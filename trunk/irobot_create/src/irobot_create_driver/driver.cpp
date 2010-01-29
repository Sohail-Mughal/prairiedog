/*
 *  irobot_create.cpp
 *  
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/15/09.
 *
 */

//#include "irobot_create.h"
//#include <unistd.h>
//#include <stdlib.h>
//#include <std_msgs/String.h>

#include <ros/ros.h>
#include <create_comms.h>
#include <irobot_create/Speeds.h>
#include <irobot_create/Sound.h>
#include <irobot_create/Position2D.h>
#include <irobot_create/Power.h>
#include <irobot_create/Bumper.h>

//#include <iostream>
//#include <strings.h>
//#include <string.h>
//#include <stdio.h>

class IRobotCreateDriver
{
public:
	create_comm_t* create_dev_;			// the underlying create object
	std::string serial_port_;			// serial port where the create is
	bool safe_;							// full control or not
	
	ros::NodeHandle n_;
	ros::Subscriber speeds_sub_;
	ros::Subscriber sound_sub_;
	ros::Publisher pos2d_pub_;
	ros::Publisher power_pub_;
	ros::Publisher bumper_pub_;

	unsigned short last_bumper_left_;
	unsigned short last_bumper_right_;
	
	void init() 
	{
		//strcpy(serial_port_,"/dev/ttyUSB0");  // pull out serial string
		//strcpy(serial_port_,"/dev/tty.usbserial-FTE2VF80");  // pull out serial string

                // NOTE: ROS now gets mad at using '~' here, this is a hack job to fix this

		//safe_ = 1;
		//n_.param("~safe", safe_, false);
                safe_ = true;
		ROS_DEBUG("Safe mode: %s",(safe_)?"true":"false");
		
		// Retrieve internal serial port parameter
		//n_.param("~serial_port", serial_port_, std::string("/dev/create"));
                serial_port_ = std::string("/dev/create");
		ROS_DEBUG_STREAM("Serial Port: " << serial_port_);

		// Attempt to connect to the create
		create_dev_ = create_create(serial_port_.c_str());

		if(create_open(create_dev_, !safe_) < 0)
		{
			create_destroy(create_dev_);
			create_dev_ = NULL;
			//PLAYER_ERROR("failed to connect to create");
			//fprintf(stdout,"failed to connect to create\n");
			ROS_FATAL("Failed to connect to create");
			ROS_BREAK();
		}
                else
                   ROS_INFO("connect to: %s successfull", serial_port_.c_str());
		
		
		// Subscribe to the speeds bus
		speeds_sub_ = n_.subscribe("speeds_bus", 10, &IRobotCreateDriver::speedsCallback, this);
		// Subscribe to the sound bus
		sound_sub_ = n_.subscribe("sound_bus", 10, &IRobotCreateDriver::soundCallback, this);
		// Advertise the position2d bus
		pos2d_pub_ = n_.advertise<irobot_create::Position2D>("pos2d_bus",10);
		// Advertise the power bus
		power_pub_ = n_.advertise<irobot_create::Power>("power_bus",10);
		// Advertise the bumper bus
		bumper_pub_ = n_.advertise<irobot_create::Bumper>("bumper_bus",10);
		last_bumper_left_ = 0;
		last_bumper_right_ = 0;
	}
	
	void speedsCallback(const irobot_create::SpeedsConstPtr& in)
	{
		ROS_DEBUG("Sending motor commands forward: %f, rotate: %f", in->forward, in->rotate);
		if(create_set_speeds(create_dev_, 
							 in->forward, 
							 in->rotate) < 0)
		{
			ROS_ERROR("Failed to set speeds to create");
		}
	}
	
	void soundCallback(const irobot_create::SoundConstPtr& in)
	{
		
		if (in->command == 1) {// irobot_create::Sound.PLAY_SONG) {		// ROS BUG??
			unsigned char index = in->song_index;
			ROS_DEBUG("Playing song at index: %d", index);
			create_play_song(create_dev_, index);
		} else if (in->command == 2) {//irobot_create::Sound.SET_SONG) {
			unsigned char index = in->song_index;
			unsigned char song_length = in->song_length;
			unsigned char notes[song_length];
			unsigned char note_lengths[song_length];
			
			for (unsigned int i=0; i<song_length; i++) {
				notes[i] = in->notes[i];
				note_lengths[i] = in->note_lengths[i];
			}
			
			ROS_DEBUG("Setting new song at index: %d", index);
			create_set_song(create_dev_, index, song_length, notes, note_lengths);
		}
		
		//ROS_DEBUG("Sending motor commands forward: %f, rotate: %f", in->forward, in->rotate);
		/*if(create_set_speeds(create_dev_, 
							 in->forward, 
							 in->rotate) < 0)
		{
			ROS_ERROR("Failed to set speeds to create");
		}*/
		
	}
	
	void mainDeviceLoop() {
		ros::Rate loop_rate(5);
		
		while (n_.ok())
		{
			if(create_get_sensors(create_dev_, -1) < 0)
			{
				ROS_ERROR("Failed to get sensor data from create");
				create_close(create_dev_);
				return;
			}
			
			////////////////////////////
			// Update position2d data
			irobot_create::Position2D out_p2d;
			out_p2d.x = create_dev_->ox;
			out_p2d.y = create_dev_->oy;
			out_p2d.a = create_dev_->oa;
			pos2d_pub_.publish(out_p2d);
			ROS_DEBUG("Sending Position2D Data:");
			ROS_DEBUG("\tX:%f",out_p2d.x);
			ROS_DEBUG("\tY:%f",out_p2d.y);
			ROS_DEBUG("\tA:%f",out_p2d.a);
			
			////////////////////////////
			// Update power data
			irobot_create::Power out_pow;
			out_pow.volts = create_dev_->voltage;
			out_pow.watts = create_dev_->voltage * create_dev_->current;
			out_pow.joules = create_dev_->charge; 
			out_pow.percent = 100.0 * (create_dev_->charge / create_dev_->capacity); 
			if (out_pow.percent > 110.0) {
				out_pow.joules = 0.0;
				out_pow.percent = 0.0;
			}
			out_pow.charging = (create_dev_->charging_state == CREATE_CHARGING_NOT) ? 0 : 1;
			power_pub_.publish(out_pow);
			ROS_DEBUG("Sending Power Data:");
			ROS_DEBUG("\tVolts:%f",out_pow.volts);
			ROS_DEBUG("\tWatts:%f",out_pow.watts);
			ROS_DEBUG("\tJoules:%f",out_pow.joules);
			ROS_DEBUG("\tPercent:%f",out_pow.percent);
			ROS_DEBUG("\tCharging:%s",(out_pow.charging)?"true":"false");
			
			////////////////////////////
			// Update bumper data
			unsigned short new_bumper_left = create_dev_->bumper_left;
			unsigned short new_bumper_right = create_dev_->bumper_right;
			if (new_bumper_left != last_bumper_left_ || 
				new_bumper_right != last_bumper_right_) {
				irobot_create::Bumper out_bump;
				out_bump.left = new_bumper_left;
				out_bump.right = new_bumper_right;
				bumper_pub_.publish(out_bump);
				last_bumper_left_ = new_bumper_left;
				last_bumper_right_ = new_bumper_right;
			}
			ROS_DEBUG("Sending Bumper State:");
			ROS_DEBUG("\tLeft:%s", (last_bumper_left_)?"true":"false");
			ROS_DEBUG("\tRight:%s", (last_bumper_right_)?"true":"false");
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	void shutdown()
	{
		if(create_close(create_dev_))
		{
			//PLAYER_ERROR("failed to close create connection");
			//fprintf(stdout,"failed to close create connection\n");
			ROS_ERROR("Failed to close create connection");
		}
		create_destroy(create_dev_);
		create_dev_ = NULL;
	}
	
};
			 



int main(int argc, char** argv) 
{
	
	// Initialize
	ros::init(argc, argv, "irobot_create_driver");
	
	// Create a new instance of IRobot_Create
	IRobotCreateDriver driver;
	
	//ros::Publisher speeds_pub = n.advertise<irobot_create::Speed>("speeds", 100);
	
	ROS_INFO("Initializing...");
	driver.init();
	ROS_INFO("Connection established");
	
	// main loop
	driver.mainDeviceLoop();
	
	//destroy
	ROS_INFO("Shutting Down...");
	driver.shutdown();
	ROS_INFO("Done.");
}
