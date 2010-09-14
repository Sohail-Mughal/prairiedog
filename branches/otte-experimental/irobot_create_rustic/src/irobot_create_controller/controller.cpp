/*
 *  irobot_create_controller.cpp
 *  
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/15/09.
 *
 *  Modified by Michael Otte University of Colorado at Boulder 2010 for irobot_create_rustic 
 *
 */

//#include "irobot_create.h"
//#include <unistd.h>
//#include <stdlib.h>
//#include <std_msgs/String.h>

#include <irobot_create_rustic/irobot_create_controller.h>

//#include <iostream>
//#include <strings.h>
//#include <string.h>
//#include <stdio.h>


IRobotCreateController::IRobotCreateController() {//ros::NodeHandle n) {
	// Initialize
	//n_ = n;
	bump_left = false;
	bump_right = false;
	
	speeds_pub_ = n_.advertise<irobot_create_rustic::Speeds>("speeds_bus",10);
	sound_pub_ = n_.advertise<irobot_create_rustic::Sound>("sound_bus", 10);
	pos2d_sub_ = n_.subscribe("pos2d_bus", 10,  &IRobotCreateController::pos2DCallback, this);
	power_sub_ = n_.subscribe("power_bus", 10, &IRobotCreateController::powerCallback, this);
	bumper_sub_ = n_.subscribe("bumper_bus", 10, &IRobotCreateController::bumperCallback, this);
}

IRobotCreateController::~IRobotCreateController() {
	// Destructor
	ROS_DEBUG("Stopping the robot...");
	setSpeeds(0,0);
	//printf("CONTROLLER BEING DESTROYED\n");
}

void
IRobotCreateController::pos2DCallback(const irobot_create_rustic::Position2DConstPtr& in) {
	ROS_DEBUG("Recieved Position2D State:");
	x = in->x;
	ROS_DEBUG("\tX:%f",x);
	y = in->y;
	ROS_DEBUG("\tY:%f",y);
	a = in->a;
	ROS_DEBUG("\tA:%f",a);
}

void
IRobotCreateController::powerCallback(const irobot_create_rustic::PowerConstPtr& in) {
	ROS_DEBUG("Recieved Power State:");
	volts = in->volts;
	ROS_DEBUG("\tVolts:%f",volts);
	watts = in->watts;
	ROS_DEBUG("\tWatts:%f",watts);
	joules = in->joules;
	ROS_DEBUG("\tJoules:%f",joules);
	percent = in->percent;
	ROS_DEBUG("\tPercent:%f",percent);
	charging = in->charging;
	ROS_DEBUG("\tCharging:%s",(charging)?"true":"false");
}

void
IRobotCreateController::bumperCallback(const irobot_create_rustic::BumperConstPtr& in) {
	ROS_DEBUG("Recieved Bumper State:");
	bump_left = in->left;
	ROS_DEBUG("\tLeft:%s",
			  (bump_left)?"true":"false");
	bump_right = in->right;
	ROS_DEBUG("\tRight:%s",
			  (bump_right)?"true":"false");
}

void 
IRobotCreateController::setSpeeds(double forward, double rotate) {
	irobot_create_rustic::Speeds out;
	out.forward = forward;
	out.rotate = rotate;
	speeds_pub_.publish(out);
}

void
IRobotCreateController::playSong(unsigned char index) {
	irobot_create_rustic::Sound out;
	out.command = out.PLAY_SONG;
	out.song_index = index;
	out.song_length = 0;
	//out.notes[0] = 0;
	//out.note_lengths[0] = 0;
	
	ROS_DEBUG("Sending PLAY SONG command at index: %d\n", index);
	sound_pub_.publish(out);
}

void
IRobotCreateController::setSong(unsigned char index, 
								unsigned char length, 
								unsigned char notes[], 
								unsigned char note_lengths[]) {
	irobot_create_rustic::Sound out;
	out.command = out.SET_SONG;
	out.song_index = index;
	out.song_length = length;
	
	ROS_DEBUG("Sending SET SONG command at index: %d\n", index);
	for (unsigned int i=0; i<out.song_length; i++) {
		out.notes.push_back(notes[i]);
		out.note_lengths.push_back(note_lengths[i]);
		ROS_DEBUG("\tnotes[%d]: %d note_lenghts[%d]: %d\n",i,notes[i],i,note_lengths[i]);
	}
	sound_pub_.publish(out);
}
