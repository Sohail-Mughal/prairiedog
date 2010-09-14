/*
 *  irobot_create_controller.h
 *  
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/15/09.
 *
 *  Modified by Michael Otte University of Colorado at Boulder 2010 for irobot_create_rustic 
 *
 */

#include <ros/ros.h>
#include <irobot_create_rustic/Speeds.h>
#include <irobot_create_rustic/Sound.h>
#include <irobot_create_rustic/Position2D.h>
#include <irobot_create_rustic/Power.h>
#include <irobot_create_rustic/Bumper.h>

class IRobotCreateController
{
private:
	
	ros::NodeHandle n_;
	ros::Publisher speeds_pub_;
	ros::Publisher sound_pub_;
	ros::Subscriber pos2d_sub_;
	ros::Subscriber power_sub_;
	ros::Subscriber bumper_sub_;
	
	unsigned short bump_left;
	unsigned short bump_right;
	double x;
	double y;
	double a;
	double volts;
	double watts;
	double joules;
	double percent;
	unsigned short charging;
	
	void pos2DCallback(const irobot_create_rustic::Position2DConstPtr& in);
	void powerCallback(const irobot_create_rustic::PowerConstPtr& in);
	void bumperCallback(const irobot_create_rustic::BumperConstPtr& in);
	
public:
	
	IRobotCreateController();//ros::NodeHandle n);
	~IRobotCreateController();
	
	void setSpeeds(double forward, double rotate);
	void playSong(unsigned char index);
	void setSong(unsigned char index, 
				 unsigned char length,
				 unsigned char notes[], 
				 unsigned char note_lengths[]);
	
	bool isBumpedLeft() {return bump_left;}
	bool isBumpedRight() {return bump_right;}
	double getX() {return x;}
	double getY() {return y;}
	double getRot() {return a;}
	double getVolts() {return volts;}
	double getWatts() {return watts;}
	double getJoules() {return joules;}
	double getPercent() {return percent;}
	bool isCharging() {return charging;}
	
};
