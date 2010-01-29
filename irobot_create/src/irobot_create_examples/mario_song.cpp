/*
 *  client.cpp
 *  
 *
 *  Created by Gheric Speiginer and Keenan Black on 6/16/09.
 *
 */

#include <ros/ros.h>
#include <irobot_create/irobot_create_controller.h>
#include <signal.h>


bool endnow = false;
void stopper(int x)
{
	endnow = true;
}

int main(int argc, char** argv) 
{

	ros::init(argc, argv, "mario_song");
	// initialize ROS
	ros::NodeHandle n;
	// create a handle to this process node	
	IRobotCreateController controller = IRobotCreateController();
	// create an instance of controller
	
	//usleep(2*1000000);
	// allow for a 5 second break
	
	unsigned char length = 75;
	unsigned char notes[length];
	unsigned char note_lengths[length];
	int count = 0;
	notes[count] = 76;	note_lengths[count++] = 8;
	notes[count] = 76;	note_lengths[count++] = 16;
	notes[count] = 76;	note_lengths[count++] = 16;
	notes[count] = 72;	note_lengths[count++] = 8;
	notes[count] = 76;	note_lengths[count++] = 16;
	notes[count] = 79;	note_lengths[count++] = 32;
	notes[count] = 67;	note_lengths[count++] = 32;
	//7
	notes[count] = 72;	note_lengths[count++] = 24;
	notes[count] = 67;	note_lengths[count++] = 24;	
	notes[count] = 64;	note_lengths[count++] = 24;
	notes[count] = 69;	note_lengths[count++] = 16;	
	notes[count] = 71;	note_lengths[count++] = 16;
	notes[count] = 70;	note_lengths[count++] = 8;
	notes[count] = 69;	note_lengths[count++] = 16;
	//14
	notes[count] = 67;	note_lengths[count++] = 8;
	notes[count] = 72;	note_lengths[count++] = 16;	
	notes[count] = 76;	note_lengths[count++] = 8;
	notes[count] = 81;	note_lengths[count++] = 16;	
	notes[count] = 77;	note_lengths[count++] = 8;
	notes[count] = 79;	note_lengths[count++] = 16;
	notes[count] = 76;	note_lengths[count++] = 16;
	//21
	notes[count] = 72;	note_lengths[count++] = 8;
	notes[count] = 74;	note_lengths[count++] = 16;
	notes[count] = 71;	note_lengths[count++] = 24;
	//24
	
	notes[count] = 60;	note_lengths[count++] = 16;	
	notes[count] = 79;	note_lengths[count++] = 8;
	notes[count] = 78;	note_lengths[count++] = 8;
	notes[count] = 77;	note_lengths[count++] = 8;
	notes[count] = 74;	note_lengths[count++] = 8;
	notes[count] = 75;	note_lengths[count++] = 8;
	notes[count] = 76;	note_lengths[count++] = 16;
	//31
	
	notes[count] = 67;	note_lengths[count++] = 8;
	notes[count] = 69;	note_lengths[count++] = 8;
	notes[count] = 72;	note_lengths[count++] = 16;
	notes[count] = 69;	note_lengths[count++] = 8;
	notes[count] = 72;	note_lengths[count++] = 8;
	notes[count] = 74;	note_lengths[count++] = 8;
	//37
	
	notes[count] = 60;	note_lengths[count++] = 16;	
	notes[count] = 79;	note_lengths[count++] = 8;
	notes[count] = 78;	note_lengths[count++] = 8;
	notes[count] = 77;	note_lengths[count++] = 8;
	notes[count] = 74;	note_lengths[count++] = 8;
	notes[count] = 75;	note_lengths[count++] = 8;
	notes[count] = 76;	note_lengths[count++] = 16;
	//44
	notes[count] = 76;	note_lengths[count++] = 4;
	notes[count] = 78;	note_lengths[count++] = 4;
	notes[count] = 84;	note_lengths[count++] = 16;
	notes[count] = 84;	note_lengths[count++] = 8;
	notes[count] = 84;	note_lengths[count++] = 16;
	notes[count] = 84;	note_lengths[count++] = 16;
	//50	
	notes[count] = 60;	note_lengths[count++] = 16;	
	notes[count] = 79;	note_lengths[count++] = 8;
	notes[count] = 78;	note_lengths[count++] = 8;
	notes[count] = 77;	note_lengths[count++] = 8;
	notes[count] = 74;	note_lengths[count++] = 8;
	notes[count] = 75;	note_lengths[count++] = 8;
	notes[count] = 76;	note_lengths[count++] = 16;
	//57
	
	notes[count] = 67;	note_lengths[count++] = 8;
	notes[count] = 69;	note_lengths[count++] = 8;
	notes[count] = 72;	note_lengths[count++] = 16;
	notes[count] = 69;	note_lengths[count++] = 8;
	notes[count] = 72;	note_lengths[count++] = 8;
	notes[count] = 74;	note_lengths[count++] = 16;
	//63
	
	notes[count] = 70;	note_lengths[count++] = 4;
	notes[count] = 72;	note_lengths[count++] = 4;
	notes[count] = 75;	note_lengths[count++] = 16;
	notes[count] = 69;	note_lengths[count++] = 4;
	notes[count] = 71;	note_lengths[count++] = 4;
	notes[count] = 74;	note_lengths[count++] = 16;
	notes[count] = 67;	note_lengths[count++] = 4;
	notes[count] = 69;	note_lengths[count++] = 4;
	notes[count] = 72;	note_lengths[count++] = 16;
	notes[count] = 67;	note_lengths[count++] = 8;
	notes[count] = 67;	note_lengths[count++] = 16;
	notes[count] = 60;	note_lengths[count++] = 24;
	
	
	printf("hi\n");
	
	controller.setSong(1,length,notes,note_lengths);
	printf("hi2\n");
	usleep(2*1000000);
	controller.playSong(1);
	
	/*signal(SIGINT,stopper);
	while(!endnow)//n.ok())
	{
		ros::spinOnce();
	}
	printf("out of loop\n");*/
	
	
	
	
	// stop the create from moving
	/*forward = 0.0;
	rotate = 0.0;
	controller.setSpeeds(forward, rotate);
	*/
	printf("done\n");
	
	
	return(0);
}
