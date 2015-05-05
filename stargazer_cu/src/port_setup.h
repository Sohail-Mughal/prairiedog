//Nikos Arechiga 2009
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>

int setup_serial_port ( const char * portname ) 
{
	struct termios toptions;
	int fd;
	//int val_ifread;
	//char * sensor_data;
	//sensor_data = (char *)malloc(40*sizeof(char));
	//char * my_port = "/dev/ttyUSB1";

	fd = open( portname, O_RDWR | O_NOCTTY );
	if ( fd == -1 ) {
		perror("error: Could not open port\n");
		exit(-1);
	}

	if ( tcgetattr(fd, &toptions) < 0 ) {
		perror("error: Couldn't get term attributes\n");
		exit(-1);
	}

	cfsetispeed(&toptions, B115200);
	cfsetospeed(&toptions, B115200);

	//8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;

	//no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;
	toptions.c_iflag &= ~( IXON | IXOFF | IXANY );

	toptions.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG );
	toptions.c_oflag &= ~OPOST;

//	toptions.c_cc[VMIN] = 1;
//	toptions.c_cc[VTIME] = 30;
	
	if ( tcsetattr(fd, TCSANOW, &toptions) < 0 ) {
		perror("error: Couldn't set term attributes\n");
		exit(-1);
	}
	return fd;
}
