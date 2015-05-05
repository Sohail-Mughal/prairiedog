/**
 * serial_communication.cpp
 *
 * Created on: 
 * Author: 
 **/
#include <serial_communication.h>
/**
 * Ctor.
 */
SerialCommuniucation::SerialCommuniucation(std::string port_name): 
    port_name_(port_name)
{
}

/**
 * Dtor.
 */
SerialCommuniucation::~SerialCommuniucation()
{
}

bool SerialCommuniucation::initilizeSerial()
{
    struct termios toptions;

    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
    if ( serial_fd_ == -1 ) {
        perror("error: Could not open port\n");
        return false;
    }

    if ( tcgetattr(serial_fd_, &toptions) < 0 ) {
        perror("error: Couldn't get term attributes\n");
        return false;
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
    
    if ( tcsetattr(serial_fd_, TCSANOW, &toptions) < 0 ) {
        perror("error: Couldn't set term attributes\n");
        return false;
    }
}

bool SerialCommuniucation::writeSerial(char *data)
{
    bool success = false;
    if (write ( serial_fd_, data, 1))
        success = true;

    return success;
}

bool SerialCommuniucation::readSerial(char *data)
{
    //std::cout<< "readSerial" << std::endl;
    bool success = false;
    if (read ( serial_fd_, data, 1 ))
        success = true;
    return success;
}

bool SerialCommuniucation::closeSerial()
{
    bool success = false;
    close(serial_fd_);
    return !success;
}