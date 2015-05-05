/**
 * stargazer_communication.cpp
 *
 * Created on: 
 * Author: 
 **/

#include <stargazer_communication.h>
#include <iostream>
#include <string>
#include <serial_communication.h>

/**
 * Ctor.
 */
StarGazerCommuniucation::StarGazerCommuniucation(): 
    serial_interface_(NULL)
{
}

/**
 * Dtor.
 */
StarGazerCommuniucation::~StarGazerCommuniucation()
{
}

bool StarGazerCommuniucation::initilizeCommunication()
{
}

bool StarGazerCommuniucation::sendCommand(std::string command, std::string value, bool read_write)
{
    bool success = false;
   
    return success;
}

bool StarGazerCommuniucation::isACKRecieved()
{
    bool success = false;

    return success;
}


bool StarGazerCommuniucation::writeserial(char data)
{
    bool success = false;

    return success;
}

bool StarGazerCommuniucation::readserial(char *data)
{
    bool success = false;

    return success;
}

bool StarGazerCommuniucation::closeCommunication()
{
    bool success = false;
    return !success;
}