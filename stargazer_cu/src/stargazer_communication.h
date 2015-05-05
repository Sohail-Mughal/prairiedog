/**
 * stargazer_communication.h
 *
 * Created on: 
 * Author: 
 **/

#ifndef STARGAZERCOMMUNICATION_H_
#define STARGAZERCOMMUNICATION_H_
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

class StarGazerCommuniucation
{
    public:
        /**
         * Ctor.
         */
        StarGazerCommuniucation();
        /**
         * Dtor.
         */
        virtual ~StarGazerCommuniucation();

        bool initilizeCommunication();

        bool sendCommand(std::string command, std::string value, bool read_write);

        bool read();

        bool isACKRecieved();

        bool closeCommunication();

    private:
};

#endif /** STARGAZERCOMMUNICATION_H_ **/