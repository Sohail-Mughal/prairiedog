/**
 * serial_communication.h
 *
 * Created on: 
 * Author: 
 **/

#ifndef SERIALCOMMUNICATION_H_
#define SERIALCOMMUNICATION_H_
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <iostream>
#include <string>


class SerialCommuniucation
{
    public:
        /**
         * Ctor.
         */
        SerialCommuniucation(std::string port_name);
        /**
         * Dtor.
         */
        virtual ~SerialCommuniucation();

        bool initilizeSerial();

        bool writeSerial(char *data);

        bool readSerial(char *data);

        bool closeSerial();
    private:
        std::string port_name_;
        int serial_fd_;
};
#endif /** SERIALCOMMUNICATION_H_ **/