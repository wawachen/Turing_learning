/*
 *  main.cpp
 *  EPuckMonitor
 *
 *  Created by Stefano Morgani on 10/2/08.
 *
 *	Copyright 2008 GCtronic
 *
 *  This file is part of EPuckMonitor.
 *
 *  EPuckMonitor is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  EPuckMonitor is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with EPuckMonitor; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 */
 
#include "SerialComm.h"
#include <math.h>
#include <iostream>
#include <stdio.h>


int main(int argc, char *argv[]) {

    char RxBuffer[45];
    char *portName = "/dev/rfcomm0";
    char command[20];
    SerialComm *comm;
    uint8_t bytesToSend;
    int bytes;
    char msg[50];
    unsigned int distance;
    int err = 0;

    comm = new SerialComm();
    err = comm->connect(portName);
    if(err==-1) {
        std::cerr << "Unable to open serial port " << portName << std::endl;
        return 1;
    }
    bytesToSend = 2;
    command[0]=-0x0D;;          //ToF request
    command[1]=0;               //binary command ending

    comm->flush();
    bytes=comm->writeData(command, bytesToSend, 1000000);
    
    memset(RxBuffer, 0x0, 45);
    bytes=comm->readData((char*)RxBuffer,2,1000000);
            
    if(bytes == 0) {
        std::cerr << "Nothing found"<< std::endl;
        if(comm!=NULL) 
        {
            comm->disconnect();
            comm=NULL;
        }
        return 1;
    } else if(bytes<2) {
        sprintf(msg, "ToF: only %d bytes red", bytes);
        std::cerr << msg << std::endl;
        if(comm!=NULL) 
        {
            comm->disconnect();
            comm=NULL;
        }
        return 1;
    } else {
        distance = (uint16_t)(((uint8_t)RxBuffer[1]<<8)|((uint8_t)RxBuffer[0]))/10;
        distance = (distance>200)?200:distance;
        std::cout<< "The current distance is : "<< distance << std::endl;
    }
	
	// send moving comand
    int speed_left = 100;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = 100;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    comm->writeData(command, 6, 500000);
    std::cout<<"Start moving"<<std::endl;
    usleep(2000000);

    //send stop comand 
    speed_left = 0;
    high_left = (speed_left>>8) & 0xFF;
    low_left = speed_left & 0xFF;
    speed_right = 0;
    high_right = (speed_right>>8) & 0xFF;
    low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    comm->writeData(command, 6, 20000);
    std::cout<<"Stop moving"<<std::endl;
    
    usleep(10000);
    //close communication
    if(comm!=NULL) {
        comm->disconnect();
        comm=NULL;
    }

    return 0;

}

