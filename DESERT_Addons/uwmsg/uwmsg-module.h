//
// Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Padova (SIGNET lab) nor the 
//    names of its contributors may be used to endorse or promote products 
//    derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

/**
 * @file   uwmsg-module.h
 * @author Ivano Calabrese
 * @version 1.1.0
 * 
 * \brief Provides the <i>UWMSG</i> packets header description and the definition of the class <i>UWMSG</i>.
 * 
 * Provides the <i>UWMSG</i> packets header description and the definition of the class <i>UWMSG</i>.
 * <i>UWMSG</i> can manage no more than 2^16 packets. If a module generates more
 * than 2^16 packets, they will be dropped.
 */

#ifndef UWMSG_MODULE_H
#define APPTCP_H
#define SERVER_IP "127.0.0.1"
#define _MAX_QUEUE_LENGTH 10
#define HEADER_BYTES 8
#define SAMPLERATE 8000
#define RIP_OFFSET 16777216

#include <module.h>

#include <uwip-module.h>
#include <uwudp-module.h>

#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <sstream>
#include <sys/stat.h>
#include <fcntl.h>

#include <signal.h>

#include <unistd.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
//------------------------------------------------------------------------------

#include <arpa/inet.h>
#include <deque>
#include <list>
#include <queue>

#include "uwmsg_pkt.h"
#include "uwmsg-extpkt.h"

using namespace std;

typedef unsigned char recvPck[2048];

class UwMsgModule;

class CheckTimerUwmsg : public TimerHandler{
public:
    CheckTimerUwmsg(UwMsgModule* pmUwMsgModule_) : TimerHandler(){
        pmUwMsgModule = pmUwMsgModule_;
    }

protected:
    /** 
     * Method to handle the expiration of a given event.
     * 
     * @param e event to be handled.
     */
    virtual void expire(Event* e);
    UwMsgModule* pmUwMsgModule; /**< Pointer to an UwMsgModule object. It is used to call UwMsgModule::check_modem() when the countdown expires.*/
};

/** 
 * Class to patch the absence of a PHY layer's module when we want to use a module of the MAC layer. This module just 
 * receive a packet and forward it to the layer below (or above) sending the required cross-layer messages to the MAC layer. 
 */
class UwMsgModule : public Module
{
    CheckTimerUwmsg checkTmr;
    int debug_;
    int sockfd; // Sockfd is now file-descriptor
    struct sockaddr_in serv_addr;
    int portno;
    double period;
    ext_pkt receivedExt_pkt;
    int sn;
    

public:
    std::queue<ext_pkt> queueExt_pkt;
    int fd;

    /** 
     * Class constructor.
     */
    UwMsgModule(int portno);

    /**
     * Class destructor.
     */
    ~UwMsgModule();

    /** 
     * Method to handle the reception of packets arriving from the upper layers of the network simulator.
     * 
     * @param p pointer to the packet that has been received from the simulator's upper layers.
     */
    void recv(Packet*);
    void readFromTCP();
    int ReadString(int fd, char* buf, int n);
    int getSocket(){
        return sockfd;
    };

    double getPeriod(){
        return period;
    };

protected:
    int writeToTCP(const ext_pkt& msgapp_);
    int openConnection();
    void closeConnection();
};

#endif // UWMSG_MODULE_H

extern "C" {
    void *read_process(void* arg);
}