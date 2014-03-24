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
 * @file   uwmsg-module.cc
 * @author Ivano Calabrese
 * @version 1.1.0
 * 
 * \brief Provides the <i>UWMSG</i> class implementation.
 * 
 * Provides the <i>UWMSG</i> class implementation.
 */

#include "uwmsg-module.h"
packet_t PT_UWMSG;

/**
 * Class to create the Otcl shadow object for an object of the class UWMPhypatch.
 */
static class UwMsgModuleClass : public TclClass {
public:
    int portno;

    UwMsgModuleClass() : TclClass("Module/UW/MSG") {
    }

    TclObject* create(int args, const char*const* argv) {
        portno = atoi(argv[4]);
        return (new UwMsgModule(portno));
    }
} class_module_apptcp;

UwMsgModule::UwMsgModule(int portno_) : checkTmr(this) {
    bind("debug_", &debug_);
    bind("period_", &period);
    portno = portno_;
    sn = 0;

    openConnection();
}

UwMsgModule::~UwMsgModule() {
    closeConnection();
}

void UwMsgModule::recv(Packet* p) {
    hdr_cmn* ch = HDR_CMN(p);
    hdr_uwudp* uwudph = hdr_uwudp::access(p);
    hdr_uwip* uwiph = hdr_uwip::access(p);
    hdr_uwmsg* uwmsgh = HDR_UWMSG(p);

    if (ch->direction() == hdr_cmn::UP) {
        ext_pkt msgapp;
        msgapp.data = (char*) &(uwmsgh->payloadmsg_);
        msgapp.dst = static_cast<ostringstream*> (&(ostringstream() << uwiph->daddr_))->str();
        msgapp.src = static_cast<ostringstream*> (&(ostringstream() << uwiph->saddr_))->str();
        writeToTCP(msgapp);
    } else {
        ch->uid_ = sn++;
        ch->ptype_ = PT_UWMSG;
        ch->direction_ = hdr_cmn::DOWN;
        ch->timestamp() = (u_int32_t) (SAMPLERATE * Scheduler::instance().clock());

        ch->size_ = sizeof (receivedExt_pkt);
        ch->prev_hop_ = RIP_OFFSET + atoi(receivedExt_pkt.src.c_str());
        ch->next_hop_ = RIP_OFFSET + atoi(receivedExt_pkt.dst.c_str());
        memcpy(&(uwmsgh->payloadmsg_), receivedExt_pkt.data.c_str(), receivedExt_pkt.data.length());
        uwiph->saddr_ = 0;
//        uwiph->daddr_ = RIP_OFFSET + atoi(receivedExt_pkt.dst.c_str());
        uwiph->daddr_ = static_cast<uint8_t>(atoi(receivedExt_pkt.dst.c_str()));
        uwudph->dport_ = 1; //HARDCODED, is this right? How many RACUN.APP can we have?

        std::cout << "\033[0;32m +[BEFORE TO SENDDOWN] msgapp_me.src: " << ch->prev_hop_ << "\033[0m" << std::endl;
        std::cout << "\033[0;32m +[BEFORE TO SENDDOWN] msgapp_me.dst: " << ch->next_hop_ << "\033[0m" << std::endl;
        std::cout << "\033[0;32m +[BEFORE TO SENDDOWN] msgapp_me.data: " << uwmsgh->payloadmsg_ << "\033[0m" << std::endl;

        sendDown(p);
    }
}

int UwMsgModule::openConnection() {
    // stucture that contains amongst others server address and port
    struct sockaddr_in serv;
    // sockfd is now file-descriptor
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
        printf("\033[0;31m SERVER_OPEN --> Error: Building socket!\033[0m");
        return 0;
    }

    memset(&serv, 0, sizeof (serv));
    // communicate with TCP/IP
    serv.sin_family = AF_INET;
    // server address
    serv.sin_addr.s_addr = inet_addr(SERVER_IP);
    // Port in "network-byte-order" therefore "htons"
    serv.sin_port = htons(portno);

    ::bind(sockfd, (struct sockaddr *) &serv, sizeof (serv));

    if (listen(sockfd, 1) < 0) {
        printf("\033[0;31m SERVER_OPEN --> Error: Listening to socket!\033[0m");
        return 0;
    } else
        printf("\033[0;32m SERVER_OPEN --> Listening on port: %d\033[0m \n", portno);


    pthread_t pth;
    pthread_create(&pth, NULL, read_process, (void*) this);

    checkTmr.resched(getPeriod());

    return (sockfd); 
}

void UwMsgModule::closeConnection() {
    close(sockfd);
}

int UwMsgModule::writeToTCP(const ext_pkt& msgapp_) {
    char tmpsend[255];
    strcpy(tmpsend, msgapp_.data.c_str());
    return send(fd, tmpsend, sizeof (tmpsend), 0);
}

void UwMsgModule::readFromTCP() {
    if (!queueExt_pkt.empty()) {

        receivedExt_pkt = queueExt_pkt.front();
        queueExt_pkt.pop();

        Packet* dummy;
        dummy = Packet::alloc();
        hdr_cmn *ch = HDR_CMN(dummy);
        ch->direction_ = hdr_cmn::DOWN;
        recv(dummy); // this function call sends down the ns packet.
    }
}

void *read_process(void* arg) {
    UwMsgModule* pUwMsgModule_me = (UwMsgModule*) arg;
    ext_pkt msgapp_me;
    char msgapp_data[2040];

    struct sockaddr_in dest;
    socklen_t len_sock = sizeof (sockaddr_in);

    while (1) {
        usleep(50);

        // Fetch a new connection off the queue.
        pUwMsgModule_me->fd = accept(pUwMsgModule_me->getSocket(), (struct sockaddr *) &dest, &len_sock);
        if (pUwMsgModule_me->fd == -1) {
            // Oh dear! Something’s gone wrong! Whimper and die.
            perror("Could not accept incoming connection.");
            exit(EXIT_FAILURE);
        }
        break;
    }

    while (1) {
        usleep(50000);
        strcpy(msgapp_data, "");
        if (read(pUwMsgModule_me->fd, msgapp_data, sizeof (msgapp_data)) > 0) {
            string msgapp_mesrc;
            string msgapp_medst;
            char * ptmp_ = strtok(msgapp_data, "-");
            if (ptmp_ != NULL) {
                std::cout << "save in msgapp_me.src " << ptmp_ << std::endl;
                msgapp_mesrc = (char*) ptmp_;
                ptmp_ = strtok(NULL, "-");
            }
            if (ptmp_ != NULL) {
                std::cout << "save in msgapp_me.dst " << ptmp_ << std::endl;
                msgapp_medst = (char*) ptmp_;
                ptmp_ = strtok(NULL, "-");
            }
            if (ptmp_ != NULL) {
                std::cout << "save in msgapp_me.data " << ptmp_ << std::endl;
                string msgapp_medata = (char*) ptmp_;
                ptmp_ = strtok(NULL, "-");
                const int lenData = msgapp_medata.length();
                int ii = 0;
                while (ii < lenData) {
                    msgapp_me.src = msgapp_mesrc;
                    msgapp_me.dst = msgapp_medst;
                    msgapp_me.data = msgapp_medata.substr(ii, 30);
                    pUwMsgModule_me->queueExt_pkt.push(msgapp_me);
                    ii += 30;
                }
            } else {
                msgapp_me.src = msgapp_mesrc;
                msgapp_me.dst = msgapp_medst;
                msgapp_me.data = "";
                pUwMsgModule_me->queueExt_pkt.push(msgapp_me);
            }

            if (pUwMsgModule_me->queueExt_pkt.size() > _MAX_QUEUE_LENGTH) {
                pUwMsgModule_me->queueExt_pkt.pop();
            }

            flush(std::cout);
        }
    }
}

//////////////////////////////////////////
void CheckTimerUwmsg::expire(Event *e) {
    pmUwMsgModule->readFromTCP();
    resched(pmUwMsgModule->getPeriod());
}
