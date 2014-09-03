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
 * @file   uwrovctr-module.h
 * @author Filippo Campagnaro
 * @version 1.1.0
 * 
 * \brief Provides the <i>UWROV</i> packets header description and the definition of the class <i>UWROV</i>.
 * 
 * Provides the <i>UWROV</i> packets header description and the definition of the class <i>UWROV</i>.
 * <i>UWROV</i> can manage no more than 2^16 packets. If a module generates more
 * than 2^16 packets, they will be dropped.
 */

#ifndef UWROV_CTR_MODULE_H
#define UWROV_CTR_MODULE_H

#include <uwcbr-module.h>
#include "uwrov-packet.h"
#include "node-core.h"
#include <queue>

#define UWROV_DROP_REASON_UNKNOWN_TYPE "UKT"      /**< Reason for a drop in a <i>UWROV</i> module. */
#define UWROV_DROP_REASON_OUT_OF_SEQUENCE "OOS"   /**< Reason for a drop in a <i>UWROV</i> module. */
#define UWROV_DROP_REASON_DUPLICATED_PACKET "DPK" /**< Reason for a drop in a <i>UWROV</i> module. */

#define HDR_UWROV_MONITORING(p)      (hdr_uwROV_monitoring::access(p))
#define HDR_UWROV_CTR(p)      (hdr_uwROV_ctr::access(p))

using namespace std;
class UwROVCtrModule;

/**
 * UwSendTimer class is used to handle the scheduling period of <i>UWROV</i> packets.
 */
class UwROVCtrSendTimer : public UwSendTimer {
public:
    UwROVCtrSendTimer(UwROVCtrModule *m) : UwSendTimer((UwCbrModule*)(m)){
    };

};
/**
 * UwROVCtrModule class is used to manage <i>UWROVCtr</i> packets and to collect statistics about them.
 */
class UwROVCtrModule : public UwCbrModule {

public:
    Position posit;
	float x_rov;
	float y_rov;
	float z_rov;
	float newX;
	float newY;
	float newZ;
    float speed;
    int sn;
    Packet * p = NULL;
  //  std::queue<Packet*> buffer;
/**
 * Constructor of UwROVCtrModule class.
 */
    UwROVCtrModule();
/**
 * Constructor of UwROVCtrModule class with position setting.
 */
    UwROVCtrModule(Position p);
    
/**
 * Destructor of UwROVCtrModule class.
 */
    virtual ~UwROVCtrModule();

/**
 * Tcl command management
 */
    virtual int command(int argc, const char*const* argv);

  
/**
 * Performs the initialization of a control packet.
 */
    virtual void initPkt(Packet* p) ;
 /**
* Reset retransmissions
*/   
    inline void reset_retx() {p=NULL;sendTmr_.force_cancel();}
/**
* Set the position of the ROV
*/
    virtual void setPosition(Position p);
/**  
* Get the position of the ROV
*/
    virtual Position getPosition();
/**
 * Performs the reception of packets from upper and lower layers.
 * 
 * @param Packet* Pointer to the packet will be received.
 */
    virtual void recv(Packet*);
    
/**
 * Performs the reception of packets from upper and lower layers.
 * 
 * @param Packet* Pointer to the packet will be received.
 * @param Handler* Handler.
 */
    virtual void recv(Packet* p, Handler* h);
     /**
     * Creates and transmits a packet.
     * 
     * @see UwCbrModule::sendPkt()
     */
    virtual void transmit();
    
    /**
     * Start .
     */
    virtual void start();
    

/**
 * Returns the size in byte of a <i>hdr_uwROV_monitoring</i> packet header.
 * 
 * @return The size of a <i>hdr_uwROV_monitoring</i> packet header.
 */
    static inline int getROVMonHeaderSize() { return sizeof(hdr_uwROV_monitoring); }
/**
 * Returns the size in byte of a <i>hdr_uwROV_ctr</i> packet header.
 * 
 * @return The size of a <i>hdr_uwROV_monitoring</i> packet header.
 */
    static inline int getROVCTRHeaderSize() { return sizeof(hdr_uwROV_ctr); }
};

#endif // UWROVCtr_MODULE_H
