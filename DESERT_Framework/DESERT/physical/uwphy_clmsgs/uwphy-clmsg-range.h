//
// Copyright (c) 2015 Regents of the SIGNET lab, University of Padova.
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

/**
 * @file   uwphy-clmsg-range.h
 * @author Federico Guerra
 * @version 1.0.0
 *
 * \brief Definition of ClMsgUwPhyRange class.
 *
 */

#ifndef UWPHY_CLMSG_RANGE_H
#define UWPHY_CLMSG_RANGE_H

#include <clmessage.h>

#define CLMSG_UWPHY_RANGE_VERBOSITY (3)

#define CLMSG_UWPHY_RANGE_NOT_VALID (-1)

#define CLMSG_UWPHY_TIME_NOT_VALID (-1)

#define CLMSG_UWPHY_STACK_ID_NOT_VALID (-1)

extern ClMessage_t CLMSG_UWPHY_RANGE;

class Packet;


/**
* ClMsgUwPhyRange should be used to ask to the intended receiver the range to a specific MAC address.
* how to use
* send the ASYNCHRONOUS message in broadcast with optional stack idor to a specific module id (check constructor). the type should be RANGE_REQ
* the receiver will use the copy() method and save a local copy.
* the receiver will use Packet COPY inside the message in order to provide range to destinaction 
* the receiver will either modify the saved message or create a new one with the required parameters (do not forget to change the type to RANGE_REPLY)
* the receiver can set:
* range [m]= CLMSG_UWPHY_RANGE_NOT_VALID if not in range
* time_validity [s]= CLMSG_UWPHY_TIME_NOT_VALID if no info on time validity can be given
* 
**/
class ClMsgUwPhyRange : public ClMessage
{

public:

  enum MsgType
  {
    UW_PHY_CLMSG_TYPE_RANGE_REQ = 0,
    UW_PHY_CLMSG_TYPE_RANGE_REPLY,
    UW_PHY_CLMSG_TYPE_NOT_VALID
  };

  /**
  * Broadcast constructor of the ClMsgUwPhyRange class
  **/
  ClMsgUwPhyRange(int sender_module_id, int stack_id, MsgType type = UW_PHY_CLMSG_TYPE_RANGE_REQ);


  /**
  * Unicast constructor of the ClMsgUwPhyRange class
  **/
  ClMsgUwPhyRange(int sender_module_id, int stack_id, int dest_module_id, MsgType type = UW_PHY_CLMSG_TYPE_RANGE_REQ);

  ClMsgUwPhyRange(const ClMsgUwPhyRange& msg);


  /**
    * Destructor of the ClMsgUwPhyRange class
  **/
  virtual ~ClMsgUwPhyRange();
  
  /**
    * Copy method of the ClMsgUwPhyRange class, the specialization of the return value is intentional and it is allowed by c++ standard
  **/
  virtual ClMsgUwPhyRange* copy();

  /**
  * Copies the input packet into the cross layer message
  **/
  virtual void setPacket(const Packet* pkt_ptr);
  
  virtual const Packet* const getPacket() { return packet_ptr; }


  virtual MsgType getMsgType() { return msg_type; }
  
  virtual void SetMsgType(MsgType type) { msg_type = type; }
  
  
  virtual void setRange(double r) { range = r; }
  
  virtual double getRange() { return range; }


  virtual void setStackId(int s) { stack_id = s; }
  
  virtual double getStackId() { return stack_id; }


  virtual void setTimeValid(double t) { time_valid = t; }

  virtual double getTimeValid() {return time_valid; }

  
protected:

  // receiver stack id (valid only for broadcast request)
  int stack_id;

  /**
  * dynamically allocated Packet object
  **/
  Packet* packet_ptr;
  
  MsgType msg_type;
  
  double range; /// range measure in meters 

  double time_valid;/// validity of the range measure seconds
  
};

#endif /* UWPHY_CLMSG_RANGE_H  */
