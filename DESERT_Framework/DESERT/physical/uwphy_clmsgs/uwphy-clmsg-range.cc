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
 * @file   uwphy-clmsg-range.cc
 * @author Federico Guerra
 * @version 1.0.0
 *
 * \brief Implementation of ClMsgUwPhyRange class.
 *
 */

#include "uwphy-clmsg-range.h"
#include <packet.h>

ClMsgUwPhyRange::ClMsgUwPhyRange(int sender_module_id, int sid, MsgType type)
: ClMessage(CLMSG_UWPHY_RANGE_VERBOSITY, CLMSG_UWPHY_RANGE),
  stack_id(sid),
  packet_ptr(NULL),
  msg_type(type),
  range(CLMSG_UWPHY_RANGE_NOT_VALID),
  time_valid(CLMSG_UWPHY_TIME_NOT_VALID)
{
  setSource(sender_module_id);
}


ClMsgUwPhyRange::ClMsgUwPhyRange(int sender_module_id, int sid, int dest_module_id, MsgType type)
: ClMessage(CLMSG_UWPHY_RANGE_VERBOSITY, CLMSG_UWPHY_RANGE, UNICAST, dest_module_id),
  stack_id(sid),
  packet_ptr(NULL),
  msg_type(type),
  range(CLMSG_UWPHY_RANGE_NOT_VALID),
  time_valid(CLMSG_UWPHY_TIME_NOT_VALID)
{
  setSource(sender_module_id);
}

ClMsgUwPhyRange::ClMsgUwPhyRange(const ClMsgUwPhyRange& msg)
: ClMessage(msg),
  stack_id(CLMSG_UWPHY_STACK_ID_NOT_VALID),
  packet_ptr(NULL)
{
  stack_id = msg.stack_id;

  if (msg.packet_ptr != NULL)
  {
    packet_ptr = msg.packet_ptr->copy();
  }

  msg_type = msg.msg_type;
  range = msg.range;
  time_valid = msg.time_valid;
}


ClMsgUwPhyRange::~ClMsgUwPhyRange()
{
  if (packet_ptr != NULL)
  {
    Packet::free(packet_ptr);
  }
}

ClMsgUwPhyRange* ClMsgUwPhyRange::copy()
{
  return new ClMsgUwPhyRange(*this);
}

void ClMsgUwPhyRange::setPacket(const Packet* pkt_ptr)
{
  if (packet_ptr != NULL)
  {
    Packet::free(packet_ptr);
  }
  packet_ptr = pkt_ptr->copy();
}
