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
 * @file   application/uwmsg/initlib.cc
 * @author Ivano Calabrese
 * @version 1.1.0
 *
 * \brief Provides the initialization of uwmsg libraries.
 *
 * Provides the initialization of uwmsg libraries.
 */

#include <tclcl.h>
#include "uwmsg_pkt.h"

extern packet_t PT_UWMSG;
int hdr_uwmsg::offset_;


/**
 * Adds the header for <i>hdr_uwmsg</i> packets in ns2.
 */
static class UwMsgPktClass : public PacketHeaderClass {
public:
	UwMsgPktClass() : PacketHeaderClass("PacketHeader/UWMSG", sizeof(hdr_uwmsg)) {
		this->bind();
		bind_offset(&hdr_uwmsg::offset_);
	}
} class_uwmsg_hdr;

extern EmbeddedTcl UwMsgInitTclCode;

extern "C" int Uwmsg_Init() {
  	UwMsgPktClass* uwsh = new UwMsgPktClass;
	uwsh->bind();
	PT_UWMSG = p_info::addPacket("UWMSG");

	UwMsgInitTclCode.load();
	return 0;
}

extern "C" int Cyguwmsg_Init() {
     return Uwmsg_Init();
}


