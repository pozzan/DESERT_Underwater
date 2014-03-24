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
// This module has only slightly modification as respect to the mll module contained  in Miracle, 
// released under the same BSD copyright and implemented by 
// Erik Andersson, Emil Ljungdahl, Lars-Olof Moilanen (Karlstad University)

/**
 * @file   uwmll-clmsg.cc
 * @author Saiful Azad
 * @version 1.0.0
 *
 * \brief Provides the implementations of CrossLayer messages useful for MLL module.
 *
 */

#include <cassert>
#include "uwmll-with-arp-clmsg.h"


extern ClMessage_t UWMLL_CLMSG_UPD_MAC;


UWMllClMsgUpdMac::UWMllClMsgUpdMac( Packet* p )
: ClMessage(UWMLL_CLMSG_UPD_MAC_VERBOSITY, UWMLL_CLMSG_UPD_MAC),
  packet(p) 
{
}


ClMessage* UWMllClMsgUpdMac::copy() {
  // Supporting only synchronous messages!!!
  assert(0);
}

