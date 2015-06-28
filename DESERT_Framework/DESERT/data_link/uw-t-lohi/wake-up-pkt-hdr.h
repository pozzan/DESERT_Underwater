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
//

/**
 * @file   wake-up-pkt-hdr.h
 * @author Federico Guerra
 * @version 1.0.0
 * 
 * \brief Provides the definition of the Wake Up Tone Packet Header
 *
 */

#ifndef UW_WAKEUP_PKT_H 
#define UW_WAKEUP_PKT_H

#define HDR_WKUP(P)      (hdr_wkup::access(P))

extern packet_t PT_WKUP;

/**
 * Class used to manage the Packet in the network
 */
class Packet;
/**
 * Header of the Wake Up Tone
 */
typedef struct hdr_wkup {

  double startRx_time; /**< Start Reception Time */
  double endRx_time;   /**< End Reception Time */

  static int offset_;	/**< Required by PacketManagerHeader class */
	/**
	 * Reference to the offset_ variable */
  inline static int& offset() { return offset_; }
	/**
	 * Method that permits to access to the header from the Packet
	 * @param Packet* pointer to the packet 
	 * @return hdr_wkup* pointer to the struct that represent the header
	 */
  inline static struct hdr_wkup* access(const Packet* p) {
    return (struct hdr_wkup*)p->access(offset_);
  }
  
} hdr_wkup;


#endif /* UW_WAKEUP_PKT_H */
