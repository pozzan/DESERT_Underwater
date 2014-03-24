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

/**
 * @file   msun-hdr-ack.h
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the Ack Messages header description.
 *
 * Provides the Ack Messages header description.
 */

#ifndef __HDR_MSUN_ACK_H_
#define __HDR_MSUN_ACK_H_

#include "msun-common.h"

#include <packet.h>

#define HDR_MSUN_ACK(p) (hdr_msun_ack::access(p))

extern packet_t PT_MSUN_ACK;

/**
 * <i>hdr_msun_ack</i> describes acks packets used by <i>UWMSUN</i>.
 */
typedef struct hdr_msun_ack {
#ifdef SMALL_ADDR
    uint8_t    saddr_;          /**< Source address of the ACK sender */
#else
    nsaddr_t   saddr_;          /**< Source address of the ACK sender */
#endif 
    uint16_t   uid_;            /**< Uid of the packet acked. */
    static int offset_;         /**< Required by the PacketHeaderManager. */
    
#ifdef SMALL_ADDR
    /**
     * Pointer to the saddr_ variable
     */
    inline uint8_t& saddr() {
        return saddr_;
    }
#else
    /**
     * Pointer to the saddr_ variable
     */
    inline nsaddr_t& saddr() {
        return saddr_;
    }
#endif
    
    /**
     * Reference to the uid_ variable.
     */
    inline uint16_t& uid() {
        return uid_;
    }
    
    /**
     * Reference to the offset_ variable.
     */
    inline static int& offset() {
        return offset_;
    }
    
    inline static struct hdr_msun_ack* access(const Packet * p) {
        return (struct hdr_msun_ack*) p->access(offset_);
    }
} hdr_msun_ack;

#endif // __HDR_MSUN_ACK_H_
