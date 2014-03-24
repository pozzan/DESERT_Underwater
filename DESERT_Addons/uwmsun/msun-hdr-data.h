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
 * @file   msun-hdr-data.h
 * @author Giovanni Toso
 * @version 1.0.0
 * 
 * \brief Provides the Data Messages header description.
 *
 * Provides the Data Messages header description
 */

#ifndef __HDR_MSUN_DATA_H_
#define __HDR_MSUN_DATA_H_

#include "msun-common.h"

#include <packet.h>

#define HDR_MSUN_DATA(p) (hdr_msun_data::access(p))

extern packet_t PT_MSUN_DATA;

/**
 * <i>hdr_msun_data</i> describes data packets used by <i>UWMSUN</i>
 */
typedef struct hdr_msun_data {
#ifdef SMALL_ADDR
    uint8_t list_of_hops_[msun::MAX_HOP_NUMBER];  /**< List of IPs saved in the header */
#else
    nsaddr_t list_of_hops_[msun::MAX_HOP_NUMBER]; /**< List of IPs saved in the header */
#endif 
    int8_t pointer_to_list_of_hops_;              /**< Pointer used to keep track of the last IPs processed */
    uint8_t list_of_hops_length_;                 /**< Current number of IPs stored in the header */
    static int offset_;                           /**< Required by the PacketHeaderManager */

    /**
     * Reference to the offset_ variable
     */
    inline static int& offset() {
        return offset_;
    }

#ifdef SMALL_ADDR
    /**
     * Pointer to the list_of_hops_ variable
     */
    inline uint8_t* list_of_hops() {
        return list_of_hops_;
    }
#else
    /**
     * Pointer to the list_of_hops_ variable
     */
    inline nsaddr_t* list_of_hops() {
        return list_of_hops_;
    }
#endif
    
    /**
     * Reference to the list_of_hops_length_ variable
     */
    inline uint8_t& list_of_hops_length() {
        return list_of_hops_length_;
    }
    
    /**
     * Reference to the pointer_to_list_of_hops_ variable
     */
    inline int8_t& pointer() {
        return pointer_to_list_of_hops_;
    }
    
    inline static struct hdr_msun_data* access(const Packet * p) {
        return (struct hdr_msun_data*) p->access(offset_);
    }    
} hdr_msun_data;

#endif // __HDR_MSUN_DATA_H_
