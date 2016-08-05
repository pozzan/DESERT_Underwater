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

#ifndef UWCBR_PACKET_H
#define UWCBR_PACKET_H

#include <packet.h>

extern "C" {
#include <stdint.h>
}

#define HDR_UWCBR(p) (hdr_uwcbr::access(p))

extern packet_t PT_UWCBR;

typedef uint16_t sn_t;

/**
 * <i>hdr_uwcbr</i> describes <i>UWCBR</i> packets.
 */
struct hdr_uwcbr {
    sn_t sn_;       /**< Serial number of the packet. */
    float rftt_;        /**< Forward Trip Time of the packet. */
    bool rftt_valid_;   /**< Flag used to set the validity of the fft field. */
    char priority_;     /**< Priority flag: 1 means high priority, 0 normal priority. */
    bool is_ack_;       /**< Flag that indicates if this packet is an ACK */
    double gen_timestamp_; /**< Time when the packet was generated and put in the send_queue */

    static int offset_; /**< Required by the PacketHeaderManager. */

    /**
     * Reference to the offset_ variable.
     */
    static int& offset() { return offset_; }

    static hdr_uwcbr *access(const Packet *p) {
        return (hdr_uwcbr*) p->access(offset_);
    }

    /**
     * Reference to the sn_ variable.
     */
    sn_t& sn() { return sn_; }

    /**
     * Reference to the rftt_valid_ variable.
     */
    bool& rftt_valid() { return rftt_valid_; }

    /**
     * Reference to the priority_ variable.
     */
    char& priority() { return priority_; }

    /**
     * Reference to the rftt_ variable.
     */
    float& rftt() { return (rftt_); }

    bool &is_ack() { return is_ack_; }
    double &gen_timestamp() { return gen_timestamp_; }
};
#endif

// Local Variables:
// mode: c++
// c-basic-offset: 4
// indent-tabs-mode: nil
// End:
