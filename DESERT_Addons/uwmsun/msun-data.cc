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
 * @file   msun-data.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding Data Packets.
 *
 * Provides the implementation of all the methods regarding Data Packets.
 */

#include "msun.h"

extern packet_t PT_MSUN_DATA;
extern packet_t PT_MSUN_PATH_EST;

void MSun::forwardDataPacket(Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> forwardDataPacket()" << std::endl;
    
    hdr_cmn* ch          = HDR_CMN(p);
    hdr_uwip* iph        = HDR_UWIP(p);
    hdr_msun_data* hdata = HDR_MSUN_DATA(p);
    
    if (hdata->list_of_hops_length() <= 0) { // Garbage Packet.
        if (trace_)
            this->tracePacket(p, "DROP_DHZ");
        drop(p, 1, DROP_DATA_HOPS_LENGTH_EQUALS_ZERO);
        return;
    } else { // hdata->list_of_hops_length() > 0: the current node acts as relay.
        int i_ = hdata->pointer();
        if (hdata->list_of_hops()[i_] == ipAddr_) { // If I'm the right next hop.
            ch->next_hop() = hdata->list_of_hops()[i_ + 1];
            if (ch->next_hop() == 0 || i_ == hdata->list_of_hops_length() - 1) { // Reached the end of the list -> send directly to the sink.
                ch->next_hop() = iph->daddr();
            }
            ch->prev_hop_ = ipAddr_;
            hdata->pointer()++;
            num_data_tx_++;
            num_data_fw_++;
            if (trace_)
                this->tracePacket(p, "FRWD_DTA");
            sendDown(p, this->getDelay(delay_data_));
        } else {
            Packet::free(p);
        }
    }
} /* MSun::forwardDataPacket() */
