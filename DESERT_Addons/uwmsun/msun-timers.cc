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
 * @file   msun-timers.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of some of the timers of MSUN.
 *
 * Provides the implementation of some of the timers of MSUN.
 */

#include "msun.h"

void SearchPathTimer::expire(Event*) {
    module->searchPathExpire();
} /* SearchPathTimer::expire */

void MSun::searchPathExpire() {
    if (msun::STACK_TRACE)
        std::cout << "> searchPathExpire()" << std::endl;
    
    timer_search_path_enabled_ = true;
    return;
} /* MSun::searchPathExpire */

void BroadcastDuplicatePacketsTimer::expire(Event*) {
    module->broadcastDuplicatePacketsExpire();
} /* BroadcastDuplicatePacketsTimer::expire */

void MSun::broadcastDuplicatePacketsExpire() {
    if (msun::STACK_TRACE)
        std::cout << "> broadcastDuplicatePacketsExpire()" << std::endl;

    if (bcast_queue_.empty()) {
        return;
    } else {
        Packet* p         = bcast_queue_.front();
        hdr_cmn* ch       = HDR_CMN(p);
        hdr_uwip *iph     = HDR_UWIP(p);
        // hdr_uwcbr* uwcbrh = HDR_UWCBR(p);
        
        uint8_t ip_sink_ = 0;
        if (metric_ == msun::HOPCOUNT) {
            ip_sink_ = this->ipLowestHopCountPath();
        } else if (metric_ == msun::SNR) {
            ip_sink_ = this->ipMaxMinSNRPath();
        }
        
        if (ip_sink_ == 0) { // No path to any sink -> send a request.
            if (timer_search_path_enabled_) {
                this->searchPath(iph->daddr());
                timer_search_path_enabled_ = false;
                searchPathTmr_.resched(timer_search_path_ + delay_status_);
            }
            broadcastDuplicatePacketsTimer_.resched(timer_search_path_);
            return;
        } else { // Create several copies of the packet.
            std::map<uint8_t, route_container>::const_iterator it;
            for (it = routing_table.begin(); it != routing_table.end(); ++it) { // Create one copy of the packet for each destination.
                if (this->hcToIp(it->first) > 0) {
                    // Alloc a new packet, set the destination and store it.
                    iph->daddr() = it->first;
                    if (buffer_data.size() < buffer_max_size_) { // There is space to store the packet.
                        num_data_stored_++;
                        buffer_data.push_back(buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_BROADCAST, ipAddr_));
                    } else {
                        num_drop_buff_full_++;
                        if (trace_)
                            this->tracePacket(p, "DROP_BIF");
                        drop(p, 1, DROP_BUFFER_IS_FULL); // If the buffer is full, drop the original packet and stop with the process of generating packets.
                        bcast_queue_.pop();
                        return;
                    }
                }
            }
            Packet::free(p);
            bcast_queue_.pop();
            return;
        }
    }
    return;
} /* MSun::broadcastDuplicatePacketsExpire */

void ErrorPacketsTimer::expire(Event*) {
    module->errorPacketsExpire();
} /* BroadcastDuplicatePacketsTimer::expire */

void MSun::errorPacketsExpire() {
    if (msun::STACK_TRACE)
        std::cout << "> errorPacketsExpire()" << std::endl;
    
    timer_error_packets_enabled_ = true;
    return;
} /* MSun::errorPacketsExpire */
