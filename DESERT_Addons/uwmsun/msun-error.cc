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
 * @file   msun-error.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding Error Packets.
 *
 * Provides the implementation of all the methods regarding Error Packets.
 */

#include "msun.h"

void MSun::createRouteError(const Packet* p_data, Packet* p) { // p is a path establishment packet beacause it contains a list of hops.
    if (msun::STACK_TRACE)
        std::cout << "> createRouteError()" << std::endl;
    
    hdr_cmn* ch_data          = HDR_CMN(p_data);
    hdr_uwip* iph_data        = HDR_UWIP(p_data);
    hdr_msun_data* hdata      = HDR_MSUN_DATA(p_data);
    hdr_cmn* ch               = HDR_CMN(p);
    hdr_uwip* iph             = HDR_UWIP(p);
    hdr_msun_path_est* hpest  = HDR_MSUN_PATH_EST(p);
    
    this->initPktPathEstSearch(p);
    
    ch->prev_hop_       = ipAddr_;
    if (ch_data->prev_hop_ == iph_data->saddr()) { // Trick to fix the case in which the node that generates the error is one hop distant from the source.
        ch->next_hop()      = ch_data->prev_hop_;
    } else {
        ch->next_hop()      = hdata->list_of_hops()[hdata->pointer() - 1];
    }
    
    iph->saddr()        = ipAddr_;
    iph->daddr()        = iph_data->saddr();
    
    hpest->ptype()               = msun::PATH_ERROR;
    hpest->list_of_hops_length() = hdata->pointer(); // Not + 1 because the current node is excluded.
    hpest->pointer()             = 1;
    
    // Hops from the current node to the source of the data packet.
    int w_ = hdata->pointer() - 1;
    for (int i = 0; i < hdata->pointer(); ++i) {
        hpest->list_of_hops()[i] = hdata->list_of_hops()[w_];
        w_--;
    }

    // Hack to identify the broken link. TODO(giovanni): check it.
    hpest->list_of_hops()[msun::MAX_HOP_NUMBER + 1] = ch_data->next_hop();
} /* MSun::createRouteError */

void MSun::sendRouteErrorBack(Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> sendRouteErrorBack()" << std::endl;
    
    hdr_cmn* ch              = HDR_CMN(p);
    hdr_uwip* iph            = HDR_UWIP(p);
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);
    
    if (hpest->pointer() >= 0 && hpest->pointer() < hpest->list_of_hops_length()) {
        ch->next_hop() = hpest->list_of_hops()[hpest->pointer()];
    } else {
        ch->next_hop() = iph->daddr();
    }
    
    ch->prev_hop_ = ipAddr_;
    hpest->pointer()--;
    num_error_fw_++;
    
    if (trace_)
        this->tracePacket(p, "FRWD_ERR");
    sendDown(p, this->getDelay(delay_status_));
} /* MSun::sendRouteErrorBack */

void MSun::updateRoutingTableAfterError(const Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> updateRoutingTableAfterError()" << std::endl;
    
    hdr_uwip* iph            = HDR_UWIP(p);
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);
    
    
    if (!disable_route_error_) {
        uint8_t broken_link_node_1_ = iph->saddr();
        uint8_t broken_link_node_2_ = hpest->list_of_hops()[msun::MAX_HOP_NUMBER + 1];
        msun_updateRoutingTableAfterError_new_loop_label_:
        if (!routing_table.empty()) {
            for (std::map<uint8_t, route_container>::iterator it_rt(routing_table.begin()); it_rt != routing_table.end(); ++it_rt) { // For each entry.
                if (metric_ == msun::HOPCOUNT && !it_rt->second.hop_count_.list_hops_.empty()) {
                    for (std::vector<uint8_t>::iterator itv(it_rt->second.hop_count_.list_hops_.begin()); itv != it_rt->second.hop_count_.list_hops_.end(); ++itv) { // For each couple.
                        if (*itv == broken_link_node_1_ && *(itv + 1) == broken_link_node_2_) {  // Match: remove the entry.
                            it_rt->second.hop_count_.list_hops_.clear();
                            routing_table.erase(it_rt);
                            goto msun_updateRoutingTableAfterError_new_loop_label_;
                        }
                    }
                } else if (metric_ == msun::SNR && !it_rt->second.snr_.list_hops_.empty()) {
                    for (std::vector<uint8_t>::iterator itv(it_rt->second.snr_.list_hops_.begin()); itv != it_rt->second.snr_.list_hops_.end(); ++itv) { // For each couple.
                        if (*itv == broken_link_node_1_ && *(itv + 1) == broken_link_node_2_) {  // Match: remove the entry.
                            it_rt->second.snr_.list_hops_.clear();
                            routing_table.erase(it_rt);
                            goto msun_updateRoutingTableAfterError_new_loop_label_;
                        }
                    }
                }
            }
        }
    }
    return;
} /* MSun::removeEntryAfterError */
