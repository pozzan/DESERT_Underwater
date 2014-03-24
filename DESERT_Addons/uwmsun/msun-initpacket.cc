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
 * @file   msun-initpacket.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding the Initialization of Data Packet from upper layers.
 *
 * Provides the implementation of all the methods regarding the Initialization of Data Packet from upper layers.
 */

#include "msun.h"

int MSun::initPacketAnycast(Packet* _p) {
    if (msun::STACK_TRACE)
        std::cout << "> initPacketUnicast()" << std::endl;
    hdr_cmn* ch          = HDR_CMN(_p);
    hdr_uwip* iph        = HDR_UWIP(_p);
    hdr_msun_data* hdata = HDR_MSUN_DATA(_p);
    
    if (iph->daddr() == UWIP_BROADCAST || iph->daddr() == ipAddr_) { // Error: the destination is not a valid IP.
        return 0;
    }
    
    uint8_t ip_sink_ = 0;
    if (metric_ == msun::HOPCOUNT) {
        ip_sink_ = this->ipLowestHopCountPath();
    } else if (metric_ == msun::SNR) {
        ip_sink_ = this->ipMaxMinSNRPath();
    }
    
    int hop_to_sink_ = this->hcToIp(ip_sink_);
    if (hop_to_sink_ > 0) { // The validity of the timer was already checks in hpTpIp().
        // Common header.
        //ch->uid()       = 0; // Set by the Application above.
        //ch->ptype()     = 0; // Set by the Application above.
        //ch->size()      = 0; // Look Down;
        ch->direction()   = hdr_cmn::DOWN;
        //ch->next_hop()  = 0; // This value must be set by the invoker.
        ch->prev_hop_     = ipAddr_;

        // IP Header.
        iph->daddr()  = ip_sink_; // To set because it is decided by the protocol.
        //iph->dport()  = 0; // Set by the Application above.
        //iph->saddr()  = ipAddr_; // Already set.
        //iph->sport()  = 0; // Set by the Application above.

        // Path establishment header.
        hdata->list_of_hops_length() = hop_to_sink_ - 1;

        // i is incremented inside the for loop.
        int i = 0;
        if (metric_ == msun::HOPCOUNT) {
            for (std::vector<uint8_t>::iterator itv = routing_table[iph->daddr()].hop_count_.list_hops_.begin(); itv != routing_table[iph->daddr()].hop_count_.list_hops_.end(); ++itv, ++i) {
                hdata->list_of_hops()[i] = *itv;
            }
        } else if (metric_ == msun::SNR) {
            for (std::vector<uint8_t>::iterator itv = routing_table[iph->daddr()].snr_.list_hops_.begin(); itv != routing_table[iph->daddr()].snr_.list_hops_.end(); ++itv, ++i) {
                hdata->list_of_hops()[i] = *itv;
            }
        }

        hdata->pointer()            = 0; 

        if (hdata->list_of_hops_length() > 0) {
            ch->next_hop()          = hdata->list_of_hops()[0];
        } else {
            ch->next_hop()          = iph->daddr();
        }
        // Do not set here the size because this function is called several times for the same packet!
        // ch->size()                  += sizeof(hdr_msun_data);
        ch->timestamp()             = Scheduler::instance().clock();

        return hop_to_sink_;
    } else { // No valid path the the sink.
        return 0;
    }
} /* MSun::initPacketUnicast */

int MSun::initPacketUnicast(Packet* _p) {
    if (msun::STACK_TRACE)
        std::cout << "> initPacketUnicast()" << std::endl;
    hdr_cmn* ch          = HDR_CMN(_p);
    hdr_uwip* iph        = HDR_UWIP(_p);
    hdr_msun_data* hdata = HDR_MSUN_DATA(_p);
    
    if (iph->daddr() == UWIP_BROADCAST || iph->daddr() == 0 || iph->daddr() == ipAddr_) { // Error: the destination is not a valid IP.
        return 0;
    }
    
    int hop_to_sink_ = this->hcToIp(iph->daddr());
    if (hop_to_sink_ > 0) { // The validity of the timer was already checked in hpToIp().
        // Common header.
        //ch->uid()       = 0; // Set by the Application above.
        //ch->ptype()     = 0; // Set by the Application above.
        //ch->size()      = 0; // Look Down;
        ch->direction()   = hdr_cmn::DOWN;
        //ch->next_hop()  = 0; // Look Down;
        ch->prev_hop_     = ipAddr_;

        // IP Header.
        //iph->daddr()  = 0; // Already set.
        //iph->dport()  = 0; // Set by the Application above.
        //iph->saddr()  = ipAddr_; // Already set.
        //iph->sport()  = 0; // Set by the Application above.

        // Path establishment header.
        hdata->list_of_hops_length() = hop_to_sink_ - 1;

        int i = 0;
        if (metric_ == msun::HOPCOUNT) {
            for (std::vector<uint8_t>::iterator itv = routing_table[iph->daddr()].hop_count_.list_hops_.begin(); itv != routing_table[iph->daddr()].hop_count_.list_hops_.end(); ++itv, ++i) {
                hdata->list_of_hops()[i] = *itv;
            }
        } else if (metric_ == msun::SNR) {
            for (std::vector<uint8_t>::iterator itv = routing_table[iph->daddr()].snr_.list_hops_.begin(); itv != routing_table[iph->daddr()].snr_.list_hops_.end(); ++itv, ++i) {
                hdata->list_of_hops()[i] = *itv;
            }
        }
        hdata->pointer()            = 0; 

        if (hdata->list_of_hops_length() > 0) {
            ch->next_hop()          = hdata->list_of_hops()[0];
        } else {
            ch->next_hop()          = iph->daddr();
        }
        
        // Do not set here the size because this function is called several times for the same packet!
        // ch->size()                  += sizeof(hdr_msun_data);
        ch->timestamp()             = Scheduler::instance().clock();

        return hop_to_sink_;
    } else { // No valid path the the sink.
        return 0;
    }
} /* MSun::initPacketUnicast */

void MSun::initPktPathEstSearch(Packet* p, uint8_t _ip) const {
    if (msun::STACK_TRACE)
        std::cout << "> initPktPathEstSearch()" << std::endl;

    // Common header.
    hdr_cmn* ch     = HDR_CMN(p);
    ch->uid()       = msun::uid_++;
    ch->ptype()     = PT_MSUN_PATH_EST;
    //ch->size()    = 0; // Look down.
    ch->direction() = hdr_cmn::DOWN;
    ch->next_hop()  = UWIP_BROADCAST;
    ch->prev_hop_   = ipAddr_;
    
    // IP header.
    hdr_uwip* iph  = HDR_UWIP(p);
    iph->daddr()   = _ip; // Path search packet sent to a specific address.
    //iph->dport() = 0; // Not needed: no applications above.
    //iph->saddr() = 0; // Set by IPModule.
    //iph->sport() = 0; // Not needed: no applications above.
    //iph->ttl()   = 0; // Set by IPModule.
    
    // Path establishment header.
    hdr_msun_path_est* hpest      = HDR_MSUN_PATH_EST(p);
    hpest->ptype()                = msun::PATH_SEARCH;
    hpest->pointer()              = 0;
    hpest->list_of_hops_length()  = 0;
    for (int i = 0; i < msun::MAX_HOP_NUMBER; ++i) {
        hpest->list_of_hops()[i]  = 0;
    }
    if (metric_ == msun::HOPCOUNT) {
        hpest->quality()          = 0;
    } else if (metric_ == msun::SNR) {
        hpest->quality()          = - msun::MIN_SNR;
    }
    ch->size()                    += sizeof(hdr_msun_path_est);
    ch->timestamp()               = Scheduler::instance().clock();
} /* MSun::initPktPathEstSearch */

void MSun::initPktPathEstAnswer(Packet* new_pkt, const Packet* const old_pkt) {
    if (msun::STACK_TRACE)
        std::cout << "> initPktPathEstAnswer()" << std::endl;
    
    // Common header.
    hdr_cmn* ch_old      = HDR_CMN(old_pkt);
    hdr_cmn* ch_new      = HDR_CMN(new_pkt);
    ch_new->uid()        = msun::uid_++;
    ch_new->ptype()      = PT_MSUN_PATH_EST;
    //ch_new->size()     = 0; // Look Down.
    ch_new->direction()  = hdr_cmn::DOWN;
    //ch_new->next_hop() = 0; // Look Down.
    ch_new->prev_hop_    = ipAddr_;

    // IP Header
    hdr_uwip* iph_old    = HDR_UWIP(old_pkt);
    hdr_uwip* iph_new    = HDR_UWIP(new_pkt);
    iph_new->saddr()     = ipAddr_; // The relay with hop count 1 will set as source it's own IP.
    //iph_new->sport() = 0; // Set by the Application above.
    iph_new->daddr()   = iph_old->saddr(); // The daddr is the IP of the node that sent the path request.
    //iph_new->dport() = 0; // Set by the Application above.

    // Path establishment header.
    hdr_msun_path_est* hpest_new      = HDR_MSUN_PATH_EST(new_pkt);
    hdr_msun_path_est* hpest_old      = HDR_MSUN_PATH_EST(old_pkt);
    hpest_new->ptype()                = msun::PATH_ANSWER;
    hpest_new->list_of_hops_length()  = hpest_old->list_of_hops_length();
    hpest_new->pointer()              = hpest_new->list_of_hops_length() - 1; // -1 because the length starts from 1
//    std::cout << ">-----------begin initPathAnswer--------------<" << std::endl;
//    std::cout << "Node IP: " << printIP(ipAddr_) << std::endl;
//    std::cout << "Requesting node: " << printIP(iph_old->saddr()) << std::endl;
//    std::cout << "Receiving from: " << printIP(ch_old->prev_hop_) << std::endl;
//    std::cout << "List of hops length: " << static_cast<int>(hpest_old->list_of_hops_length()) << std::endl;
    for (int i = 0; i < hpest_new->list_of_hops_length(); ++i) {
        hpest_new->list_of_hops()[i] = hpest_old->list_of_hops()[i];
//        std::cout << "hop[" << i << "]: " << printIP(hpest_new->list_of_hops()[i]) << std::endl;
    }
//    std::cout << "Pointer to the next: " << static_cast<int>(hpest_new->pointer()) << std::endl;
    hpest_new->quality()             = hpest_old->quality();
    
    ch_new->size()                   += sizeof(hdr_msun_path_est);
    if (hpest_new->pointer() < 0) {
        ch_new->next_hop()           = ch_old->prev_hop_; //iph_old->saddr();
    } else {
        ch_new->next_hop()           = hpest_new->list_of_hops()[hpest_new->pointer()];
    }
//    std::cout << "Next hop node: " << printIP(ch_new->next_hop()) << std::endl;
//    std::cout << ">------------end initPathAnswer---------------<" << std::endl;
    ch_new->timestamp()              = Scheduler::instance().clock();
} /* MSun::initPktPathEstAnswer */
