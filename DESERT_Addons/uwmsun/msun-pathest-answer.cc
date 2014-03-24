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
 * @file   msun-pathest-answer.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding Path Establishment Answer Packets.
 *
 * Provides the implementation of all the methods regarding Path Establishment Answer Packets.
 */

#include "msun.h"

extern packet_t PT_MSUN_DATA;
extern packet_t PT_MSUN_PATH_EST;

/* This function is invoked by a node with hop count = 1.
 * This function creates an Path Establishment Answer packet and, using other functions it sends
 * the packet back.
 */
void MSun::answerPath(const Packet* const p_old) {
    if (msun::STACK_TRACE)
        std::cout << "> answerPath()" << std::endl;
    
    if (enable_sink_) { // Create an answer only if the node is a sink.
        Packet* p_answer         = Packet::alloc();
        this->initPktPathEstAnswer(p_answer, p_old);
        // hdr_cmn* ch_answer       = HDR_CMN(p_answer);
        // hdr_uwip* iph            = HDR_UWIP(p_answer);
        // hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p_answer);
        num_pathest_answer_tx_++;
        if (trace_)
            this->tracePacket(p_answer, "SEND_PTH");
        sendDown(p_answer, this->getDelay(delay_status_));
        return;
    } else {
        return;
    }
} /* MSun::answerPath */

void MSun::forwardPathEstAnswer(Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> sendRouteBack()" << std::endl;
    
    hdr_cmn* ch              = HDR_CMN(p);
    hdr_uwip* iph            = HDR_UWIP(p);
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);

    if (hpest->ptype() == msun::PATH_ANSWER) { // This function processes only msun::PATH_ANSWER packets
        if (hpest->list_of_hops_length() == 0) { // The packet must be processed not forwarded.
            if (trace_)
                this->tracePacket(p, "DROP_PAG");
            drop(p, 1, DROP_PATH_ESTABLISHMENT_ANSWER_PACKET_GARBAGE);
            return;
        } else {
            short j_ = hpest->pointer();
            if (hpest->list_of_hops()[j_] == ipAddr_) { // The current node is the right next hop.
                // Update the hop table of the current node if required.
                this->evaluateForwardedPathEstAnswer(p);
                // Update the information in the Path Establishment Answer packet.
                hpest->pointer()--;
                if (hpest->pointer() < 0) { // The list is over, the next hop will be the destination.
                    ch->next_hop() = iph->daddr();
                    ch->prev_hop_  = ipAddr_;
                } else { // Look in the hop list for the next hop.
                    ch->next_hop() = hpest->list_of_hops()[hpest->pointer()];
                    ch->prev_hop_  = ipAddr_;
                }
                num_pathest_answer_fw_++;
                if (trace_)
                    this->tracePacket(p, "FRWD_PTH");
                sendDown(p, this->getDelay(delay_status_));
                return;
            } else {
                if (trace_)
                    this->tracePacket(p, "DROP_PAG");
                drop(p, 1, DROP_PATH_ESTABLISHMENT_ANSWER_PACKET_GARBAGE);
                return;
            }
        }
    } else {
        Packet::free(p);
        return;
    }
} /* MSun::forwardPathEstAnswer */

// Important: only the transmitter should use this function, not the relays!
int MSun::evaluatePath(const Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> evaluatePath()" << std::endl;
    
    // hdr_cmn* ch              = HDR_CMN(p);
    hdr_uwip* iph            = HDR_UWIP(p);
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);
    
    /*
     * These two if statements are used to exclude from the evaluation a specific
     * set Path Answer packets in order to avoid strange loops.
     */
    std::vector<buffer_element>* buffer;
    buffer = &buffer_data;
    if (!buffer->empty()) {
        buffer_element& buffer_head_ = buffer->front();
        for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
            // Avoids strange loops
            if ((ipAddr_ & 0x000000ff) == hpest->list_of_hops()[i]) {
                return 0;
            }
            // Do not forward to a node a packet that it generated.
            if (buffer_head_.source_ == hpest->list_of_hops()[i]) {
                return 0;
            }
        }
    }
    buffer = &buffer_data_forward;
    if (!buffer->empty()) {
        buffer_element& buffer_head_ = buffer->front();
        for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
            // Avoids strange loops
            if ((ipAddr_ & 0x000000ff) == hpest->list_of_hops()[i]) {
                return 0;
            }
            // Do not forward to a node a packet that it generated.
            if (buffer_head_.source_ == hpest->list_of_hops()[i]) {
                return 0;
            }
        }
    }
    
    if (metric_ == msun::HOPCOUNT) {
        int current_hop_count_ = this->hcToIp(iph->saddr());
        std::map<uint8_t, route_container>::iterator it = routing_table.find(iph->saddr());
        /*
         * New sink: add the path without any further check.
         */
        if (it == routing_table.end()) {
            route_container tmp_container_;
            for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
                tmp_container_.hop_count_.list_hops_.push_back(hpest->list_of_hops()[i]);
            }
            tmp_container_.hop_count_.quality_   = hpest->list_of_hops_length() + 1;
            tmp_container_.hop_count_.timestamp_ = Scheduler::instance().clock();
            tmp_container_.snr_.list_hops_.clear();
            tmp_container_.snr_.quality_         = msun::MIN_SNR;
            tmp_container_.snr_.timestamp_       = 0;
            routing_table.insert(pair<uint8_t, route_container>(iph->saddr(), tmp_container_));
        }
        /*
         * And old path exists. Update the route if:
         * the hop count value of the new route is lower
         * ||
         * the hop count of the old one is expired.
         */
        else if ((hpest->list_of_hops_length() < current_hop_count_) ||
                current_hop_count_ == 0) {
            route_entry tmp_;
            for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
                tmp_.list_hops_.push_back(hpest->list_of_hops()[i]);
            }
            tmp_.quality_   = hpest->list_of_hops_length() + 1;
            tmp_.timestamp_ = Scheduler::instance().clock();
            routing_table[iph->saddr()].hop_count_ = tmp_;
        }
        /*
         * The packet does not contain a better path than the old one: do nothing.
         */
        else {
            return 0;
        }
        return this->hcToIp(iph->saddr());
    } else if (metric_ == msun::SNR) {
        int current_snr_ = this->snrToIp(iph->saddr());
        std::map<uint8_t, route_container>::iterator it = routing_table.find(iph->saddr());
        /*
         * New sink: add the path without any further check.
         */
        if (it == routing_table.end()) { // New sink: add the new destination.
            route_container tmp_container_;
            for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
                tmp_container_.snr_.list_hops_.push_back(hpest->list_of_hops()[i]);
            }
            tmp_container_.snr_.quality_   = hpest->quality();
            tmp_container_.snr_.timestamp_ = Scheduler::instance().clock();
            tmp_container_.hop_count_.list_hops_.clear();
            tmp_container_.hop_count_.quality_         = 0;
            tmp_container_.hop_count_.timestamp_       = 0;
            routing_table.insert(pair<uint8_t, route_container>(iph->saddr(), tmp_container_));
        }
        /*
         * Check that:
         * the quality of the new route is higher or equal to the old one
         * &&
         * the value of quality is not the default one
         * && the number of hops 
         * ||
         * there are no valid routes initialized to that destination.
         */
        else if ((hpest->quality() >= current_snr_ &&
                !this->isZero(hpest->quality() + msun::MIN_SNR)) ||
                this->isZero(current_snr_ + msun::MIN_SNR)) {
            
            /*
             * If the two quality values are equal, update the route only if the number of relays in the new route is lower.
             * Tweak required when simulating with Urick and nodes placed at the same distance.
             */
            if (this->isZero(hpest->quality() - current_snr_) && hpest->list_of_hops_length() >= routing_table[iph->saddr()].snr_.list_hops_.size()) {
                ;
            } else {
                route_entry tmp_;
                for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
                    tmp_.list_hops_.push_back(hpest->list_of_hops()[i]);
                }
                tmp_.quality_   = hpest->quality();
                tmp_.timestamp_ = Scheduler::instance().clock();
                routing_table[iph->saddr()].snr_ = tmp_;
            }
        }
        /*
         * The packet does not contain a better path than the old one: do nothing.
         */
        else {
            return 0;
        }
        return this->hcToIp(iph->saddr());
    } else {
        return 0;
    }
} /* MSun::evaluatePath */

void MSun::evaluateForwardedPathEstAnswer(const Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> evaluateForwardedPathEstAnswer()" << std::endl;
    
    // hdr_cmn* ch              = HDR_CMN(p);
    hdr_uwip* iph            = HDR_UWIP(p);
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);
    
    /*
     * These two if statements are used to exclude from the evaluation a specific
     * set Path Answer packets in order to avoid strange loops.
     * 
     * NOTE: commented because the performance degrades. The following lines
     * are enables only in MSun::evaluatePathEstAnswer.
     */
//    std::vector<buffer_element>* buffer;
//    buffer = &buffer_data;
//    if (!buffer->empty()) {
//        buffer_element& buffer_head_ = buffer->front();
//        for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
//            // Do not forward to a node a packet that it generated.
//            if (buffer_head_.source_ == hpest->list_of_hops()[i]) {
//                return;
//            }
//        }
//    }
//    buffer = &buffer_data_forward;
//    if (!buffer->empty()) {
//        buffer_element& buffer_head_ = buffer->front();
//        for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
//            // Do not forward to a node a packet that it generated.
//            if (buffer_head_.source_ == hpest->list_of_hops()[i]) {
//                return;
//            }
//        }
//    }
    
    int16_t j_ = hpest->pointer();
    if (metric_ == msun::HOPCOUNT) {
        int current_hop_count_ = this->hcToIp(iph->saddr());
        std::map<uint8_t, route_container>::iterator it = routing_table.find(iph->saddr());
        /*
         * New sink: add the path without any further check.
         */
        if (it == routing_table.end()) {
            route_container tmp_container_;
            for (int i = j_ + 1; i < hpest->list_of_hops_length(); ++i) {
                tmp_container_.hop_count_.list_hops_.push_back(hpest->list_of_hops()[i]);
            }
            tmp_container_.hop_count_.quality_   = hpest->list_of_hops_length() - hpest->pointer() - 1;;
            tmp_container_.hop_count_.timestamp_ = Scheduler::instance().clock();
            tmp_container_.snr_.list_hops_.clear();
            tmp_container_.snr_.quality_         = msun::MIN_SNR;
            tmp_container_.snr_.timestamp_       = 0;
            routing_table.insert(pair<uint8_t, route_container>(iph->saddr(), tmp_container_));
        }
        /*
         * And old path exists. Update the route if:
         * the hop count value of the new route is lower
         * ||
         * the hop count of the old one is expired.
         */
        else if (hpest->list_of_hops_length() - hpest->pointer() <= current_hop_count_ ||
                current_hop_count_ == 0) {
            route_entry tmp_;
            for (int i = j_ + 1; i < hpest->list_of_hops_length(); ++i) {
                tmp_.list_hops_.push_back(hpest->list_of_hops()[i]);
            }
            tmp_.quality_   = hpest->list_of_hops_length() - hpest->pointer();
            tmp_.timestamp_ = Scheduler::instance().clock();
            routing_table[iph->saddr()].hop_count_ = tmp_;
        }
        /*
         * The packet does not contain a better path than the old one: do nothing.
         */
        else {
        }
    } else if (metric_ == msun::SNR) {
        int current_snr_ = this->snrToIp(iph->saddr());
        std::map<uint8_t, route_container>::iterator it = routing_table.find(iph->saddr());
        /*
         * New sink: add the path without any further check.
         */
        if (it == routing_table.end()) {
            route_container tmp_container_;
            for (int i = j_ + 1; i < hpest->list_of_hops_length(); ++i) {
                tmp_container_.snr_.list_hops_.push_back(hpest->list_of_hops()[i]);
            }
            tmp_container_.snr_.quality_   = hpest->quality();
            tmp_container_.snr_.timestamp_ = Scheduler::instance().clock();
            tmp_container_.hop_count_.list_hops_.clear();
            tmp_container_.hop_count_.quality_         = 0;
            tmp_container_.hop_count_.timestamp_       = 0;
            routing_table.insert(pair<uint8_t, route_container>(iph->saddr(), tmp_container_));
        }
        /*
         * Check that:
         * the quality of the new route is higher or equal to the old one
         * &&
         * the value of quality is not the default one
         * && the number of hops 
         * ||
         * there are no valid routes initialized to that destination.
         */
        else if ((hpest->quality() >= current_snr_ &&
                !this->isZero(hpest->quality() + msun::MIN_SNR)) ||
                this->isZero(current_snr_ + msun::MIN_SNR)) {
            /*
             * If the two quality values are equal, update the route only if the number of relays in the new route is lower.
             * Tweak required when simulating with Urick and nodes placed at the same distance.
             */
            if (this->isZero(hpest->quality() - current_snr_) && hpest->list_of_hops_length() >= routing_table[iph->saddr()].snr_.list_hops_.size()) {
            } else {
                route_entry tmp_;
                for (int i = j_ + 1; i < hpest->list_of_hops_length(); ++i) {
                    tmp_.list_hops_.push_back(hpest->list_of_hops()[i]);
                }
                tmp_.quality_   = hpest->quality();
                tmp_.timestamp_ = Scheduler::instance().clock();
                routing_table[iph->saddr()].snr_ = tmp_;
            }
        }
        /*
         * The packet does not contain a better path than the old one: do nothing.
         */
        else {
        }
    }
} /* MSun::evaluateForwardedPathEstAnswer */
