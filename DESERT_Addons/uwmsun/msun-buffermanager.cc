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
 * @file   msun-buffermanager.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding Buffer Management.
 *
 * Provides the implementation of all the methods regarding Buffer Management.
 */

#include "msun.h"

void BufferTimer::expire(Event*) {
    module->bufferManager();
} /* BufferTimer::expire */

void MSun::bufferManager() {
    if (msun::STACK_TRACE)
        std::cout << "> bufferManager()" << std::endl;
    
    double prob_int_buff;
    if ((buffer_data.size() + buffer_data_forward.size()) > 0) { // If there is at least one element in one of the two buffers compute the probability.
        prob_int_buff = std::max(msun::PROC_INT_BUFF, static_cast<double>(buffer_data.size()) / (static_cast<double>(buffer_data.size()) + static_cast<double>(buffer_data_forward.size())));
    } else {
        prob_int_buff = 1;
    }
    
    double random_num = RNG::defaultrng()->uniform_double(); // Random number between 0 and 1.
    std::vector<buffer_element>* buffer;
    
    /* If the random number is smaller than the probability of process the internal buffer
     * then process the internal buffer, otherwise process a packet to forward.
     */
    if (random_num <= prob_int_buff) {
        buffer = &buffer_data;
        if (buffer->empty()) {
            buffer = &buffer_data_forward;
        }
    } else {
        buffer = &buffer_data_forward;
        if (buffer->empty()) {
            buffer = &buffer_data;
        }
    }
    
    // The delay_tx_ variable is used to add a delay in transmission for Data Packets.
    double delay_tx_ = this->getDelay(delay_data_);
    if (!buffer->empty()) { // There is at least one pkt in the buffer, otherwise resched the lecture of the buffer.
        buffer_element& buffer_head_ = buffer->front(); // Reference to the first element in the buffer.
        if (buffer_head_.num_retx_ <= num_retx_) { // The first pkt is valid.
            // hdr_cmn* ch          = HDR_CMN(buffer_head_.p_);
            hdr_uwip* iph        = HDR_UWIP(buffer_head_.p_);
            hdr_msun_data* hdata = HDR_MSUN_DATA(buffer_head_.p_);
            
            /*
             * The three possible addresses type are considered separately:
             * - msun::MSUN_BROADCAST
             * - msun::MSUN_ANYCAST
             * - msun::MSUN_UNICAST
             */
            if (buffer_head_.destination_ == msun::MSUN_BROADCAST) { // Only if restricted_broadcast_ was enabled.
                if ((iph->saddr() & 0x000000ff) == (ipAddr_ & 0x000000ff)) { // The current node is the source of the packet.
                    /*
                     * Problem with the destination address: drop the packet.
                     */
                    if ((iph->daddr() & 0x000000ff) == UWIP_BROADCAST || (iph->daddr() & 0x000000ff) == 0) {
                        if (trace_)
                            this->tracePacket(buffer_head_.p_, "DROP_EBP");
                        drop(buffer_head_.p_, 1, DROP_ERROR_BCAST_PACKET);
                        buffer->erase(buffer->begin()); // Remove the first pkt.
                        bufferTmr_.resched(min_buffer_tmr_);
                        return;
                    } else {
                        /*
                         * If the path to the destination still exists then update the packet and send it.
                         */
                        if (this->initPacketUnicast(buffer_head_.p_) > 0) {
                            buffer_head_.num_retx_++;
                            buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                            // Update the statistic about the hop count of each packet sent.
                            std::map<uint8_t, std::vector<int> >::iterator it(data_tx_.find(iph->daddr()));
                            if (it != data_tx_.end()) {
                                it->second[hdata->list_of_hops_length()]++;
                            } else {
                                std::vector<int> tmp_(msun::MAX_HOP_NUMBER + 1, 0);
                                tmp_[hdata->list_of_hops_length()] = 1;
                                data_tx_.insert(pair<uint8_t, std::vector<int> >(iph->daddr(), tmp_));
                            }
                            num_data_tx_++;
                            if (trace_)
                                this->tracePacket(buffer_head_.p_, "SEND_DTA");
                            sendDown(buffer_head_.p_->copy(), delay_tx_);
                            Packet::free(buffer_head_.p_);
                            buffer->erase(buffer->begin());
                            bufferTmr_.resched(timer_buffer_ + delay_tx_);
                            return;
                        }
                        /*
                         * If the path to the destination does not exist any more: drop the packet.
                         */
                        else {
                            if (trace_)
                                this->tracePacket(buffer_head_.p_, "DROP_BDO");
                            drop(buffer_head_.p_, 1, DROP_BCAST_DST_OUTDATED);
                            buffer->erase(buffer->begin()); // Remove the first pkt.
                            bufferTmr_.resched(min_buffer_tmr_);
                            return;
                        }
                    }
                } else { // A node can not receive msun::MSUN_BROADCAST packets from other nodes if restricted_broadcast_ was enabled => drop the packet.
                    if (trace_)
                        this->tracePacket(buffer_head_.p_, "DROP_EBP");
                    drop(buffer_head_.p_, 1, DROP_ERROR_BCAST_PACKET);
                    buffer->erase(buffer->begin()); // Remove the first pkt.
                    bufferTmr_.resched(min_buffer_tmr_);
                    return;
                }
                std::cerr << Scheduler::instance().clock() << "> Warning: something wrong might have happened in MSun::bufferManager()." << std::endl;
                bufferTmr_.resched(min_buffer_tmr_);
                return;
            } else if (buffer_head_.destination_ == msun::MSUN_ANYCAST) {
                /*
                 * The current node is the source of the packet, or it is a relay and the path in the
                 * header of the packet is not valid.
                 */
                if ((iph->saddr() & 0x000000ff) == (ipAddr_ & 0x000000ff) || ((iph->saddr() & 0x000000ff) != (ipAddr_ & 0x000000ff) && !buffer_head_.valid_packet_)) {
                    uint8_t ip_sink_ = 0;
                    if (metric_ == msun::HOPCOUNT) {
                        ip_sink_ = this->ipLowestHopCountPath();
                    } else if (metric_ == msun::SNR) {
                        ip_sink_ = this->ipMaxMinSNRPath();
                    }
                    /*
                     * No valid destination in the routing table.
                     */
                    if (ip_sink_ == 0) { // No entry.
                        double time_to_wait = min_buffer_tmr_;
                        if (timer_search_path_enabled_) {
                            this->searchPath(UWIP_BROADCAST); // Node not connected with the destination: send a path establishment request.
                            timer_search_path_enabled_ = false;
                            searchPathTmr_.resched(timer_search_path_);
                            time_to_wait = wait_path_answer_ + delay_status_;
                            buffer_head_.num_retx_++;
                            buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                        }
                        bufferTmr_.resched(time_to_wait); // Wait the maximum time necessary to retrieve a path plus the maximum delay used for status packets.
                        return;
                    } else { // Valid entry.
                        this->initPacketAnycast(buffer_head_.p_);
                        buffer_head_.num_retx_++;
                        buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                        // Update the statistic about the hop count of each packet sent.
                        if ((iph->saddr() & 0x000000ff) == (ipAddr_ & 0x000000ff)) {
                            std::map<uint8_t, std::vector<int> >::iterator it(data_tx_.find(iph->daddr()));
                            if (it != data_tx_.end()) {
                                it->second[hdata->list_of_hops_length()]++;
                            } else {
                                std::vector<int> tmp_(msun::MAX_HOP_NUMBER + 1, 0);
                                tmp_[hdata->list_of_hops_length()] = 1;
                                data_tx_.insert(pair<uint8_t, std::vector<int> >(iph->daddr(), tmp_));
                            }
                            num_data_tx_++;
                            if (trace_)
                                this->tracePacket(buffer_head_.p_, "SEND_DTA");
                        } else {
                            if (trace_)
                                this->tracePacket(buffer_head_.p_, "FRWD_DTA");
                        }
                        sendDown(buffer_head_.p_->copy(), delay_tx_);
                        bufferTmr_.resched(timer_buffer_ + delay_tx_);
                        return;
                    }
                }
                /*
                 * I am not the source and the original path is valid. Just forward the packet.
                 */
                else if ((iph->saddr() & 0x000000ff) != (ipAddr_ & 0x000000ff) && buffer_head_.valid_packet_) {
                    buffer_head_.num_retx_++;
                    buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                    this->forwardDataPacket(buffer_head_.p_->copy());
                    bufferTmr_.resched(timer_buffer_ + delay_tx_);
                    return;
                } else {
                    std::cerr << Scheduler::instance().clock() << "> Warning: something wrong might have happened in MSun::bufferManager()." << std::endl;
                    bufferTmr_.resched(min_buffer_tmr_);
                    return;
                }
            } else if (buffer_head_.destination_ == msun::MSUN_UNICAST) {
                /*
                 * The current node is the source of the packet, or it is a relay and the path in the
                 * header of the packet is not valid.
                 */
                if ((iph->saddr() & 0x000000ff) == (ipAddr_ & 0x000000ff) || ((iph->saddr() & 0x000000ff) != (ipAddr_ & 0x000000ff) && !buffer_head_.valid_packet_)) {
                    std::map<uint8_t, route_container>::iterator it_map(routing_table.find(iph->daddr()));
                    /*
                     * Check if there is an entry in the routing table to the destination.
                     */
                    if (it_map == routing_table.end()) { // No entry.
                        double time_to_wait = min_buffer_tmr_;
                        if (timer_search_path_enabled_) {
                            this->searchPath(iph->daddr()); // Node not connected with the destination: send a path establishment request.
                            timer_search_path_enabled_ = false;
                            searchPathTmr_.resched(timer_search_path_);
                            time_to_wait = wait_path_answer_ + delay_status_;
                            buffer_head_.num_retx_++;
                            buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                        }
                        bufferTmr_.resched(time_to_wait); // Wait the maximum time necessary to retrieve a path plus the maximum delay used for status packets.
                        return;
                    } else { // Entry.
                        /*
                         * Check if the timestamp of the entry is valid.
                         */
                        bool is_valid_entry_ = false;
                        if (metric_ == msun::HOPCOUNT) {
                            if (this->testValidTimestamp(it_map->second.hop_count_.timestamp_)) {
                                is_valid_entry_ = true;
                            }
                        } else if (metric_ == msun::SNR) {
                            if (this->testValidTimestamp(it_map->second.snr_.timestamp_)) {
                                is_valid_entry_ = true;
                            }
                        }

                        if (!is_valid_entry_) {
                            double time_to_wait = min_buffer_tmr_;
                            if (timer_search_path_enabled_) {
                                this->searchPath(iph->daddr()); // Node not connected with the destination: send a path establishment request.
                                timer_search_path_enabled_ = false;
                                searchPathTmr_.resched(timer_search_path_);
                                time_to_wait = wait_path_answer_ + delay_status_;
                                buffer_head_.num_retx_++;
                                buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                            }
                            bufferTmr_.resched(time_to_wait); // Wait the maximum time necessary to retrieve a path plus the maximum delay used for status packets.
                            return;
                        } else {
//                            std::cout << Scheduler::instance().clock() << "> " << static_cast<uint32_t>(ipAddr_) << " - Valid entry" << std::endl;
                            this->initPacketUnicast(buffer_head_.p_);
                            buffer_head_.num_retx_++;
                            buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                            // Update the statistic about the hop count of each packet sent.
                            if ((iph->saddr() & 0x000000ff) == (ipAddr_ & 0x000000ff)) {
                                std::map<uint8_t, std::vector<int> >::iterator it(data_tx_.find(iph->daddr()));
                                if (it != data_tx_.end()) {
                                    it->second[hdata->list_of_hops_length()]++;
                                } else {
                                    std::vector<int> tmp_(msun::MAX_HOP_NUMBER + 1, 0);
                                    tmp_[hdata->list_of_hops_length()] = 1;
                                    data_tx_.insert(pair<uint8_t, std::vector<int> >(iph->daddr(), tmp_));
                                }
                                num_data_tx_++;
                                if (trace_)
                                    this->tracePacket(buffer_head_.p_, "SEND_DTA");
                            } else {
                                if (trace_)
                                    this->tracePacket(buffer_head_.p_, "FRWD_DTA");
                            }
                            sendDown(buffer_head_.p_->copy(), delay_tx_);
                            bufferTmr_.resched(timer_buffer_ + delay_tx_);
                            return;
                        }
                    }
                }
                /*
                 * I am not the source and the original path is valid. Just forward the packet.
                 */
                else if ((iph->saddr() & 0x000000ff) != (ipAddr_ & 0x000000ff) && buffer_head_.valid_packet_) {
                    buffer_head_.num_retx_++;
                    buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                    this->forwardDataPacket(buffer_head_.p_->copy());
                    bufferTmr_.resched(timer_buffer_ + delay_tx_);
                    return;
                } else {
                    std::cerr << Scheduler::instance().clock() << "> Warning: something wrong might have happened in MSun::bufferManager()." << std::endl;
                    bufferTmr_.resched(min_buffer_tmr_);
                    return;
                }
            } else { // Wrong type of packet, drop it.
//                std::cout << "> Wrong type of packet. Dropped." << std::endl;
                if (trace_)
                    this->tracePacket(buffer_head_.p_, "DROP_WTP");
                drop(buffer_head_.p_, 1, DROP_WRONG_MSUN_DST_TYPE);
                buffer->erase(buffer->begin()); // Remove the first pkt.
                bufferTmr_.resched(min_buffer_tmr_);
                return;
            }
            std::cerr << Scheduler::instance().clock() << "> Warning: something wrong might have happened in MSun::bufferManager()." << std::endl;
            bufferTmr_.resched(min_buffer_tmr_);
            return;
        } else { // The first packet in the buffer is invalid.
            hdr_cmn* ch          = HDR_CMN(buffer_head_.p_);
            hdr_uwip* iph        = HDR_UWIP(buffer_head_.p_);
            /*
             * Dynamic timer_buffer_ computation.
             * Increases the value of timer_buffer_.
             */
            if (adaptive_timer_buffer_) {
                timer_buffer_ = std::min(max_buffer_tmr_, timer_buffer_ + (1 - alpha_data_) * timer_buffer_);
                timer_buffer_ = std::max(min_buffer_tmr_, timer_buffer_);
            }
            
            /*
             * The head packet in the buffer generated an error => the link with the next hop is broken.
             * Search for all the destination in the routing table if the route contains
             * that broken_link. If yes remove the entry.
             */
            if (!disable_route_error_) {
                msun_buffermanager_invalid_packet_new_loop_:
                if (!routing_table.empty()) {
                    for (std::map<uint8_t, route_container>::iterator it_rt(routing_table.begin()); it_rt != routing_table.end(); ++it_rt) {
                        if (metric_ == msun::HOPCOUNT && !it_rt->second.hop_count_.list_hops_.empty()) {
                            for (std::vector<uint8_t>::iterator itv(it_rt->second.hop_count_.list_hops_.begin()); itv != it_rt->second.hop_count_.list_hops_.end(); ++itv) {
                                if (*itv == ch->next_hop()) { // Match. This entry must be removed.
                                    routing_table.erase(it_rt);
                                    goto msun_buffermanager_invalid_packet_new_loop_;
                                }
                            }
                        } else if (metric_ == msun::SNR && !it_rt->second.snr_.list_hops_.empty()) {
                            for (std::vector<uint8_t>::iterator itv(it_rt->second.snr_.list_hops_.begin()); itv != it_rt->second.snr_.list_hops_.end(); ++itv) {
                                if (*itv == ch->next_hop()) { // Match. This entry must be removed.
                                    routing_table.erase(it_rt);
                                    goto msun_buffermanager_invalid_packet_new_loop_;
                                }
                            }
                        }
                    }
                }
            }
            
            /*
             * The current node is the source of the data packet. Never send
             * back an error but update directly the routing table.
             */
            if ((iph->saddr() & 0x000000ff) == (ipAddr_ & 0x000000ff)) {
                /*
                 * Reset all the packets in the buffers with the same next_hop of the
                 * packet that generated an error. Reset them only if the flag disable_route_error_
                 * is false.
                 */
                if (!disable_route_error_) {
                    for (std::vector<buffer_element>::iterator itv(buffer_data.begin()); itv != buffer_data.end(); ++itv) {
                        Packet* packet_to_check_ = itv->p_;
                        hdr_cmn* ch_to_check_    = HDR_CMN(packet_to_check_);
                        if (ch_to_check_->next_hop() == ch->next_hop()) {
                            itv->valid_packet_ = false;
                        }
                    }
                    for (std::vector<buffer_element>::iterator itv(buffer_data_forward.begin()); itv != buffer_data_forward.end(); ++itv) {
                        Packet* packet_to_check_ = itv->p_;
                        hdr_cmn* ch_to_check_    = HDR_CMN(packet_to_check_);
                        if (ch_to_check_->next_hop() == ch->next_hop()) {
                            itv->valid_packet_ = false;
                        }
                    }
                }
                /*
                 * Packet with error generated by me.
                 * In case of msun::MSUN_BROADCAST drop the packet.
                 */
                if (buffer_head_.destination_ == msun::MSUN_BROADCAST) {
                    num_drop_maxretx_++;
                    if (trace_)
                        this->tracePacket(buffer_head_.p_, "DROP_MTX");
                    drop(buffer_head_.p_, 1, DROP_MAX_RETX);
                    buffer->erase(buffer->begin()); // Remove the first pkt.
                    bufferTmr_.resched(min_buffer_tmr_);
                    return;
                }
                /*
                 * In case of msun::MSUN_ANYCAST or msun::MSUN_UNICAST if enabled
                 * reset the stats of the packet otherwise drop it.
                 */
                else if (buffer_head_.destination_ == msun::MSUN_ANYCAST || buffer_head_.destination_ == msun::MSUN_UNICAST) {
                    /*
                     * Help the packet: reset it and send again.
                     */
                    if (prevent_from_drop_) {
                        if (!disable_route_error_) { // Reset the route.
                            buffer_head_.valid_packet_ = false;
                            buffer_head_.num_retx_ = 0;
                            buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                            bufferTmr_.resched(min_buffer_tmr_);
                            return;
                        } else { // Use the old route.
                            buffer_head_.valid_packet_ = true;
                            buffer_head_.num_retx_ = 0;
                            buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                            bufferTmr_.resched(min_buffer_tmr_);
                            return;
                        }
                    } else { // Drop the packet.
                        num_drop_maxretx_++;
                        if (trace_)
                            this->tracePacket(buffer_head_.p_, "DROP_MTX");
                        drop(buffer_head_.p_, 1, DROP_MAX_RETX);
                        buffer->erase(buffer->begin()); // Remove the first pkt.
                        bufferTmr_.resched(min_buffer_tmr_);
                        return;
                    }
                }
                /*
                 * Something went wrong, no destination type set up for packets generated
                 * by the current node: drop it.
                 */
                else {
                    std::cerr << Scheduler::instance().clock() << "> Warning -: something wrong might have happened in MSun::bufferManager()." << std::endl;
                    num_drop_maxretx_++;
                    if (trace_)
                        this->tracePacket(buffer_head_.p_, "DROP_MTX");
                    drop(buffer_head_.p_, 1, DROP_MAX_RETX);
                    buffer->erase(buffer->begin()); // Remove the first pkt.
                    bufferTmr_.resched(min_buffer_tmr_);
                    return;
                }
            } else if ((iph->saddr() & 0x000000ff) != (ipAddr_ & 0x000000ff)) { // I am not the source, update the packet if possible and send back an error. The packet is always in UNICAST for relays.
                /*
                 * If the <i>Path Error</i> mechanism is enabled send an error.
                 */
                if (!disable_path_error_) {
                    if (timer_error_packets_enabled_ && buffer_head_.valid_packet_) { // Send an error back only if the packet is valid => the path in the header was not reset.
                        Packet* p_error = Packet::alloc();
                        this->createRouteError(buffer_head_.p_, p_error);
                        num_error_tx_++;
                        timer_error_packets_enabled_ = false;
                        errorPacketsTimer_.resched(timer_error_packets_);
                        if (trace_)
                            this->tracePacket(p_error, "SEND_ERR");
                        sendDown(p_error->copy(), this->getDelay(delay_status_));
                        Packet::free(p_error);
                    }
                }
                /*
                 * Reset all the packets in the buffers with the same next_hop of the
                 * packet that generated an error. Reset them only if the flag disable_route_error_
                 * is false. In case of a relay node, resets the packets only after signaling the error.
                 */
                if (!disable_route_error_) {
                    for (std::vector<buffer_element>::iterator itv(buffer_data.begin()); itv != buffer_data.end(); ++itv) {
                        Packet* packet_to_check_ = itv->p_;
                        hdr_cmn* ch_to_check_    = HDR_CMN(packet_to_check_);
                        if (ch_to_check_->next_hop() == ch->next_hop()) {
                            itv->valid_packet_ = false;
                        }
                    }
                    for (std::vector<buffer_element>::iterator itv(buffer_data_forward.begin()); itv != buffer_data_forward.end(); ++itv) {
                        Packet* packet_to_check_ = itv->p_;
                        hdr_cmn* ch_to_check_    = HDR_CMN(packet_to_check_);
                        if (ch_to_check_->next_hop() == ch->next_hop()) {
                            itv->valid_packet_ = false;
                        }
                    }
                }
                /*
                 * Erase the packet from the buffer or save it?
                 */
                if (prevent_from_drop_) {
                    if (!disable_route_error_) { // Reset the route.
                        buffer_head_.valid_packet_ = false;
                        buffer_head_.num_retx_ = 0;
                        buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                        bufferTmr_.resched(min_buffer_tmr_);
                        return;
                    } else { // Use the old route.
                        buffer_head_.valid_packet_ = true;
                        buffer_head_.num_retx_ = 0;
                        buffer_head_.t_last_tx_ = Scheduler::instance().clock();
                        bufferTmr_.resched(min_buffer_tmr_);
                        return;
                    }
                } else { // Drop the packet.
                    num_drop_maxretx_++;
                    if (trace_)
                        this->tracePacket(buffer_head_.p_, "DROP_MTX");
                    drop(buffer_head_.p_, 1, DROP_MAX_RETX);
                    buffer->erase(buffer->begin()); // Remove the first pkt.
                    bufferTmr_.resched(min_buffer_tmr_);
                    return;
                }
            }
            std::cerr << Scheduler::instance().clock() << "> Warning: something wrong might have happened in MSun::bufferManager()." << std::endl;
            bufferTmr_.resched(timer_buffer_ + delay_tx_);
            return;
        }
    } else {
        /* If the buffer is empty wait the minimum time allowed.
         * Usually (timer_buffer_) gives better throughput (because of the lower congestion)
         * but higher forward trip time.
         */
        bufferTmr_.resched(min_buffer_tmr_); 
        return;
    }
}
