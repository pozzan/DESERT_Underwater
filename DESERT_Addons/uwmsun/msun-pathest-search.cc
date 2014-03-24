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
 * @file   msun-pathest-search.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding Path Establishment Search Packets.
 *
 * Provides the implementation of all the methods regarding Path Establishment Search Packets.
 */

#include "msun.h"

extern packet_t PT_MSUN_DATA;
extern packet_t PT_MSUN_PATH_EST;

void MSun::searchPath(const uint8_t& _ip) {
    if (msun::STACK_TRACE)
        std::cout << "> searchPath()" << std::endl;
    
    // Create and send a new path establishment packet.
    Packet* p = Packet::alloc();
    this->initPktPathEstSearch(p, _ip);
    num_pathest_search_tx_++;
    if (trace_)
        this->tracePacket(p, "SEND_SRC");
    sendDown(p, this->getDelay(delay_status_));
    
    return;
} /* MSun::searchPath */

void MSun::forwardPathEstSearch(Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> forwardPathEstSearch()" << std::endl;
    
    hdr_cmn* ch              = HDR_CMN(p);
    hdr_uwip* iph            = HDR_UWIP(p);
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);
    
    if (this->getSinr(p) >= min_sinr_path_request_) {
        if (hpest->ptype() == msun::PATH_SEARCH) {
            if (iph->saddr() != ipAddr_) { // Am I the source of the request?
                if (iph->daddr() == UWIP_BROADCAST) {
                    if (enable_sink_) { // I am a sink.
                        num_pathest_search_me_++;
                        this->updateQuality(p);

                        std::map<uint8_t, pathest_reply_element>::iterator it(pathest_reply_table.find(iph->saddr()));
                        if (it == pathest_reply_table.end()) {
                            pathest_reply_element tmp_container_;
                            if (metric_ == msun::HOPCOUNT) {
                                tmp_container_.quality_last_path_search_ = 0;
                                tmp_container_.time_last_path_search_    = 0;
                            } else if (metric_ == msun::SNR) {
                                tmp_container_.quality_last_path_search_ = msun::MIN_SNR;
                                tmp_container_.time_last_path_search_    = 0;
                            }
                            pathest_reply_table.insert(pair<uint8_t, pathest_reply_element>(iph->saddr(), tmp_container_));
                        }
                        it = pathest_reply_table.find(iph->saddr());

                        // I am a sink: do I have to evaluate the packet before answering it?
                        // -> No because the packet contain info about the reversed path.
                        // Send back an answer.
                        bool old_path_outdated_ = (Scheduler::instance().clock() - it->second.time_last_path_search_) > timer_answer_path_;
                        bool new_path_is_better_ = false;
                        double quality_new_path_;

                        if (metric_ == msun::HOPCOUNT) {
                            quality_new_path_ = hpest->list_of_hops_length();
                            if (old_path_outdated_) {
                                new_path_is_better_       = true;
                                it->second.quality_last_path_search_ = quality_new_path_;
                                it->second.time_last_path_search_    = Scheduler::instance().clock();
                            } else {
                                if (quality_new_path_ < it->second.quality_last_path_search_ || this->isZero(it->second.quality_last_path_search_)) {
                                    new_path_is_better_       = true;
                                    it->second.quality_last_path_search_ = quality_new_path_;
                                    it->second.time_last_path_search_    = Scheduler::instance().clock();
                                } else {
                                    new_path_is_better_ = false;
                                }
                            }
                        } else if (metric_ == msun::SNR) {
                            quality_new_path_ = hpest->quality();
                            if (old_path_outdated_) {
                                new_path_is_better_       = true;
                                it->second.quality_last_path_search_ = quality_new_path_;
                                it->second.time_last_path_search_    = Scheduler::instance().clock();
                            } else {
                                if (quality_new_path_ > it->second.quality_last_path_search_ || this->isZero(it->second.quality_last_path_search_ + msun::MIN_SNR)) {
                                    new_path_is_better_       = true;
                                    it->second.quality_last_path_search_ = quality_new_path_;
                                    it->second.time_last_path_search_    = Scheduler::instance().clock();
                                } else {
                                    new_path_is_better_ = false;
                                }
                            }
                        } else {
                            Packet::free(p);
                            return;
                        }

                        if (new_path_is_better_) {
                            this->answerPath(p);
                        }

                        // Forward the request.
                        if (this->isMyIpInList(p)) { // My Ip is already in list or the packet is returned to the source: drop.
                            if (trace_)
                                this->tracePacket(p, "DROP_PSA");
                            drop(p, 1, DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_ALREADY_PROCESSED);
                            return;
                        } else { // This is the first time that the current node sees this request.
                            if (this->addMyIpInList(p) == false) { // The queue is full, impossible to add my IP.
                                if (trace_)
                                    this->tracePacket(p, "DROP_PSF");
                                drop(p, 1, DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_HOP_LIST_FULL);
                                return;
                            } else { // The IP of the current node has been added to the queue.
                                // this->updateQuality(p); Not required, already done
                                ch->next_hop() = UWIP_BROADCAST;
                                ch->prev_hop_  = ipAddr_;
                                num_pathest_search_fw_++;
                                if (trace_)
                                    this->tracePacket(p, "FRWD_SRC");
                                sendDown(p, this->getDelay(delay_status_));
                                return;
                            }
                        }
                    } else {
                        if (this->isMyIpInList(p)) { // My Ip is already in list or the packet is returned to the source: drop.
                            if (trace_)
                                this->tracePacket(p, "DROP_PSA");
                            drop(p, 1, DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_ALREADY_PROCESSED);
                            return;
                        } else { // This is the first time that the current node sees this request.
                            if (this->addMyIpInList(p) == false) { // The queue is full, impossible to add my IP.
                                if (trace_)
                                    this->tracePacket(p, "DROP_PSF");
                                drop(p, 1, DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_HOP_LIST_FULL);
                                return;
                            } else { // The IP of the current node has been added to the queue.
                                this->updateQuality(p);
                                ch->next_hop() = UWIP_BROADCAST;
                                ch->prev_hop_  = ipAddr_;
                                num_pathest_search_fw_++;
                                if (trace_)
                                    this->tracePacket(p, "FRWD_SRC");
                                sendDown(p, this->getDelay(delay_status_));
                                return;
                            }
                        }
                    }
                } else if (iph->daddr() != 0) { // Specific IP, check always that the IP is != 0
                    if (iph->daddr() == ipAddr_ && enable_sink_) { // I am the designated sink: update the quality and create an answer.
                        num_pathest_search_me_++;
                        this->updateQuality(p);

                        std::map<uint8_t, pathest_reply_element>::iterator it(pathest_reply_table.find(iph->saddr()));
                        if (it == pathest_reply_table.end()) {
                            pathest_reply_element tmp_container_;
                            if (metric_ == msun::HOPCOUNT) {
                                tmp_container_.quality_last_path_search_ = 0;
                                tmp_container_.time_last_path_search_    = 0;
                            } else if (metric_ == msun::SNR) {
                                tmp_container_.quality_last_path_search_ = msun::MIN_SNR;
                                tmp_container_.time_last_path_search_    = 0;
                            }
                            pathest_reply_table.insert(pair<uint8_t, pathest_reply_element>(iph->saddr(), tmp_container_));
                        }
                        it = pathest_reply_table.find(iph->saddr());

                        // I am a sink: do I have to evaluate the packet before answering it?
                        // -> No because the packet contain info about the reversed path.
                        // Send back an answer.
                        bool old_path_outdated_ = (Scheduler::instance().clock() - it->second.time_last_path_search_) > timer_answer_path_;
                        bool new_path_is_better_ = false;
                        double quality_new_path_;

                        if (metric_ == msun::HOPCOUNT) {
                            quality_new_path_ = hpest->list_of_hops_length();
                            if (old_path_outdated_) {
                                new_path_is_better_       = true;
                                it->second.quality_last_path_search_ = quality_new_path_;
                                it->second.time_last_path_search_    = Scheduler::instance().clock();
                            } else {
                                if (quality_new_path_ < it->second.quality_last_path_search_ || this->isZero(it->second.quality_last_path_search_)) {
                                    new_path_is_better_       = true;
                                    it->second.quality_last_path_search_ = quality_new_path_;
                                    it->second.time_last_path_search_    = Scheduler::instance().clock();
                                } else {
                                    new_path_is_better_ = false;
                                }
                            }
                        } else if (metric_ == msun::SNR) {
                            quality_new_path_ = hpest->quality();
                            if (old_path_outdated_) {
                                new_path_is_better_       = true;
                                it->second.quality_last_path_search_ = quality_new_path_;
                                it->second.time_last_path_search_    = Scheduler::instance().clock();
                            } else {
                                if (quality_new_path_ > it->second.quality_last_path_search_ || this->isZero(it->second.quality_last_path_search_ + msun::MIN_SNR)) {
                                    new_path_is_better_       = true;
                                    it->second.quality_last_path_search_ = quality_new_path_;
                                    it->second.time_last_path_search_    = Scheduler::instance().clock();
                                } else {
                                    new_path_is_better_ = false;
                                }
                            }
                        } else {
                            Packet::free(p);
                            return;
                        }

                        if (new_path_is_better_) {
                            this->answerPath(p);
                        }

                        Packet::free(p);
                        return;
                    } else if (iph->daddr() == ipAddr_ && !enable_sink_) { // The request is for me but I am not a sink.
                        if (trace_)
                            this->tracePacket(p, "DROP_NAS");
                        drop(p, 1, DROP_I_AM_NOT_A_SINK);
                        return;
                    } else { // I am not the designated sink, forward the request.
                        if (this->isMyIpInList(p)) { // My Ip is already in list or the packet is returned to the source: drop.
                            if (trace_)
                                this->tracePacket(p, "DROP_PSA");
                            drop(p, 1, DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_ALREADY_PROCESSED);
                            return;
                        } else {
                            if (this->addMyIpInList(p) == false) { // The queue is full, impossible to add my IP.
                                if (trace_)
                                    this->tracePacket(p, "DROP_PSF");
                                drop(p, 1, DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_HOP_LIST_FULL);
                                return;
                            } else { // The IP of the current node has been added to the queue.
                                this->updateQuality(p);
                                ch->next_hop() = UWIP_BROADCAST;
                                ch->prev_hop_  = ipAddr_;
                                num_pathest_search_fw_++;
                                if (trace_)
                                    this->tracePacket(p, "FRWD_SRC");
                                sendDown(p, this->getDelay(delay_status_));
                                return;
                            }
                        }
                    }
                } else {
                    Packet::free(p);
                    return;
                }
            } else {
                Packet::free(p);
                return;
            }
        } else {
            Packet::free(p);
            return;
        }
    } else {
        if (trace_)
            this->tracePacket(p, "DROP_PMS");
        drop(p, 1, DROP_PATH_ESTABLISHMENT_MIN_SINR_NOT_REACHED);
        return;
    }
} /* MSun::forwardPathEstSearch */

bool MSun::addMyIpInList(Packet* p) const {
    if (msun::STACK_TRACE)
        std::cout << "> addMyIpInList()" << std::endl;
    
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);

    if (hpest->pointer() >= msun::MAX_HOP_NUMBER) { // Impossible to add new hops.
        return false;
    } else {
        // Time to add current node IP at the end of the list.
        hpest->list_of_hops()[hpest->pointer()] = ipAddr_;
        hpest->list_of_hops_length()++;
        hpest->pointer()++;
        return true;
    }
} /* MSun::addMyIpInList */

bool MSun::isMyIpInList(const Packet* p) const {
    if (msun::STACK_TRACE)
        std::cout << "> isMyIpInList()" << std::endl;
    
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);

    for (int i = 0; i < hpest->list_of_hops_length(); ++i) {
        if (hpest->list_of_hops()[i] == ipAddr_)
            return true;
    }
    return false;
} /* MSun::isMyIpInList */

bool MSun::isMyIpInListData(const Packet* p) const {
    if (msun::STACK_TRACE)
        std::cout << "> isMyIpInListData()" << std::endl;
    
    hdr_msun_data* hdata = HDR_MSUN_DATA(p);

    for (int i = 0; i < hdata->list_of_hops_length(); ++i) {
        if (hdata->list_of_hops()[i] == ipAddr_)
            return true;
    }
    return false;
} /* MSun::isMyIpInListData */

void MSun::updateQuality(Packet* p) const {
    if (msun::STACK_TRACE)
        std::cout << "> updateQuality()" << std::endl;
    
    hdr_msun_path_est* hpest = HDR_MSUN_PATH_EST(p);
    
    if (metric_ == msun::HOPCOUNT) {
        hpest->quality() += 1;
    } else if (metric_ ==  msun::SNR) {
        double sinr_ = this->getSinr(p);
        if (sinr_ < hpest->quality()) {
            hpest->quality() = sinr_;
        }
    } else {
        cerr << "The metric_ field of SUN was not set up." << std::endl;
    }
} /* MSun::updateQuality */
