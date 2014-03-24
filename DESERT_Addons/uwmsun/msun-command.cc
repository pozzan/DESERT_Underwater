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
 * @file   msun-command.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding the Command method.
 *
 * Provides the implementation of all the methods regarding the Command method.
 */

#include "msun.h"

void MSun::initialize() {
    if (msun::STACK_TRACE)
        std::cout << "> initialize()" << std::endl;
    
    if (ipAddr_ == 0) {
        UWIPClMsgReqAddr* m = new UWIPClMsgReqAddr(getId());
        m->setDest(CLBROADCASTADDR);
        sendSyncClMsgDown(m);
    }
} /* MSun::initialize */

void MSun::clearRoutingTable() {
    if (msun::STACK_TRACE)
        std::cout << "> clearRoutingTable()" << std::endl;
    
    routing_table.clear();
} /* MSun::clearRoutingTable */

void MSun::clearBuffer() {
    if (msun::STACK_TRACE)
        std::cout << "> clearBuffer()" << std::endl;
    
    buffer_data.clear();
} /* MSun::clearBuffer */

void MSun::clearBufferForwarded() {
    if (msun::STACK_TRACE)
        std::cout << "> clearBufferForwarded()" << std::endl;
    
    buffer_data_forward.clear();
} /* MSun::clearBufferForwarded */

void MSun::printLowestHopCount() const {
    if (msun::STACK_TRACE)
        std::cout << "> printHopCount()" << std::endl;
    
    std::map<uint8_t, route_container>::const_iterator it(routing_table.find(this->ipLowestHopCountPath()));
    if (it != routing_table.end() && (Scheduler::instance().clock() - it->second.hop_count_.timestamp_) < timer_route_validity_) {
        std::cout << "Node: " << this->printIP(ipAddr_) <<  " connected to sink: " << this->printIP(it->first) << " with # hops = " << it->second.hop_count_.quality_ << std::endl;
    } else {
        std::cout << "Node: " << this->printIP(ipAddr_) <<  " not connected to any sink." << std::endl;
    }
} /* Msun::printHopCount */

void MSun::printRoutingTable() const {
    if (msun::STACK_TRACE)
        std::cout << "> printRoutingTable()" << std::endl;
    std::vector<uint8_t>::const_iterator itv;
    
    for (std::map<uint8_t, route_container>::const_iterator it = routing_table.begin(); it != routing_table.end(); ++it) {
        if (it->first != 0) {
            std::cout << "Routing Table node: " << this->printIP(ipAddr_) << std::endl;
            std::cout << "-> Destination: " << this->printIP(it->first) << std::endl;
//            if (it->second.hop_count_.quality_ != 0) {
            if (!this->isZero(it->second.hop_count_.quality_)) {
                std::cout << "   - Shortest path -" << std::endl;
                std::cout << "   -> " << this->printIP(ipAddr_) << " - ";
                for (itv = it->second.hop_count_.list_hops_.begin(); itv != it->second.hop_count_.list_hops_.end(); ++itv) {
                    std::cout << this->printIP((uint8_t) *itv) << " - ";
                }
                std::cout << this->printIP(it->first) << std::endl;
                std::cout << "   - Number of hops: " << it->second.hop_count_.quality_ << std::endl;
                std::cout << "   - Timestamp: " << it->second.hop_count_.timestamp_ << std::endl;
            }

            if (!this->isZero(it->second.snr_.quality_ - msun::MIN_SNR)) {
                std::cout << " - MaxMin SNR path: " << std::endl;
                std::cout << " -> " << this->printIP(ipAddr_) << " - ";
                for (itv = it->second.snr_.list_hops_.begin(); itv != it->second.snr_.list_hops_.end(); ++itv) {
                    std::cout << this->printIP((uint8_t) *itv) << " - ";
                }
                std::cout << this->printIP(it->first) << std::endl;
                std::cout << " - Quality: " << it->second.snr_.quality_ << std::endl;
                std::cout << " - Timestamp: " << it->second.snr_.timestamp_ << std::endl;
            }
        }
    }
} /* MSun::printHopTable */

uint32_t MSun::getBuffersSize() const {
    if (msun::STACK_TRACE)
        std::cout << "> getBuffersSize()" << std::endl;
    
    return (buffer_data.size() + buffer_data_forward.size());
} /* MSun::getBuffersSize() */

double MSun::getMeanRetx() const {
    if (msun::STACK_TRACE)
        std::cout << "> getMeanRetx()" << std::endl;
    
    double num = double(num_data_tx_) - double(num_data_stored_ - this->getBuffersSize());
    double den = double(num_data_stored_ - this->getBuffersSize());
    if (num_data_stored_ == 0 || num_data_stored_ == this->getBuffersSize()) {
        return 0;
    } else {
        return std::max(double(num)/double(den), double(0));
    }
} /* MSun::getMeanRetx() */

string MSun::getStatsTx(const uint8_t& _ip) const {
    if (msun::STACK_TRACE)
        std::cout << "> getStatsTx()" << std::endl;
    
    std::map<uint8_t, std::vector<int> >::const_iterator it = data_tx_.find(_ip);
    std::stringstream out;
    
    if (it != data_tx_.end()) {
        for (int i = 0; i < msun::MAX_HOP_NUMBER; ++i) {
            out << it->second[i];
            if (i < msun::MAX_HOP_NUMBER - 1) {
                out << "\t";
            }
        }
        return out.str();
    } else {
        for (int i = 0; i < msun::MAX_HOP_NUMBER; ++i) {
            out << "0";
            if (i < msun::MAX_HOP_NUMBER - 1) {
                out << "\t";
            }
        }
        return out.str();
    }
} /* MSun::getStatsTx() */

string MSun::getStatsRx(const uint8_t& _ip) const {
    if (msun::STACK_TRACE)
        std::cout << "> getStatsRx()" << std::endl;
    
    std::map<uint8_t, std::vector<int> >::const_iterator it = data_rx_.find(_ip);
    std::stringstream out;
    
    if (it != data_rx_.end()) {
        for (int i = 0; i < msun::MAX_HOP_NUMBER; ++i) {
            out << it->second[i];
            if (i != msun::MAX_HOP_NUMBER - 1) {
                out << "\t";
            }
        }
        return out.str();
    } else {
        for (int i = 0; i < msun::MAX_HOP_NUMBER; ++i) {
            out << "0";
            if (i != msun::MAX_HOP_NUMBER - 1) {
                out << "\t";
            }
        }
        return out.str();
    }
} /* MSun::getStatsRx() */
