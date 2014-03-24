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
 * @file   msun-routetable.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding the Routing Table.
 *
 * Provides the implementation of all the methods regarding the Routing Table.
 */

#include "msun.h"

uint8_t MSun::ipLowestHopCountPath() const {
    if (msun::STACK_TRACE)
        std::cout << "> ipLowestHopCountPath()" << std::endl;
    
    uint8_t best_ip_ = 0;
    if (metric_ == msun::HOPCOUNT) {
        int old_best_hopcount_     = INT_MAX;
        for (std::map<uint8_t, route_container>::const_iterator it = routing_table.begin(); it != routing_table.end(); ++it) {
            const route_entry* itv(&it->second.hop_count_);
            /* If
             * (the new SNR is > the old best)
             * and
             * (the new quality is not the default min value)
             * and
             * (the timestamp of the new quality is valid)
             * then
             * (update the best)
             */
            if (itv->quality_ <= old_best_hopcount_ && !this->isZero(itv->quality_) && this->testValidTimestamp(itv->timestamp_)) {
                best_ip_           = it->first;
                old_best_hopcount_ = itv->quality_;
            }
        }
    } else if (metric_ == msun::SNR) {
        uint32_t old_best_hopcount_ = INT_MAX;
        for (std::map<uint8_t, route_container>::const_iterator it = routing_table.begin(); it != routing_table.end(); ++it) {
            const route_entry* itv(&it->second.snr_);
            /* If
             * (the new SNR is > the old best)
             * and
             * (the new quality is not the default min value)
             * and
             * (the timestamp of the new quality is valid)
             * then
             * (update the best)
             */
            if (itv->list_hops_.size() + 1 <= old_best_hopcount_ && !this->isZero(itv->quality_ + msun::MIN_SNR) && this->testValidTimestamp(itv->timestamp_)) {
                best_ip_            = it->first;
                old_best_hopcount_  = itv->quality_;
            }
        }
    }
    
    return best_ip_;
} /* MSun::ipLowestHopCountPath */
    
uint8_t MSun::ipMaxMinSNRPath() const {
    if (msun::STACK_TRACE)
        std::cout << "> ipMaxMinSNRPath()" << std::endl;
    
    uint8_t best_ip_ = 0;
    if (metric_ == msun::HOPCOUNT) {
        std::cerr << "MSun::snrToIp() with msun::HOPCOUNT metric." << std::endl;
    } else if (metric_ == msun::SNR) {
        double old_best_snr_        = msun::MIN_SNR;
        for (std::map<uint8_t, route_container>::const_iterator it = routing_table.begin(); it != routing_table.end(); ++it) {
            const route_entry* itv = &it->second.snr_;
            /* If
             * (the new SNR is > the old best)
             * and
             * (the new quality is not msun::MIN_SNR)
             * and
             * (the timestamp of the new quality is valid)
             * then
             * (update the best)
             */
            if (itv->quality_ > old_best_snr_ && !this->isZero(itv->quality_ + msun::MIN_SNR) && this->testValidTimestamp(itv->timestamp_)) {
                best_ip_            = it->first;
                old_best_snr_       = itv->quality_;
            }
        }
    }
    
    return best_ip_;
} /* MSun::ipMaxMinSNRPath */

bool MSun::testValidTimestamp(const double& _time) const {
    if (msun::STACK_TRACE)
        std::cout << "> testValidTimestamp()" << std::endl;
    
    return (Scheduler::instance().clock() - _time <= timer_route_validity_);
} /* MSun::testValidTimestamp */

int MSun::hcToIp(const uint8_t& _ip) const {
    if (msun::STACK_TRACE)
        std::cout << "> hcToIp()" << std::endl;
    
    std::map<uint8_t, route_container>::const_iterator it = routing_table.find(_ip);
    
    if (it != routing_table.end()) { // There is an entry in the routing table for the specific IP
        if (metric_ == msun::HOPCOUNT) {
            if (this->testValidTimestamp(it->second.hop_count_.timestamp_)) {
                return it->second.hop_count_.quality_;
            } else {
                return 0;
            }
        } else if (metric_ == msun::SNR) {
            if (this->testValidTimestamp(it->second.snr_.timestamp_)) {
                // list_hops_ contains the list of relays. The hop count values is the size of the list + 1.
                return it->second.snr_.list_hops_.size() + 1;
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    } else {
        return 0;
    }
} /* MSun::hcToIp */

double MSun::snrToIp(const uint8_t& _ip) const {
    if (msun::STACK_TRACE)
        std::cout << "> snrToIp()" << std::endl;
    
    if (metric_ == msun::HOPCOUNT) {
        std::cerr << "MSun::snrToIp() with HOPCOUNT metric." << std::endl;
        return msun::MIN_SNR;
    } else if (metric_ == msun::SNR) {
        std::map<uint8_t, route_container>::const_iterator it = routing_table.find(_ip);
        if (it != routing_table.end()) { // There is an entry in the routing table for the specific IP
            if (this->testValidTimestamp(it->second.snr_.timestamp_)) {
                return it->second.snr_.quality_;
            } else {
                return msun::MIN_SNR;
            }
        } else {
            return msun::MIN_SNR;
        }
    } else {
        std::cerr << "MSun::with UNKNOWN metric." << std::endl;
        return msun::MIN_SNR;
    }
} /* MSun::snrToIp */
