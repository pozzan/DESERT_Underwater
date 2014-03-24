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
 * @file   msun-utility.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of other methods used by MSUN.
 *
 * Provides the implementation of other methods used by MSUN.
 */

#include "msun.h"

string MSun::printIP(const nsaddr_t& _ip) const {
    if (msun::STACK_TRACE)
        std::cout << "> printIP()" << std::endl;
    
    this->printIP(static_cast<uint8_t>(_ip));
} /* MSun::printIP */

string MSun::printIP(const uint8_t& _ip) const {
    if (msun::STACK_TRACE)
        std::cout << "> printIP()" << std::endl;
    
    std::stringstream out;
    out << ((_ip & 0x000000ff));
    return out.str();
} /* MSun::printIP */

string MSun::printIP(const ns_addr_t& _ipt) const {
    return MSun::printIP(_ipt.addr_);
} /* MSun::printIP */

int MSun::getIndexInPacket(const Packet* _p) const {
    if (msun::STACK_TRACE)
        std::cout << "> getIndexInPacket()" << std::endl;
    
    hdr_msun_data* hdata = HDR_MSUN_DATA(_p);
    
    if (this->isMyIpInListData(_p)) {
        for (int i = 0; i < hdata->list_of_hops_length(); ++i) {
        if (hdata->list_of_hops()[i] == ipAddr_)
            return i;
        }
        return 0;
    } else {
        return 0;
    }
} /* MSun::getIndexFromPacket */

int MSun::getPlusNumRetx(const Packet* _p) const {
    if (msun::STACK_TRACE)
        std::cout << "> getPlusNumRetx()" << std::endl;
    
    hdr_msun_data* hdata = HDR_MSUN_DATA(_p);
    
    int index_ = this->getIndexInPacket(_p);
    /*
     * If the function getIndexInPacket found the id of the current node:
     * - check if the value returned is valid
     * - if so calculate the number of retx to add
     * 
     * before_ is the number nodes that already forwarded the packet.
     * after_ is the number of nodes that have still to forward the packet.
     * 
     * High values for before_ means that the network already spent a lot of
     * efforts to send to me the packet.
     * 
     * The returned value is a linear combination of these two parameters.
     */
    if (index_ != 0 || index_ < hdata->list_of_hops_length()) {
        int before_ = index_;
        int after_  = hdata->list_of_hops_length() - before_ + 1;
        
        if (after_ == 0) {
            return 0;
        }
        double return_value_ = before_ - static_cast<double>(before_) / static_cast<double>(after_);
        
        if (return_value_ >= msun::THRESHOLD_RETX) {
            return ceil(return_value_);
        } else {
            return 0;
        }
    } else {
        return 0;
    }
} /* MSun::getPlusNumRetx */

void MSun::printIdsPkts() const {
    std::cout << "Msun packets IDs:" << std::endl;
    std::cout << "PT_MSUN_ACK: \t\t" << PT_MSUN_ACK << std::endl;
    std::cout << "PT_MSUN_DATA: \t\t" << PT_MSUN_DATA << std::endl;
    std::cout << "PT_MSUN_BROADCASTDATA: \t" << PT_MSUN_BROADCASTDATA << std::endl;
    std::cout << "PT_MSUN_PATH_EST: \t" << PT_MSUN_PATH_EST << std::endl;
} /* MSun::printIdsPkts */
