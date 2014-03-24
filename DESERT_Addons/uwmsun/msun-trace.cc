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
 * @file   msun-trace.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding the Tracing of packets.
 *
 * Provides the implementation of all the methods regarding the Tracing of packets.
 */

#include "msun.h"

void MSun::tracePacket(const Packet* const p, const string& position) {
    if (msun::STACK_TRACE)
        std::cout << "> tracePacket()" << std::endl;
    
    hdr_uwip* iph            = HDR_UWIP(p);
    hdr_cmn* ch              = HDR_CMN(p);
    hdr_msun_ack* hack       = HDR_MSUN_ACK(p);
    hdr_uwcbr* uwcbrh        = HDR_UWCBR(p);
    
    /* Output list summary:
     * 1) Clock
     * 2) ipAddr
     * 3) ch->uid()
     * 4) cbr->sn()
     * 5) ch->prev_hop_
     * 6) ch->next_hop_
     * 7) iph->saddr
     * 8) iph->daddr
     * 9) ch->direction
     * 10) ch->ptype
     */
    if (trace_) {
        if (ch->ptype() == PT_MSUN_ACK) {
            this->writeInTrace(this->createTraceString(position, Scheduler::instance().clock(), ipAddr_, ch->uid(), hack->uid(), ch->prev_hop_, ch->next_hop(), iph->saddr(), iph->daddr(), ch->direction(), ch->ptype()));
        } else {
            this->writeInTrace(this->createTraceString(position, Scheduler::instance().clock(), ipAddr_, ch->uid(), uwcbrh->sn(), ch->prev_hop_, ch->next_hop(), iph->saddr(), iph->daddr(), ch->direction(), ch->ptype()));
        }
    }
} /* MSun::tracePacket */

const string MSun::createTraceString(const string& info_string, const double& simulation_time_, const int& node_id_, const int& pkt_id_, const int& pkt_sn_, const int& pkt_from_, const int& pkt_next_hop, const int& pkt_source_, const int& pkt_destination_, const int& direction_, const int& pkt_type) const {
    if (msun::STACK_TRACE)
        std::cout << "> createTraceString()" << std::endl;
    
    std::stringstream osstream_;
    osstream_ << info_string << this->trace_separator_ << simulation_time_ << this->trace_separator_ << (node_id_ & 0x000000ff) << this->trace_separator_ << (pkt_id_ & 0x0000ffff) << this->trace_separator_ << (pkt_sn_ & 0x0000ffff) << this->trace_separator_ << (pkt_from_ & 0x000000ff) << this->trace_separator_ << (pkt_next_hop & 0x000000ff) << this->trace_separator_ << (pkt_source_ & 0x000000ff) << this->trace_separator_ << (pkt_destination_ & 0x000000ff) << this->trace_separator_ << direction_ << this->trace_separator_ << pkt_type;
    return osstream_.str();
} /* MSun::createTraceString */

void MSun::writeInTrace(const string& string_to_write_) {
    if (msun::STACK_TRACE)
        std::cout << "> writeInTrace()" << std::endl;
    
    trace_file_.open(trace_file_name_, fstream::app);
    if (trace_file_.is_open()) {
        trace_file_ << string_to_write_ << std::endl;
        trace_file_.close();
    } else {
        std::cerr << "Unable to open " << trace_file_name_ << std::endl;
    }
} /* MSun::writeInTrace */

void MSun::writePathInTrace(const Packet* p) {
    if (msun::STACK_TRACE)
        cout << "> writePathInTrace()" << endl;
    
    hdr_uwip* iph           = HDR_UWIP(p);
    hdr_cmn* ch             = HDR_CMN(p);
    hdr_msun_data* hdata    = HDR_MSUN_DATA(p);
    
    std::stringstream osstream_;
    osstream_ << Scheduler::instance().clock() << '\t' << ch->uid() << '\t' << hdata->list_of_hops_length() + 1 << '\t' << (iph->saddr() & 0x000000ff);
    for (int i = 0; i < hdata->list_of_hops_length(); i++) {
        osstream_ << '\t' << (hdata->list_of_hops()[i] & 0x000000ff);
    }
    osstream_ << '\t' << (iph->daddr() & 0x000000ff);

    trace_file_path_.open(trace_file_path_name_, fstream::app);
    if (trace_file_path_.is_open()) {
        trace_file_path_ << osstream_.str() << endl;
        trace_file_path_.close();
    } else {
        std::cerr << "Unable to open " << trace_file_path_name_ << std::endl;
    }
} /*  MSun::writePathInTrace */
