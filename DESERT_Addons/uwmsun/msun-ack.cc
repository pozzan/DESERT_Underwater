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
 * @file   msun-ack.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Provides the implementation of all the methods regarding Ack Packets.
 *
 * Provides the implementation of all the methods regarding Ack Packets.
 */

#include "msun.h"

/* This function uses the value contained in the header of a uwcbr packet.
 * TODO: try to modifies this in order to use the ch->uid() field instead.
 */
void MSun::sendBackAck(const Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> sendBackAck()" << std::endl;
    
    hdr_cmn* ch          = HDR_CMN(p);
    hdr_uwip* iph        = HDR_UWIP(p);
    // hdr_msun_data* hdata = HDR_MSUN_DATA(p);
    // hdr_uwcbr* uwcbrh    = HDR_UWCBR(p);
    
    Packet* p_ack        = Packet::alloc();
    this->initPktAck(p_ack);
    
    hdr_cmn* ch_ack      = HDR_CMN(p_ack);
    hdr_uwip* iph_ack    = HDR_UWIP(p_ack);
    hdr_msun_ack* hack   = HDR_MSUN_ACK(p_ack);
    
    ch_ack->next_hop()   = ch->prev_hop_;
    ch_ack->prev_hop_    = ipAddr_;
    iph_ack->daddr()     = ch->prev_hop_;
    iph_ack->saddr()     = ipAddr_;
    hack->saddr()        = iph->saddr();
    hack->uid()          = static_cast<uint16_t>(ch->uid());

    num_ack_tx_++;
    if (trace_)
        this->tracePacket(p_ack, "SEND_ACK");
    sendDown(p_ack, this->getDelay(delay_status_));
} /* MSun::sendBackAck */

void MSun::initPktAck(Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> initPktAck()" << std::endl;

    // Common header.
    hdr_cmn* ch     = HDR_CMN(p);
    ch->uid()       = msun::uid_++;
    ch->ptype()     = PT_MSUN_ACK;
    // ch->size()    = 0; // Look down.
    ch->direction() = hdr_cmn::DOWN;
    // ch->next_hop()  = 0;
    ch->prev_hop_   = ipAddr_;
    
    // IP header.
    hdr_uwip* iph   = HDR_UWIP(p);
    // iph->daddr()  = 0;
    // iph->dport()  = 0;
    iph->saddr()    = ipAddr_;
    // iph->sport()  = 0;
    
    ch->size()                  += sizeof(hdr_msun_ack);
    ch->timestamp()             = Scheduler::instance().clock();
} /* MSun::initPktAck */

void MSun::processAck(Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> processAck()" << std::endl;
    
    hdr_cmn* ch              = HDR_CMN(p);
    hdr_msun_ack* hack       = HDR_MSUN_ACK(p);
    
    /*
     * The current node received an ack. Check the source of the data
     * packet that generated that ack and decide in which buffer
     * the packet to ack belongs.
     */
    std::vector<buffer_element>* buffer;
    if (hack->saddr() == ipAddr_) {
        buffer = &buffer_data;
    } else {
        buffer = &buffer_data_forward;
    }
    
    // Check the buffers and remove all the packets acked.
    if (!buffer->empty()) { // There is at least one packet in the buffer.
        buffer_element& buffer_head_ = buffer->front();
        if (hack->uid() == buffer_head_.id_pkt_ && hack->saddr() == buffer_head_.source_) { // Ack for the first packet in the buffer.
            num_data_acked_++;
            if (adaptive_timer_buffer_) {
                /*
                 * The node received a valid ack. Than means:
                 * - good quality of the channel
                 * - the node are closer
                 * The protocol tries to be more aggressive.
                 * The new delay_data_ will be the max between
                 * - the round trip time of the ack
                 * - the scaled value of delay_data_
                 */
                double ftt_ = (Scheduler::instance().clock() - ch->timestamp()) * 2;
                timer_buffer_ = timer_buffer_ * (1 - alpha_data_) + ftt_ * (alpha_data_);
                if (timer_buffer_ + timer_buffer_ < ftt_) {
                    timer_buffer_ = ftt_ - timer_buffer_;
                }
                timer_buffer_ = std::max(min_buffer_tmr_, timer_buffer_);
            }
            Packet::free(buffer_head_.p_);
            buffer->erase(buffer->begin()); // Remove the acked packet.
        }
        /*
         * Check in the buffer if there are multiple copies of the
         * acked packet. If so remove them.
         */
        if (!buffer->empty()) {
            for (std::vector<buffer_element>::iterator it = buffer->begin(); it != buffer->end(); /* Nothing */) {
                if (!buffer->empty() && hack->uid() == it->id_pkt_ && hack->saddr() == it->source_) {
                    Packet::free(it->p_);
                    buffer->erase(it); // Remove the acked packet.
                } else {
                    ++it;
                }
            }
        }
    }
} /* MSun::processAck */
