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
 * @file   msun-pkts.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Packets' class implementation.
 * 
 */

#include "msun-hdr-ack.h"
#include "msun-hdr-data.h"
#include "msun-hdr-pathest.h"
#include "msun-hdr-broadcastdata.h"

#include <tclcl.h>

int hdr_msun_ack::offset_ = 0;                /**< Offset used to access in <i>hdr_msun_ack</i> packets header. */
int hdr_msun_data::offset_ = 0;               /**< Offset used to access in <i>hdr_msun_data</i> packets header. */
int hdr_msun_broadcastdata::offset_ = 0;      /**< Offset used to access in <i>hdr_msun_broadcastdata</i> packets header. */
int hdr_msun_path_est::offset_ = 0;           /**< Offset used to access in <i>hdr_msun_path_est</i> packets header. */

/**
 * Adds the header for <i>hdr_msun_ack</i> packets in ns2.
 */
static class MSunAckPktClass : public PacketHeaderClass {
    public:
    MSunAckPktClass() : PacketHeaderClass("PacketHeader/MSUN_ACK", sizeof(hdr_msun_ack)) {
        this->bind();
        bind_offset(&hdr_msun_ack::offset_);
    }
} class_msun_ack_pkt;

/**
 * Adds the header for <i>hdr_msun_data</i> packets in ns2.
 */
static class MSunDataPktClass : public PacketHeaderClass {
    public:
    MSunDataPktClass() : PacketHeaderClass("PacketHeader/MSUN_DATA", sizeof(hdr_msun_data)) {
        this->bind();
        bind_offset(&hdr_msun_data::offset_);
    }
} class_msun_data_pkt;

/**
 * Adds the header for <i>hdr_msun_broadcastdata</i> packets in ns2.
 */
static class MSunBroadcastDataPktClass : public PacketHeaderClass {
    public:
    MSunBroadcastDataPktClass() : PacketHeaderClass("PacketHeader/MSUN_BROADCASTDATA", sizeof(hdr_msun_broadcastdata)) {
        this->bind();
        bind_offset(&hdr_msun_broadcastdata::offset_);
    }
} class_msun_broadcastdata_pkt;

/**
 * Adds the header for <i>hdr_msun_path_est</i> packets in ns2.
 */
static class MSunPestPktClass : public PacketHeaderClass {
    public:
    MSunPestPktClass() : PacketHeaderClass("PacketHeader/MSUN_PEST", sizeof(hdr_msun_path_est)) {
        this->bind();
        bind_offset(&hdr_msun_path_est::offset_);
    }
} class_msun_pest_pkt;
