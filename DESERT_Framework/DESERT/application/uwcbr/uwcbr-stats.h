// Copyright (c) 2015 Regents of the SIGNET lab, University of Padova.
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
//

#ifndef UWCBR_STATS_H
#define UWCBR_STATS_H

#include "uwcbr-packet.h"

class avg_stddev_stat {
public:
    avg_stddev_stat();

    void update(const double &val);
    double avg() const;
    double stddev() const;
    void reset();
private:
    double sum;
    double sum2;
    int samples;
};

/**
 * Counts the statistics for the sent/received packets
 *
 * The packets are classified in this way: 
 */
class uwcbr_stats {
public:
    int pkts_last_reset;        /**< Total number of packets seen before the last reset */
    int acks_last_reset;        /**< Total number of ACKs seen before the last reset */

    int acks_dup;                /**< Total number of duplicate ACKs received */
    int acks_dup_sent;
    int acks_invalid;            /**< Total number of invalid ACKs received */
    int acks_recv;               /**< Total number of regular ACKs received */
    int acks_sent;

    int pkts_dup;               /**< Total number of duplicate packets received, included in pkts_invalid */
    int pkts_invalid;           /**< Total number of invalid packets received. */
    int pkts_lost;              /**< Total number of lost packets. Used only when drop_out_of_sequence = true and use_arq = false */
    int pkts_ooseq;             /**< Total number of packets received out of sequence. Used only when drop_out_of_sequence = true and use_arq = false */
    int pkts_proc;              /**< Total number of packets processed in order */
    int pkts_recv;              /**< Total number of packets received, before the reordering queue */
    int pkts_retx_dupack;       /**< Total number of packets retransmitted because of a DUPACK */
    int pkts_retx_timeout;      /**< Total number of packets retransmitted because of a timeout */

    //double srtt;                /**< Smoothed Round Trip Time, calculated as for TCP. */
    //double sftt;                /**< Smoothed Forward Trip Time, calculated as srtt. */
    //double sthr;                /**< Smoothed throughput calculation. */

    double lrtime;              /**< Time of last packet reception. */
    double sumbytes;            /**< Sum of bytes received. */
    double sumdt;               /**< Sum of the delays. */

    double rftt;                /**< Forward Trip Time seen for last received packet. */
    avg_stddev_stat delay;      /**< Avg. and std.dev. of the delay between the packet generation and processing */
    avg_stddev_stat ftt;        /**< Avg. and std.dev. of the delay between the sending of the packet and its reception */
    avg_stddev_stat rtt;        /**< Avg. and std.dev. of the round trip time, without the queueing delay */

    uwcbr_stats() : pkts_last_reset(0), acks_last_reset(0), lrtime(0) {
        reset_no_last();
    }

    inline void reset() {
        pkts_last_reset += pkts_recv + pkts_invalid + pkts_ooseq;
        acks_last_reset += acks_recv + acks_invalid + acks_dup;
        reset_no_last();
    }

    void update_delay(const Packet *const &p);
    void update_ftt_rtt(const Packet *const &p);
private:
    inline void reset_no_last() {
        acks_dup = 0;
        acks_dup_sent = 0;
        acks_invalid = 0;
        acks_recv = 0;
        acks_sent = 0;

        pkts_dup = 0;
        pkts_invalid = 0;
        pkts_lost = 0;
        pkts_ooseq = 0;
        pkts_proc = 0;
        pkts_recv = 0;
        pkts_retx_dupack = 0;
        pkts_retx_timeout = 0;

        //srtt = 0;
        //sftt = 0;
        //sthr = 0;

        sumbytes = 0;
        sumdt = 0;

        rftt = -1;
        delay.reset();
        ftt.reset();
        rtt.reset();
    }
};


#endif

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
