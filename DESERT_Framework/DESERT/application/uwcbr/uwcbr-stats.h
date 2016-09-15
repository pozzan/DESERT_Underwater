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

    void reset();
    void update(const double &val);

    double avg() const;
    double last_sample() const;
    int samples() const;
    double stddev() const;
    double variance() const;

private:
    double sum;
    double sum2;
    int samples_;
    double last;
};

/**
 * Counts the statistics for the sent/received packets
 *
 * The packets are classified in this way:
 */
class uwcbr_stats {
public:
    int pkts_last_reset; /// Total number of data packets seen before the last reset
    int acks_last_reset; /// Total number of ACKs seen before the last reset

    int acks_dup; /// Number of duplicate ACKs received
    int acks_dup_sent; /// Number of duplicate ACKs sent
    int acks_old; /// Number of ACKs received for old packets (< next_ack)
    int acks_recv; /// Number of regular ACKs received
    int acks_sent; /// Number of regular ACKs sent

    int pkts_dup; /// Number of duplicate packets received
    int pkts_invalid; /// Number of packets received with the wrong ptype
    int pkts_lost; /// Number of packets lost, if ARQ is enabled must be zero
    int pkts_ooseq; /// Number of packets dropped because they were outside the rx_window
    int pkts_proc; /// Number of packets processed in order */
    int pkts_recv; /// Number of packets received and put in the recv_queue
    int pkts_retx_dupack; /// Number of packets retransmitted because of a DUPACK
    int pkts_retx_timeout; /// Number of packets retransmitted because of a timeout

    avg_stddev_stat delay;      /**< Avg. and std.dev. of the delay between the packet generation and processing */
    avg_stddev_stat ftt;        /**< Avg. and std.dev. of the delay between the sending of the packet and its reception */
    avg_stddev_stat rtt;        /**< Avg. and std.dev. of the round trip time, without the queueing delay */

    uwcbr_stats();

    void reset();
    void update_delay(const Packet *const &p);
    void update_ftt_rtt(const Packet *const &p);
    void update_throughput(const Packet *const &p);

    double throughput() const;
private:
    double lrtime;              /**< Time of last packet reception. */
    double sumbytes;            /**< Sum of bytes received. */
    double sumdt;               /**< Sum of the delays. */

    void reset_no_last();
};

#endif

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
