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

#include "uwcbr-stats.h"
#include "uwcbr-module.h"
#include <cmath>
#include <stdexcept>

using namespace std;

avg_stddev_stat::avg_stddev_stat(){
    reset();
}

void avg_stddev_stat::update(const double &val) {
    sum += val;
    sum2 += val*val;
    samples_++;
    last = val;
}

double avg_stddev_stat::avg() const {
    if (samples_ == 0) throw runtime_error("Avg of zero samples");
    else return sum / samples_;
}

double avg_stddev_stat::last_sample() const {
    if (samples_ == 0) throw runtime_error("No last sample");
    else return last;
}

int avg_stddev_stat::samples() const {
    return samples_;
}

double avg_stddev_stat::stddev() const {
    if (samples_ == 0) throw runtime_error("Stddev of zero samples");
    else if (samples_ == 1) return 0;
    else {
        double var = (sum2 - (sum*sum / samples_)) / (samples_-1);
        if (var < 0) throw runtime_error("Negative variance");
        return sqrt(var);
    }
}

void avg_stddev_stat::reset() {
    sum = 0;
    sum2 = 0;
    samples_ = 0;
    last = 0;
}


uwcbr_stats::uwcbr_stats() : pkts_last_reset(0), acks_last_reset(0), lrtime(0) {
    reset_no_last();
}

void uwcbr_stats::reset() {
    pkts_last_reset += pkts_recv + pkts_invalid +
        pkts_dup + pkts_lost + pkts_ooseq;
    acks_last_reset += acks_recv + acks_old + acks_dup;
    reset_no_last();
}

void uwcbr_stats::update_delay(const Packet *const &p) {
    hdr_uwcbr *uwcbrh = HDR_UWCBR(p);
    double d = Scheduler::instance().clock() - uwcbrh->gen_timestamp();
    delay.update(d);
}

void uwcbr_stats::update_ftt_rtt(const Packet *const &p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *uwcbrh = HDR_UWCBR(p);

    double rftt = Scheduler::instance().clock() - ch->timestamp();
    ftt.update(rftt);

    if (uwcbrh->rftt_valid())
        rtt.update(rftt + uwcbrh->rftt());
}

void uwcbr_stats::update_throughput(const Packet *const &p) {
    hdr_cmn *ch = HDR_CMN(p);
    int bytes = ch->size() - UwCbrModule::getCbrHeaderSize();
    // cerr << "Count "<<bytes<<" bytes, CBR hdr "<<UwCbrModule::getCbrHeaderSize()<<endl;
    double dt = Scheduler::instance().clock() - lrtime;
    // cerr << "Last received at " << lrtime << ", dt = " << dt << endl;
    lrtime = Scheduler::instance().clock();
    sumbytes += bytes;
    sumdt += dt;
}

double uwcbr_stats::throughput() const {
    return (sumdt > 0) ? (8 * sumbytes / sumdt) : 0;
}

void uwcbr_stats::reset_no_last() {
    acks_dup = 0;
    acks_dup_sent = 0;
    acks_old = 0;
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

    sumbytes = 0;
    sumdt = 0;

    delay.reset();
    ftt.reset();
    rtt.reset();
}

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
