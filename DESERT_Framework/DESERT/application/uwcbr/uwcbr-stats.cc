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
#include <cmath>

using namespace std;


avg_stddev_stat::avg_stddev_stat(){
    reset();
}

void avg_stddev_stat::update(const double &val) {
    sum += val;
    sum2 += val*val;
    samples++;
}

double avg_stddev_stat::avg() const {
    if (samples > 0) return sum / samples;
    else return 0;
}

double avg_stddev_stat::stddev() const {
    if (samples > 1) {
        double var = (sum2 - (sum*sum / samples)) / (samples-1);
        return var > 0 ? sqrt(var) : 0;
    }
    else return 0;
}

void avg_stddev_stat::reset() {
    sum = 0;
    sum2 = 0;
    samples = 0;
}

void uwcbr_stats::update_delay(const Packet *const &p) {
    hdr_uwcbr *uwcbrh = HDR_UWCBR(p);
    double d = Scheduler::instance().clock() - uwcbrh->gen_timestamp();
    delay.update(d);
}

void uwcbr_stats::update_ftt_rtt(const Packet *const &p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *uwcbrh = HDR_UWCBR(p);

    rftt = Scheduler::instance().clock() - ch->timestamp();
    ftt.update(rftt);

    if (uwcbrh->rftt_valid())
        rtt.update(rftt + uwcbrh->rftt());
}


// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
