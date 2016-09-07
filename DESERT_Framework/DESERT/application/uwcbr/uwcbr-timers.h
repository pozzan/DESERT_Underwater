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

#ifndef UWCBR_TIMERS_H
#define UWCBR_TIMERS_H

#include <timer-handler.h>
#include "uwcbr-packet.h"

class UwCbrModule;

/**
 * UwSendTimer is used to handle the scheduling period of <i>UWCBR</i> packets.
 */
class UwSendTimer : public TimerHandler {
public:
    UwSendTimer(UwCbrModule *m);
protected:
    virtual void expire(Event *e);
    UwCbrModule *module;
};

/**
 * UwRetxTimer is used to schedule the retransmission of packets after
 * a timeout following the algorithm used by TCP
 */
class UwRetxTimer : public TimerHandler {
public:
    UwRetxTimer(UwCbrModule *m, sn_t sn);
    void update_srtt(double rtt_sample);
    void reschedule_srtt();
    double srtt();
    double rttvar();
    bool srtt_valid();
    double k;
    double alpha;
    double beta;
protected:
    virtual void expire(Event *e);
    UwCbrModule *module;
    sn_t packet_sn;
    double srtt_;
    double rttvar_;
    bool srtt_valid_;
};

#endif

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
