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
    UwRetxTimer(UwCbrModule *m);
protected:
    virtual void expire(Event *e);
    UwCbrModule *module;
};

/**
 * Estimates the RTT from the samples with the same algorithm used by
 * TCP
 */
class timeout_estimator {
public:
    const double k;
    const double alpha;
    const double beta;
    const double default_timeout;

    timeout_estimator(double k_, double alpha_, double beta_, double def_to);

    void update(double rtt_sample);
    double timeout() const;

    double srtt() const;
    double rttvar() const;
    bool valid() const;
private:
    double srtt_;
    double rttvar_;
    bool valid_;
};

#endif

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
