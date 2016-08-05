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

#include "uwcbr-timers.h"
#include "uwcbr-module.h"
#include <stdexcept>

using namespace std;

UwSendTimer::UwSendTimer(UwCbrModule *m) : module(m) {
}

void UwSendTimer::expire(Event *e) {
    module->transmit();
    resched(module->getTimeBeforeNextPkt());
}

UwRetxTimer::UwRetxTimer(UwCbrModule *m) : module(m) {
}

void UwRetxTimer::expire(Event *e) {
    if (!module->stopped()) {
        module->retransmit_first(true);
        resched(module->getRetxTimeout());
    }
}

timeout_estimator::timeout_estimator(double k_,
                                     double alpha_,
                                     double beta_,
                                     double def_to) : k(k_),
                                                      alpha(alpha_),
                                                      beta(beta_),
                                                      default_timeout(def_to),
                                                      valid_(false) {
}

void timeout_estimator::update(double rtt_sample) {
    if (!valid_) {
        srtt_ = rtt_sample;
        rttvar_ = rtt_sample / 2;
        valid_ = true;
    }
    else {
        rttvar_ = (1-beta) * rttvar_ + beta * abs(srtt_ - rtt_sample);
        srtt_ = (1-alpha) * srtt_ + alpha * rtt_sample;
    }
}

double timeout_estimator::timeout() const {
    if (valid_)
        return srtt_ + k * rttvar_;
    else
        return default_timeout;
}

double timeout_estimator::srtt() const {
    if (!valid_) throw logic_error("The SRTT is not valid");
    return srtt_;
}

double timeout_estimator::rttvar() const {
    if (!valid_) throw logic_error("The RTTVAR is not valid");
    return rttvar_;
}

bool timeout_estimator::valid() const {
    return valid_;
}

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
