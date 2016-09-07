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
}

UwRetxTimer::UwRetxTimer(UwCbrModule *m, sn_t sn) : module(m),
						    k(4),
						    alpha(1/8),
						    beta(1/4),
						    srtt_valid_(false),
						    packet_sn(sn) {
}

void UwRetxTimer::expire(Event *e) {
  module->retransmit_first();
}

double UwRetxTimer::srtt() {
  if (!srtt_valid_)
    throw logic_error("The SRTT is not valid");
  else
    return srtt_;
}

double UwRetxTimer::rttvar() {
  if (!srtt_valid_)
    throw logic_error("The RTTVAR is not valid");
  else
    return rttvar_;
}

bool UwRetxTimer::srtt_valid() {
  return srtt_valid_;
}

void UwRetxTimer::update_srtt(double rtt_sample) {
  throw runtime_error("Not implemented");
}

void UwRetxTimer::reschedule_srtt() {
  throw runtime_error("Not implemented");
}
