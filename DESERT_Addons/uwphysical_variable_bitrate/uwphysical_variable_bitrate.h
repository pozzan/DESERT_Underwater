//
// Copyright (c) 2013 Regents of the SIGNET lab, University of Padova.
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


#ifndef UWPHYSICAL_VAR_BITRATE_H
#define UWPHYSICAL_VAR_BITRATE_H

#include "uwphysical.h"
#include "uwinterference.h"

#include "mac.h"

#include <phymac-clmsg.h>
#include <string.h>

#include <rng.h>
#include <packet.h>
#include <module.h>
#include <tclcl.h>

#include <iostream>
#include <string.h>
#include <cmath>
#include <limits>
#include <climits>

class UnderwaterPhysicalVarBitRate : public UnderwaterPhysical {
public:
	UnderwaterPhysicalVarBitRate();

	virtual ~UnderwaterPhysicalVarBitRate() {}

	virtual int command(int, const char*const*);

	virtual double getTxDuration(Packet* p);

	inline int getAUV_mac_addr() { return AUV_mac_addr;}

	inline int getBurstBitrate() {return BurstBitRate_;}

	inline int getIMBitrate() {return IMBitRate_;}

protected:

	virtual void endRx(Packet* p);

	int AUV_mac_addr;

	int BurstBitRate_;

	int IMBitRate_;
        
        bool dualBitRateMode;
        
        int phy_var_bitrate_debug;

};

#endif
