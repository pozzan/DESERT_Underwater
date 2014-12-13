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
 * @file   uwoptical.cc
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Implementation of UwOptical class.
 *
 */

#include "uwoptical-phy.h"

static class UwOpticalPhyClass : public TclClass {
public:
    UwOpticalPhyClass() : TclClass("Module/UW/OPTICAL/PHY") {}
    TclObject* create(int, const char*const*) {
        return (new UwOpticalPhy);
    }
} class_module_optical;

UwOpticalPhy::UwOpticalPhy() 
: 
MPhy_Bpsk()
{
	bind("Prx_threshold_",&Prx_threshold)
}

int UwOpticalPhy::command(int argc, const char*const* argv) {
    //Tcl& tcl = Tcl::instance();
    return MPhy_Bpsk::command(argc, argv);     
} /* UwOptical::command */

virtual int getModulationType(Packet* p)
{
	//TODO
}

virtual double getTxDuration(Packet* p)
{
	//TODO
}


virtual void startRx(Packet* p)
{
	static int mac_addr = -1;
	hdr_MPhy* ph = HDR_MPHY(p);
  	double rx_time = ph->rxtime;
  	double tx_time = ph->txtime;
  	if ( (PktRx == 0) && (txPending == false) )
    {
    	double snr_dB = 10*log10(ph->Pr / ph->Pn); //calculate SNR for future statistics
    	if (ph->Pr >= Prx_threshold)
    	{
    		if (ph->modulationType == modid) //TODO: check if useful and how it works
    		{
    			PktRx = p;
    			Phy2MacStartRx(p);
    			return;
    		}
    		else
    		{
    			//TODO: not allowed modulation type.
    		}
    	} else {
    		//TODO: Pr below threshold
    	}
    }
    else
    {
    	//TODO: we are sync onto another packet
    }
}
    
virtual void endRx(Packet* p)
{
	static int mac_addr = -1;
  
  	hdr_cmn* ch = HDR_CMN(p);
  	hdr_MPhy* ph = HDR_MPHY(p);
  	if (PktRx != 0)
    {
    	if (PktRx == p)
		{  
	  		sendUp(p);
	  		PktRx = 0; // We can now sync onto another packet
	  	}
	} else {
		MPhy_Bpsk::dropPacket(p);
	}
}

//CHECK INTERFERENCE AND PROPAGATION AND HOW IT WORKS WITH PHY AND INTEGRATE THEM.
