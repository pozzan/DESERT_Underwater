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
//MPhy_Bpsk()
{
	if (!MPhy_Bpsk::initialized)
	{
		MPhy_Bpsk::modid = MPhy::registerModulationType(OPTICAL_MODULATION_TYPE);
		MPhy_Bpsk::initialized = true;
	}
	MPhy_Bpsk();
	bind("Prx_threshold_",&Prx_threshold)
}

int UwOpticalPhy::command(int argc, const char*const* argv) {
    //Tcl& tcl = Tcl::instance();
    return MPhy_Bpsk::command(argc, argv);     
}


virtual double getTxDuration(Packet* p)
{
	//TODO: is it Bpsk method workin also for this case?
}


virtual void startRx(Packet* p)
{
	hdr_MPhy* ph = HDR_MPHY(p);
  	double rx_time = ph->rxtime;
  	double tx_time = ph->txtime;
  	if ( (PktRx == 0) && (txPending == false) )
    {
    	double snr_dB = 10*log10(ph->Pr / ph->Pn);
    	if (snr_dB > MPhy_Bpsk::getAcquisitionThreshold())
    	{
    		if (ph->modulationType == MPhy_Bpsk::modid)
    		{
    			PktRx = p;
    			Phy2MacStartRx(p);
    			return;
    		}
    		else
    		{
    			if (debug_) cout << "UwOpticalPhy::Drop Packet::Wrong modulation" << endl;
    		}
    	} 
    	else 
    	{
    		if (debug_) cout << "UwOpticalPhy::Drop Packet::Below Threshold" << endl;
    	}
    }
    else
    {
    	if (debug_) cout << "UwOpticalPhy::Drop Packet::Synced onto another packet" << endl;
    }
}
    
virtual void endRx(Packet* p)
{ 
  	hdr_cmn* ch = HDR_CMN(p);
  	hdr_MPhy* ph = HDR_MPHY(p);
  	if (PktRx != 0)
    {
    	if (PktRx == p)
		{ 
			ch->error() = 0; 
	  		sendUp(p);
	  		PktRx = 0;
	  	} 
	  	else 
	  	{
			MPhy_Bpsk::dropPacket(p);
		}
	} 
	else 
	{
		MPhy_Bpsk::dropPacket(p);
	}
}
