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

const double K = 1.38*1.E-23; //Boltzmann constant
const double q = 1.6*1.E-19; //electronic charge

static class UwOpticalPhyClass : public TclClass {
public:
    UwOpticalPhyClass() : TclClass("Module/UW/OPTICAL/PHY") {}
    TclObject* create(int, const char*const*) {
        return (new UwOpticalPhy);
    }
} class_module_optical;

UwOpticalPhy::UwOpticalPhy() 
{
    if (!MPhy_Bpsk::initialized)
    {
        MPhy_Bpsk::modid = MPhy::registerModulationType(OPTICAL_MODULATION_TYPE);
        MPhy_Bpsk::initialized = true;
    }
    MPhy_Bpsk();
    bind("Id_",&Id);
    bind("Il_",&Il);
    bind("R_",&R);
    bind("S_",&S);
    bind("T_",&T);
    bind("Ar_",&Ar_);
}

int UwOpticalPhy::command(int argc, const char*const* argv) {
    //Tcl& tcl = Tcl::instance();
    return MPhy_Bpsk::command(argc, argv);     
}

void UwOpticalPhy::startRx(Packet* p)
{
    hdr_MPhy* ph = HDR_MPHY(p);
    if ( (PktRx == 0) && (txPending == false) )
    {
        double snr_dB = getSNRdB(p);
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
    		if (debug_) cout << "UwOpticalPhy::Drop Packet::Below Threshold : snrdb = " << snr_dB << ", threshold = " << MPhy_Bpsk::getAcquisitionThreshold() << endl;
    	}
    }
    else
    {
    	if (debug_) cout << "UwOpticalPhy::Drop Packet::Synced onto another packet PktRx = " << PktRx << ", pending = " << txPending << endl;
    }
}

double UwOpticalPhy::getSNRdB(Packet* p)
{
    hdr_MPhy* ph = HDR_MPHY(p);
    double snr_linear = pow((S*ph->Pr),2)/((2*q*(Id+Il)*ph->srcSpectralMask->getBandwidth() + ((4*K*T*ph->srcSpectralMask->getBandwidth())/R)) + ph->Pn);
    return 10*log10(snr_linear);
}

double UwOpticalPhy::getNoisePower(Packet* p)
{
    // TODO: search the corect value in the LUT
    double lut_value = 0;
    return pow(lut_value * Ar_ * S , 2);//right now returns 0, due to not bias the snr calculation with unexpected values
}
    
void UwOpticalPhy::endRx(Packet* p)
{ 
    hdr_cmn* ch = HDR_CMN(p);
    hdr_MPhy* ph = HDR_MPHY(p);
    if (MPhy_Bpsk::PktRx != 0)
    {
    	if (MPhy_Bpsk::PktRx == p)
	    {
            if(interference_)
            {
                /* Old code:
                const PowerChunkList& power_chunk_list = interference_->getInterferencePowerChunkList(p);
                if(power_chunk_list.empty())
                {
                    //no interference
                    ch->error() = 0; 
                    sendUp(p);
                    PktRx = 0;
                }
                */
                // new code: 
                double interference_power = interference_->getInterferencePower(p);
                if(interference_power == 0)
                {
                    //no interference
                    ch->error() = 0; 
                }
                else
                {
                    //at least one interferent packet
                    ch->error() = 1;
                    if (debug_) cout << "UwOpticalPhy::endRx interference power = " << interference_power << endl;
                }
            }
            else
            {
                //no interference model set
                ch->error() = 0; 
            }

            sendUp(p);
            PktRx = 0;
    	} 
    	else 
    	{
            dropPacket(p);
    	}
    }
    else 
    {
        dropPacket(p);
    }
}
