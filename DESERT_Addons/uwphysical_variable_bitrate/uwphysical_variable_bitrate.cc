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

#include "uwphysical_variable_bitrate.h"
#include "uwpolling_cmn_hdr.h"

static class UnderwaterPhysicalVarBitRateClass : public TclClass {
public:
  UnderwaterPhysicalVarBitRateClass() : TclClass("Module/UW/PHYSICAL/VARIABLE_BIT_RATE") {}
  TclObject* create(int, const char*const*) {
    return (new UnderwaterPhysicalVarBitRate);
  }
} class_module_uwphysicalvarbitrate;

UnderwaterPhysicalVarBitRate::UnderwaterPhysicalVarBitRate() :
AUV_mac_addr(0),
BurstBitRate_(0),
IMBitRate_(0),
dualBitRateMode(true),
phy_var_bitrate_debug(0)
{
    bind("AUV_mac_addr_", &AUV_mac_addr);
    bind("BurstBitRate_", &BurstBitRate_);
    bind("IMBitRate_", &IMBitRate_);
    bind("phy_var_bitrate_debug_", &phy_var_bitrate_debug);
}

int UnderwaterPhysicalVarBitRate::command(int argc, const char*const* argv)
{
    if (argc == 2)
    {
        if(strcasecmp(argv[1],"DisableDualBitRateMode") == 0) {
            dualBitRateMode = false;
            return TCL_OK;
        }
    }
    
    return UnderwaterPhysical::command(argc,argv);
}

double UnderwaterPhysicalVarBitRate::getTxDuration(Packet* p)
{
    hdr_cmn* ch = HDR_CMN(p);
    hdr_MPhy* ph = HDR_MPHY(p);
    hdr_mac* mach = HDR_MAC(p);
    
    assert(ph->srcSpectralMask);
    double txduration;
    
    if(dualBitRateMode) 
    {
        if ( (mach->macDA() == AUV_mac_addr) || (mach->macSA() == AUV_mac_addr) ) 
        {
            txduration = (ch->size() * 8.0 / BurstBitRate_);
            if(phy_var_bitrate_debug) cout << "Packet from/to AUV (mac address = " << AUV_mac_addr << ") -> Using BitRate = " << BurstBitRate_ << std::endl;
        } 
        else 
        {
            txduration = (ch->size() * 8.0 / IMBitRate_);
            if(phy_var_bitrate_debug) cout << "Packet from node " << mach->macDA() << " to node " << mach->macSA() << "  -> Using BitRate = " << IMBitRate_ << std::endl;
        }
    } 
    else 
    {
        if ( BitRate_ <= 0 ) BitRate_ = ph->srcSpectralMask->getBandwidth() / 2.0 ;
        txduration = (ch->size() * 8.0 / BitRate_);
    }
    
    assert(txduration > 0);
    
    if (debug_)
    std::cout << showpoint << NOW << " " <<  __PRETTY_FUNCTION__ 
	 << " packet size: " << ch->size() 
	 << " tx duration: " << txduration 
	 << std::endl;
    
    return (txduration);
}

void UnderwaterPhysicalVarBitRate::endRx(Packet* p) {
    hdr_cmn* ch = HDR_CMN(p);
    hdr_MPhy* ph = HDR_MPHY(p);
    hdr_mac* mach = HDR_MAC(p);
    counter interferent_pkts;
    
    if (PktRx != 0) {
        if (PktRx == p) {
            double per_ni; // packet error rate due to noise and/or interference
            double per_n; // packet error rate due to noise only 

            int nbits = ch->size()*8;
            double x = RNG::defaultrng()->uniform_double();
            per_n = getPER(ph->Pr / ph->Pn, nbits, p);
            bool error_n = x <= per_n;
            bool error_ni = 0;
            if (!error_n) {
                if (interference_) {
                    if (Interference_Model == "CHUNK") {
                        const PowerChunkList& power_chunk_list = interference_->getInterferencePowerChunkList(p);

                        for (PowerChunkList::const_iterator itInterf = power_chunk_list.begin(); itInterf != power_chunk_list.end(); itInterf++) {
//                            int nbits2 = itInterf->second * BitRate_;
                            int nbits2;
                            nbits2 = 0;
                            if (dualBitRateMode)
                            {
                                if( (mach->macDA() == AUV_mac_addr) || (mach->macSA() == AUV_mac_addr) )
                                {
                                    nbits2 = itInterf->second * BurstBitRate_;
                                }
                                else
                                {
                                    nbits2 = itInterf->second * IMBitRate_;
                                }
                            }
                            else
                            {
                                nbits2 = itInterf->second * BitRate_;
                            }
                            //assert(nbits2 > 0);
                            per_ni = getPER(ph->Pr / (ph->Pn + itInterf->first), nbits2, p);
                            x = RNG::defaultrng()->uniform_double();
                            error_ni = x <= per_ni;
                            if (error_ni) {
                                break;
                            }
                        }
                    } else if (Interference_Model == "MEANPOWER") {
                        double interference = interference_->getInterferencePower(p);
                        per_ni = getPER(ph->Pr / (ph->Pn + interference), nbits, p);
                    } else {
                        cerr << "Please choose the right interference model to use: CHUNK or MEANPOWER" << endl;
                        exit(1);
                    }
                    interferent_pkts = interference_->getCounters(p);

                } else {
                    per_ni = getPER(ph->Pr / (ph->Pn + ph->Pi), nbits, p);
                    error_ni = x <= per_ni;
                }
            }
            
            if (time_ready_to_end_rx_ > Scheduler::instance().clock()) {
		Rx_Time_ = Rx_Time_ + ph->duration - time_ready_to_end_rx_ + Scheduler::instance().clock();
            } else {
                Rx_Time_ += ph->duration;
            }
            time_ready_to_end_rx_ = Scheduler::instance().clock() + ph->duration;
            Energy_Rx_ += consumedEnergyRx(ph->duration);

            ch->error() = error_ni || error_n;
           
            if (error_n) {
                incrErrorPktsNoise();
                if (mach->ftype() != MF_CONTROL) {
                    incrTot_pkts_lost();
                }
                else if (mach->ftype() == MF_CONTROL) {
                    incrTotCrtl_pkts_lost();
                }
            } else if (error_ni) {
                if (mach->ftype() != MF_CONTROL) {
                    incrErrorPktsInterf();
                    incrTot_pkts_lost();
                    if (interferent_pkts.second >= 1) {
                        incrCollisionDATA();
                    } else {
                        if (interferent_pkts.first > 0) {
                            incrCollisionDATAvsCTRL();
                        } else {
//                            std::cerr << "Logical error on counting the packet interference!!!" << std::endl;
                            ;
                        }
                    }
                }
                else if (mach->ftype() == MF_CONTROL) {
                    incrTotCrtl_pkts_lost();
                    incrErrorCtrlPktsInterf();
                    if (interferent_pkts.first > 0) {
                        incrCollisionCTRL();
                    }
                }

            }
            sendUp(p);
            PktRx = 0;
        } else {
            dropPacket(p);
        }
    } else {
        dropPacket(p);
    }
}  
