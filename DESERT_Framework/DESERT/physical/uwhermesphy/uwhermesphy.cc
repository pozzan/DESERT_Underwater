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
 * @file   uwhermesphy.cc
 * @author Filippo Campagnaro
 * @version 1.1.0
 *
 * \brief Implementation of UwHermesPhy class
 *
 */

#include "uwphysical.h"

static class UwHermesPhyClass : public TclClass {
public:
    UwHermesPhyClass() : TclClass("Module/UW/HERMES/PHY") {}
    TclObject* create(int, const char*const*) {
        return (new UwHermesPhy);
    }
} class_module_uwhermesphy;

UwHermesPhy::UwHermesPhy() :
modulation_name_("BPSK"),
time_ready_to_end_rx_(0),
Tx_Time_(0),
Rx_Time_(0),
Energy_Tx_(0),
Energy_Rx_(0),
tx_power_(3.3),
rx_power_(0.620),
Interference_Model("CHUNK")
{
    bind("rx_power_consumption_", &rx_power_);
    bind("tx_power_consumption_", &tx_power_);
}

void UwHermesPhy::endRx(Packet* p) {
    hdr_cmn* ch = HDR_CMN(p);
    hdr_MPhy* ph = HDR_MPHY(p);
    hdr_mac* mach = HDR_MAC(p);
    counter interferent_pkts;

    static int mac_addr = -1;

    ClMsgPhy2MacAddr msg;
    sendSyncClMsg(&msg);
    mac_addr = msg.getAddr();

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
                            if (itInterf->first > (ph->Pn / 2)) {
                                int nbits2 = itInterf->second * BitRate_;
                                per_ni = getPER(ph->Pr / (ph->Pn + itInterf->first), nbits2, p);
                                x = RNG::defaultrng()->uniform_double();
                                error_ni = x <= per_ni;
                                if (error_ni) {
                                    break;
                                }
                            }
                        }
                    } else if (Interference_Model == "MEANPOWER") {
                        double interference = interference_->getInterferencePower(p);
                        per_ni = getPER(ph->Pr / (ph->Pn + interference), nbits, p);
                    } else {
                        std::cerr << "Please choose the right interference model to use: CHUNK or MEANPOWER" << std::endl;
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
            if (debug_) {
                if (error_ni == 1) {
                    std::cout << NOW << "  UnderwaterPhysical(" << mac_addr << ")::endRx() packet " << ch->uid() << " contains errors due to noise and interference." << std::endl;
                } else if (error_n == 1) {
                    std::cout << NOW << "  UnderwaterPhysical(" << mac_addr << ")::endRx() packet " << ch->uid() << " contains errors due to noise." << std::endl;
                }
            }

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
} /* UnderwaterPhysical::endRx */

/*double UwHermesPhy::getPER(double _snr, int _nbits, Packet* _p) {
    double snr_with_penalty = _snr * pow(10, RxSnrPenalty_dB_ / 10.0);

    double ber_ = 0;
    if (modulation_name_ == "BPSK") {
        ber_ = 0.5 * erfc(sqrt(snr_with_penalty));
    } else if (modulation_name_ == "BFSK") {
        ber_ = 0.5 * exp(-snr_with_penalty / 2);
    } else if (modulation_name_ == "8PSK") {
        double const M = 8;
        ber_ = (1 / this->log2(M)) * get_prob_error_symbol_mpsk(snr_with_penalty, M);
    } else if (modulation_name_ == "16PSK") {
        double const M = 16;
        ber_ = (1 / this->log2(M)) * get_prob_error_symbol_mpsk(snr_with_penalty, M);
    } else if (modulation_name_ == "32PSK") {
        double const M = 32;
        ber_ = (1 / this->log2(M)) * get_prob_error_symbol_mpsk(snr_with_penalty, M);
    }

    // PER calculation
    return 1 - pow(1 - ber_, _nbits);
} */
double UwHermesPhy::getPER(double _snr, int _nbits, Packet* _p) {
    double distance = getDistance(_p);
    int size = getSize(_p);
    return matchPER(distance,size);
}/* UnderwaterPhysical::getPER */

double UwHermesPhy::getDistance(Packet* _p){
    hdr_MPhy* ph = HDR_MPHY(_p);
    double x_src = (ph->srcPosition)->getX();
    double y_src = (ph->srcPosition)->getY();
    double z_src = (ph->srcPosition)->getZ();
    double x_dst = (ph->dstPosition)->getX();
    double y_dst = (ph->dstPosition)->getY();
    double z_dst = (ph->dstPosition)->getZ();
    return pow(pow(x_src-x_dst,2.0)+pow(y_src-y_dst,2.0)+pow(z_src-z_dst,2.0),0.5);
} /* */

int UwHermesPhy::getSize(Packet* _p){
    hdr_cmn* ch = HDR_CMN(p);
    return ch->size()*8;
}

double matchPER(double distance, int size){
    return 0.0;
}