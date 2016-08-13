//
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

/**
 * @file   uwcbr-module.cc
 * @author Giovanni Toso
 * @version 1.1.0
 * 
 * \brief Provides the <i>UWCBR</i> class implementation.
 * 
 * Provides the <i>UWCBR</i> class implementation.
 */

#include "uwcbr-module.h"

#include <iostream>
#include <rng.h>
#include <stdint.h>

extern packet_t PT_UWCBR;

int hdr_uwcbr::offset_;         /**< Offset used to access in <i>hdr_uwcbr</i> packets header. */

/**
 * Adds the header for <i>hdr_uwcbr</i> packets in ns2.
 */
static class UwCbrPktClass : public PacketHeaderClass {
public:

    UwCbrPktClass() : PacketHeaderClass("PacketHeader/UWCBR", sizeof (hdr_uwcbr)) {
        this->bind();
        bind_offset(&hdr_uwcbr::offset_);
    }
} class_uwcbr_pkt;


/**
 * Adds the module for UwCbrModuleClass in ns2.
 */
static class UwCbrModuleClass : public TclClass {
public:

    UwCbrModuleClass() : TclClass("Module/UW/CBR") {
    }

    TclObject* create(int, const char*const*) {
        return (new UwCbrModule());
    }
} class_module_uwcbr;

void UwSendTimer::expire(Event *e) {
    module->transmit();
}

int UwCbrModule::uidcnt_ = 0;

UwCbrModule::UwCbrModule()
:
dstPort_(0),
dstAddr_(0),
priority_(0),
sn_check(USHRT_MAX, false),
PoissonTraffic_(0),
debug_(0),
drop_out_of_order_(0),
sendTmr_(this),
txsn(1),
hrsn(0),
pkts_recv(0),
pkts_ooseq(0),
pkts_lost(0),
pkts_invalid(0),
pkts_last_reset(0),
rftt(-1),
srtt(0),
sftt(0),
lrtime(0),
sthr(0),
period_(0),
pktSize_(0),
sumrtt(0),
sumrtt2(0),
rttsamples(0),
sumftt(0),
sumftt2(0),
fttsamples(0),
sumbytes(0),
sumdt(0),
esn(0)
{ // binding to TCL variables
    bind("period_", &period_);
    bind("destPort_", (int*) &dstPort_);
    bind("destAddr_", (int*) &dstAddr_);
    bind("packetSize_", &pktSize_);
    bind("PoissonTraffic_", &PoissonTraffic_);
    bind("debug_", &debug_);
    bind("drop_out_of_order_", &drop_out_of_order_);
}

UwCbrModule::~UwCbrModule() {
}

int UwCbrModule::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();
    if (argc == 2) {
        if (strcasecmp(argv[1], "start") == 0) {
            start();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "stop") == 0) {
            stop();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrtt") == 0) {
            tcl.resultf("%f", GetRTT());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getftt") == 0) {
            tcl.resultf("%f", GetFTT());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getper") == 0) {
            tcl.resultf("%f", GetPER());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getthr") == 0) {
            tcl.resultf("%f", GetTHR());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getcbrheadersize") == 0) {
            tcl.resultf("%d", this->getCbrHeaderSize());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrttstd") == 0) {
            tcl.resultf("%f", GetRTTstd());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getfttstd") == 0) {
            tcl.resultf("%f", GetFTTstd());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getsentpkts") == 0) {
            tcl.resultf("%d", txsn - 1);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrecvpkts") == 0) {
            tcl.resultf("%d", pkts_recv);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "setprioritylow") == 0) {
            priority_ = 0;
            return TCL_OK;
        } else if (strcasecmp(argv[1], "setpriorityhigh") == 0) {
            priority_ = 1;
            return TCL_OK;
        } else if (strcasecmp(argv[1], "sendPkt") == 0) {
            this->sendPkt();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "sendPktLowPriority") == 0) {
            this->sendPktLowPriority();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "sendPktHighPriority") == 0) {
            this->sendPktHighPriority();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "resetStats") == 0) {
            resetStats();
            fprintf(stderr, "CbrModule::command() resetStats %s, pkts_last_reset=%d, hrsn=%d, txsn=%d\n", tag_, pkts_last_reset, hrsn, txsn);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "printidspkts") == 0) {
            this->printIdsPkts();
            return TCL_OK;
        }
    }
    return Module::command(argc, argv);
}

void UwCbrModule::initPkt(Packet* p) {
    hdr_cmn* ch = hdr_cmn::access(p);
    ch->uid()   = uidcnt_++;
    ch->ptype() = PT_UWCBR;
    ch->size()  = pktSize_;

    hdr_uwip* uwiph  = hdr_uwip::access(p);
    uwiph->daddr()   = dstAddr_;
    
    hdr_uwudp* uwudp = hdr_uwudp::access(p);
    uwudp->dport()   = dstPort_;

    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    uwcbrh->sn()       = txsn++;
    uwcbrh->priority() = priority_;
    ch->timestamp()    = Scheduler::instance().clock();

    if (rftt >= 0) {
        uwcbrh->rftt() = rftt;
        uwcbrh->rftt_valid() = true;
    } else {
        uwcbrh->rftt_valid() = false;
    }
}

void UwCbrModule::start() {
    sendTmr_.resched(getTimeBeforeNextPkt());
}

void UwCbrModule::sendPkt() {
    double delay      = 0;
    Packet* p         = Packet::alloc();
    this->initPkt(p);
    hdr_cmn* ch       = hdr_cmn::access(p);
    hdr_uwcbr* uwcbrh = HDR_UWCBR(p);
    if (debug_ > 10)
        printf("CbrModule(%d)::sendPkt, send a pkt (%d) with sn: %d\n", getId(), ch->uid(), uwcbrh->sn());
    sendDown(p, delay);
}

void UwCbrModule::sendPktLowPriority() {
    double delay       = 0;
    Packet* p          = Packet::alloc();
    this->initPkt(p);
    hdr_cmn* ch        = hdr_cmn::access(p);
    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    uwcbrh->priority() = 0;
    if (debug_ > 10)
        printf("CbrModule(%d)::sendPkt, send a pkt (%d) with sn: %d\n", getId(), ch->uid(), uwcbrh->sn());
    sendDown(p, delay);
}

void UwCbrModule::sendPktHighPriority() {
    double delay       = 0;
    Packet* p          = Packet::alloc();
    this->initPkt(p);
    hdr_cmn* ch        = hdr_cmn::access(p);
    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    uwcbrh->priority() = 1;
    if (debug_ > 10)
        printf("CbrModule(%d)::sendPkt, send a pkt (%d) with sn: %d\n", getId(), ch->uid(), uwcbrh->sn());
    sendDown(p, delay);
}

void UwCbrModule::transmit() {
   sendPkt();
   sendTmr_.resched(getTimeBeforeNextPkt()); // schedule next transmission
}

void UwCbrModule::stop() {
    sendTmr_.force_cancel();
}

void UwCbrModule::recv(Packet* p) {
    hdr_cmn* ch = hdr_cmn::access(p);
    if (debug_ > 10) 
       printf("CbrModule(%d)::recv(Packet*p,Handler*) pktId %d\n", getId(), ch->uid());

    if (ch->ptype() != PT_UWCBR) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        incrPktInvalid();
        return;
    }

    hdr_uwcbr* uwcbrh = HDR_UWCBR(p);

    esn = hrsn + 1; // expected sn
    
    if (!drop_out_of_order_) {
        if (sn_check[uwcbrh->sn() & 0x00ffffff]) { // Packet already processed: drop it
            incrPktInvalid();
            drop(p, 1, UWCBR_DROP_REASON_DUPLICATED_PACKET);
            return;
        }
    }
    
    sn_check[uwcbrh->sn() & 0x00ffffff] = true;
    
    if (drop_out_of_order_) {
        if (uwcbrh->sn() < esn) { // packet is out of sequence and is to be discarded
            incrPktOoseq();
            if (debug_ > 1) {
                printf("CbrModule::recv() Pkt out of sequence! cbrh->sn=%d\thrsn=%d\tesn=%d\n", uwcbrh->sn(), hrsn, esn);
            }
            drop(p, 1, UWCBR_DROP_REASON_OUT_OF_SEQUENCE);
            return;
        }
    }

    rftt = Scheduler::instance().clock() - ch->timestamp();

    if (uwcbrh->rftt_valid()) {
        double rtt = rftt + uwcbrh->rftt();
        updateRTT(rtt);
    }

    updateFTT(rftt);

    /* a new packet has been received */
    incrPktRecv();
    
    hrsn = uwcbrh->sn();
    if (drop_out_of_order_) {
        if (uwcbrh->sn() > esn) { // packet losses are observed
            incrPktLost(uwcbrh->sn() - (esn));
        }
    }

    double dt = Scheduler::instance().clock() - lrtime;
    updateThroughput(ch->size(), dt);

    lrtime = Scheduler::instance().clock();

    Packet::free(p);

    if (drop_out_of_order_) {
        if (pkts_lost + pkts_recv + pkts_last_reset != hrsn) {
            fprintf(stderr, "ERROR CbrModule::recv() pkts_lost=%d  pkts_recv=%d  hrsn=%d\n", pkts_lost, pkts_recv, hrsn);
        }
    }
}

double UwCbrModule::GetRTT() const {
    return (rttsamples > 0) ? sumrtt / rttsamples : 0;
}

double UwCbrModule::GetFTT() const {
    return (fttsamples > 0) ? sumftt / fttsamples : 0;
}

double UwCbrModule::GetRTTstd() const {
    if (rttsamples > 1) {
        double var = (sumrtt2 - (sumrtt * sumrtt / rttsamples)) / (rttsamples - 1);
        return (sqrt(var));
    } else
        return 0;
}

double UwCbrModule::GetFTTstd() const {
    if (fttsamples > 1) {
        double var = 0;
        var = (sumftt2 - (sumftt * sumftt / fttsamples)) / (fttsamples - 1);
        if (var > 0)
            return (sqrt(var));
        else return 0;
    } else {
        return 0;
    }
}

double UwCbrModule::GetPER() const {
    if (drop_out_of_order_) {
        if ((pkts_recv + pkts_lost) > 0) {
            return ((double) pkts_lost / (double) (pkts_recv + pkts_lost));
        } else {
            return 0;
        }
    } else {
        if (esn > 1)
            return (1 - (double) pkts_recv / (double) (esn - 1));
        else
            return 0;
    }
}

double UwCbrModule::GetTHR() const {
    return ((sumdt != 0) ? sumbytes * 8 / sumdt : 0);
}

void UwCbrModule::updateRTT(const double& rtt) {
    sumrtt += rtt;
    sumrtt2 += rtt*rtt;
    rttsamples++;
}

void UwCbrModule::updateFTT(const double& ftt) {
    sumftt += ftt;
    sumftt2 += ftt*ftt;
    fttsamples++;
}

void UwCbrModule::updateThroughput(const int& bytes, const double& dt) {
    sumbytes += bytes;
    sumdt += dt;

    if (debug_ > 1) {
        cerr << "bytes=" << bytes << "  dt=" << dt << endl;
    }
}

void UwCbrModule::incrPktLost(const int& npkts) {
    pkts_lost += npkts;
}

void UwCbrModule::incrPktRecv() {
    pkts_recv++;
}

void UwCbrModule::incrPktOoseq() {
    pkts_ooseq++;
}

void UwCbrModule::incrPktInvalid() {
    pkts_invalid++;
}

void UwCbrModule::resetStats() {
    pkts_last_reset += pkts_lost + pkts_recv;
    pkts_recv = 0;
    pkts_ooseq = 0;
    pkts_lost = 0;
    srtt = 0;
    sftt = 0;
    sthr = 0;
    rftt = -1;
    sumrtt = 0;
    sumrtt2 = 0;
    rttsamples = 0;
    sumftt = 0;
    sumftt2 = 0;
    fttsamples = 0;
    sumbytes = 0;
    sumdt = 0;
}

double UwCbrModule::getTimeBeforeNextPkt() {
    if (period_ < 0) {
        fprintf(stderr, "%s : Error : period <= 0", __PRETTY_FUNCTION__);
        exit(1);
    }
    if (PoissonTraffic_) {
        double u = RNG::defaultrng()->uniform_double();
        double lambda = 1 / period_;
        return (-log(u) / lambda);
    } else {
        // CBR
        return period_;
    }
}
