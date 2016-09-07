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
#include <sstream>
#include <stdexcept>
#include <stdint.h>

std::string logprefix(const std::string &func) {
    std::ostringstream os;
    os << Scheduler::instance().clock() << " " << func << ": ";
    return os.str();
}
#define LOGPREFIX (logprefix(__PRETTY_FUNCTION__))

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

int UwCbrModule::uidcnt_ = 0;

UwCbrModule::UwCbrModule() :
    dstPort_(0),
    dstAddr_(0),
    priority_(0),
    peer_addr(0),
    peer_port(0),
    sn_check(numeric_limits<sn_t>::max(), false),
    ack_check(numeric_limits<sn_t>::max(), false),
    dupack_count(0),
    dupack_thresh(1),
    PoissonTraffic_(0),
    period_(0),
    pktSize_(0),
    debug_(0),
    drop_out_of_order_(0),
    timeout_(0),
    sendTmr_(this),
    stopped(true),
    use_arq(0),
    txsn(1),
    ack_sn(1),
    tx_window(1),
    hrsn(0),
    rx_window(1),
    esn(1)
{
    // binding to TCL variables
    bind("PoissonTraffic_", &PoissonTraffic_);
    bind("debug_", &debug_);
    bind("destAddr_", &dstAddr_);
    bind("destPort_", &dstPort_);
    bind("drop_out_of_order_", &drop_out_of_order_);
    bind("dupack_thresh", &dupack_thresh);
    bind("packetSize_", &pktSize_);
    bind("period_", &period_);
    bind("rx_window", &rx_window);
    bind("timeout_", &timeout_);
    bind("tx_window", &tx_window);
    bind("use_arq", &use_arq);
    bind("use_rtt_timeout", &use_rtt_timeout);
}

UwCbrModule::~UwCbrModule() {
    for (map<sn_t,UwRetxTimer*>::iterator i = packet_retx_timers.begin();
         i != packet_retx_timers.end();
         i++) {
        delete i->second;
    }

    for (map<sn_t,Packet*>::iterator i = packet_buffer.begin();
         i != packet_buffer.end();
         i++) {
        Packet::free(i->second);
    }

    while (!recv_queue.empty()) {
        Packet *p = recv_queue.top();
        recv_queue.pop();
        Packet::free(p);
    }

    while (!send_queue.empty()) {
        Packet *p = send_queue.top();
        send_queue.pop();
        Packet::free(p);
    }
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
        } else if (strcasecmp(argv[1], "getdelay") == 0) {
            tcl.resultf("%f", stats.delay.avg());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getdelaystd") == 0) {
            tcl.resultf("%f", stats.delay.stddev());
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
            tcl.resultf("%d", (txsn - 1) < max_tx_win_sn() ? txsn-1 : max_tx_win_sn());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getretxpkts") == 0) {
            tcl.resultf("%d", stats.pkts_retx_timeout + stats.pkts_retx_dupack);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getdupackretxpkts") == 0) {
            tcl.resultf("%d", stats.pkts_retx_dupack);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "gettimeoutretxpkts") == 0) {
            tcl.resultf("%d", stats.pkts_retx_timeout);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getgeneratedpkts") == 0) {
            tcl.resultf("%d", txsn-1);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrecvpkts") == 0) {
            tcl.resultf("%d", stats.pkts_recv);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getprocpkts") == 0) {
            tcl.resultf("%d", stats.pkts_proc);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getinvalidpkts") == 0) {
            tcl.resultf("%d", stats.pkts_invalid);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getduppkts") == 0) {
            tcl.resultf("%d", stats.pkts_dup);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getooseqpkts") == 0) {
            tcl.resultf("%d", stats.pkts_ooseq);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getlostpkts") == 0) {
            tcl.resultf("%d", stats.pkts_lost);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrecvacks") == 0) {
            tcl.resultf("%d", stats.acks_recv);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getdupacks") == 0) {
            tcl.resultf("%d", stats.acks_dup);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getinvalidacks") == 0) {
            tcl.resultf("%d", stats.acks_invalid);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getsentacks") == 0) {
            tcl.resultf("%d", stats.acks_sent);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getsentdupacks") == 0) {
            tcl.resultf("%d", stats.acks_dup_sent);
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
            fprintf(stderr, "CbrModule::command() resetStats %s, pkts_last_reset=%d, hrsn=%d, txsn=%d\n", tag_, stats.pkts_last_reset, hrsn, txsn);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "printidspkts") == 0) {
            this->printIdsPkts();
            return TCL_OK;
        }
    }
    return Module::command(argc, argv);
}

void UwCbrModule::initPkt(Packet* p) {
    if (txsn >= numeric_limits<sn_t>::max())
        throw overflow_error("Reached the max value for the SN");

    hdr_cmn* ch = hdr_cmn::access(p);
    ch->ptype() = PT_UWCBR;
    ch->size()  = pktSize_;

    hdr_uwip* uwiph  = hdr_uwip::access(p);
    uwiph->daddr()   = (nsaddr_t) dstAddr_;
    uwiph->saddr() = 0;

    hdr_uwudp* uwudp = hdr_uwudp::access(p);
    uwudp->dport()   = (uint16_t) dstPort_;
    //uwudp->sport() = 0;

    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    uwcbrh->sn()       = txsn++;
    uwcbrh->priority() = (char) priority_;
    uwcbrh->gen_timestamp() = Scheduler::instance().clock();
}

void UwCbrModule::initAck(Packet *p, Packet *recvd) {
    hdr_cmn* ch = hdr_cmn::access(p);
    ch->ptype() = PT_UWCBR;
    ch->size()  = getCbrHeaderSize();

    hdr_uwip* uwiph  = hdr_uwip::access(p);
    hdr_uwip *uwiph_recvd = hdr_uwip::access(recvd);
    uwiph->daddr()   = uwiph_recvd->saddr();

    hdr_uwudp* uwudp = hdr_uwudp::access(p);
    hdr_uwudp* uwudp_recvd = hdr_uwudp::access(recvd);
    uwudp->dport()   = uwudp_recvd->sport();

    if (peer_addr == 0 && peer_port == 0) {
        peer_addr = uwiph_recvd->saddr();
        peer_port = uwudp_recvd->sport();
        assert(peer_addr != 0);
        assert(peer_port != 0);
    }
    else {
        // if (debug_) {
        //     cerr << LOGPREFIX << "Source address has changed: " << peer_addr << "->" << (int) uwiph->saddr() << endl;
        //     cerr << LOGPREFIX << "Source port has changed: " << peer_port << "->" << (int) uwudp->sport() << endl;
        // }
        assert(peer_addr == uwiph_recvd->saddr());
        assert(peer_port == uwudp_recvd->sport());
    }

    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    hdr_uwcbr* uwcbrh_recvd  = HDR_UWCBR(recvd);
    uwcbrh->is_ack() = true;
    uwcbrh->sn()       = esn;
    uwcbrh->priority() = uwcbrh_recvd->priority();
    uwcbrh->gen_timestamp() = Scheduler::instance().clock();

}

void UwCbrModule::sendPkt(Packet *p, double delay) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *uwcbrh = HDR_UWCBR(p);

    ch->uid()   = uidcnt_++;
    ch->timestamp() = Scheduler::instance().clock();

    if (stats.rftt >= 0) {
        uwcbrh->rftt() = stats.rftt;
        uwcbrh->rftt_valid() = true;
    } else {
        uwcbrh->rftt_valid() = false;
    }

    if (use_arq && packet_buffer.find(uwcbrh->sn()) == packet_buffer.end()) {
        pair<map<sn_t,Packet*>::iterator,bool> ret =
            packet_buffer.insert(pair<sn_t,Packet*>(uwcbrh->sn(), p->copy()));
        assert(ret.second);

        UwRetxTimer *timer = new UwRetxTimer(this, uwcbrh->sn());
        pair<map<sn_t,UwRetxTimer*>::iterator,bool> ret_timer =
            packet_retx_timers.insert(pair<sn_t,UwRetxTimer*>(uwcbrh->sn(), timer));
        assert(ret_timer.second);
        timer->sched(getRetxTimeout());
    }

    if (debug_ > 10)
        printf("CbrModule(%d)::sendPkt, send a pkt (%d) with sn: %d\n",
               getId(), ch->uid(), uwcbrh->sn());
    sendDown(p, delay);
}

void UwCbrModule::sendPkt() {
    double delay      = 0;
    Packet* p         = Packet::alloc();
    initPkt(p);

    hdr_uwcbr* uwcbrh = HDR_UWCBR(p);

    if (uwcbrh->sn() > max_tx_win_sn() && use_arq) {
        if (debug_) cerr << "Tx window is full, enqueue packet SN=" <<
                        uwcbrh->sn() << endl;
        send_queue.push(p);
    }
    else {
        sendPkt(p, delay);
    }
}

void UwCbrModule::sendPktLowPriority() {
    if (txsn >= numeric_limits<sn_t>::max()) {
        cerr << "Reached the max value for the SN, " << numeric_limits<sn_t>::max()-1 << endl;
        return;
    }
    double delay       = 0;
    Packet* p          = Packet::alloc();
    initPkt(p);
    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    uwcbrh->priority() = 0;

    if (uwcbrh->sn() > max_tx_win_sn()) {
        if (debug_) cerr << "Tx window is full, enqueue packet SN=" << uwcbrh->sn() << endl;
        send_queue.push(p);
    }
    else {
        sendPkt(p, delay);
    }
}

void UwCbrModule::sendPktHighPriority() {
    if (txsn >= numeric_limits<sn_t>::max()) {
        cerr << "Reached the max value for the SN, " << numeric_limits<sn_t>::max()-1 << endl;
        return;
    }
    double delay       = 0;
    Packet* p          = Packet::alloc();
    initPkt(p);
    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    uwcbrh->priority() = 1;

    if (uwcbrh->sn() > max_tx_win_sn()) {
        if (debug_) cerr << "Tx window is full, enqueue packet SN=" << uwcbrh->sn() << endl;
        send_queue.push(p);
    }
    else {
        sendPkt(p, delay);
    }
}

void UwCbrModule::resendPkt(sn_t sn) {
    Packet *p = packet_buffer[sn]->copy();
    UwRetxTimer *timer = packet_retx_timers[sn];
    hdr_cmn* ch = hdr_cmn::access(p);
    hdr_uwcbr* uwcbrh = HDR_UWCBR(p);
    timer->resched(getRetxTimeout());

    double delay = 0;
    if (debug_ > 10)
        printf("CbrModule(%d)::resendPkt, resend a pkt (%d) with sn: %d\n", getId(), ch->uid(), uwcbrh->sn());
    sendPkt(p, delay);
}

void UwCbrModule::sendAck(Packet *recvd) {
    Packet *ack = Packet::alloc();
    initAck(ack, recvd);

    hdr_cmn* ch = hdr_cmn::access(ack);
    hdr_uwcbr* uwcbrh = HDR_UWCBR(ack);
    ch->uid()   = uidcnt_++;
    ch->timestamp()    = Scheduler::instance().clock();

    if (stats.rftt >= 0) {
        uwcbrh->rftt() = stats.rftt;
        uwcbrh->rftt_valid() = true;
    } else {
        uwcbrh->rftt_valid() = false;
    }

    if (debug_ > 10)
        printf("CbrModule(%d)::sendAck, send a pkt (%d) with sn: %d\n", getId(), ch->uid(), uwcbrh->sn());
    double delay = 0;
    sendDown(ack, delay);
}

void UwCbrModule::recv(Packet* p) {
    hdr_cmn* ch = hdr_cmn::access(p);
    if (debug_ > 10)
        printf("CbrModule(%d)::recv pktId %d\n", getId(), ch->uid());

    // Incorrect pkt type -> invalid
    if (ch->ptype() != PT_UWCBR) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        incrPktInvalid();
        return;
    }

    stats.update_ftt_rtt(p);

    hdr_uwcbr* uwcbrh = HDR_UWCBR(p);

    if (uwcbrh->is_ack() && use_arq) {
        recvAck(p);
        return;
    }

    // Check if duplicate
    if (sn_check[uwcbrh->sn() & 0x00ffffff]) {
	stats.pkts_dup++;
	stats.pkts_invalid++;
	if (use_arq) {
	    sendAck(p);
	    stats.acks_dup_sent++;
	}
	drop(p, 1, UWCBR_DROP_REASON_DUPLICATED_PACKET);
	return;
    }

    // Check if out of rx window
    if (uwcbrh->sn() > max_rx_win_sn() && use_arq) {
        if (debug_) cerr << "Packet SN=" << uwcbrh->sn() <<
                        " out of window [" << esn << "," << max_rx_win_sn() <<
                        "]" << endl;
        incrPktInvalid();
        drop(p, 1, UWCBR_DROP_REASON_OUT_OF_SEQUENCE);
        return;
    }

    // Check if out of sequence
    if (uwcbrh->sn() != esn) {
        stats.acks_dup_sent++;
        if (debug_ > 1)
            printf("CbrModule::recv() Pkt out of sequence! sn=%d\thrsn=%d\tesn=%d\n",
                   uwcbrh->sn(), hrsn, esn);
        if (!use_arq && drop_out_of_order_) {
            incrPktOoseq();
            drop(p, 1, UWCBR_DROP_REASON_OUT_OF_SEQUENCE);
            return;
        }
    }
    else if (use_arq) {
        stats.acks_sent++;
    }

    sn_check[uwcbrh->sn() & 0x00ffffff] = true;
    hrsn = max(uwcbrh->sn(), hrsn);
    stats.pkts_recv++;
    recv_queue.push(p->refcopy());
    processOrderedPackets();
    if (use_arq) sendAck(p);
    Packet::free(p);
}

void UwCbrModule::recvAck(Packet *p) {
    hdr_cmn* ch = hdr_cmn::access(p);
    hdr_uwcbr* uwcbrh = HDR_UWCBR(p);
    if (debug_ > 10)
        printf("CbrModule(%d)::recvAck pktId %d\n", getId(), ch->uid());

    // Check invalid ACK
    if (uwcbrh->sn() < ack_sn) {
        if (debug_ > 10)
            cerr << LOGPREFIX << "Invalid ACK for old packet with SN " << uwcbrh->sn() << endl;
        stats.acks_invalid++;
        drop(p, 1, UWCBR_INVALID_ACK);
        return;
    }

    // Check DUPACK
    if (uwcbrh->sn() == ack_sn) {
        if (debug_) cerr << LOGPREFIX << "Duplicate ACK for SN " << ack_sn << endl;
        stats.acks_dup++;
        dupack_count++;
        if (dupack_count >= dupack_thresh) {
            dupack_count = 0;
            stats.pkts_retx_dupack++;
            resendPkt(ack_sn);
        }
        Packet::free(p);
        return;
    }

    // Normal ACK
    stats.acks_recv++;
    dupack_count = 0;
    for (; uwcbrh->sn() > ack_sn; ack_sn++) {
        map<sn_t,Packet*>::iterator i = packet_buffer.find(ack_sn);
        assert (i != packet_buffer.end());
        Packet::free(i->second);
        packet_buffer.erase(i);

        map<sn_t,UwRetxTimer*>::iterator j = packet_retx_timers.find(ack_sn);
        j->second->force_cancel();
        delete j->second;
        packet_retx_timers.erase(j);

        ack_check[ack_sn] = true;
    }
    Packet::free(p);
    if (debug_) cerr << "Advance the window to SN=" << ack_sn << endl;
    slideTxWindow();
}

void UwCbrModule::processOrderedPackets() {
    if (debug_) cerr << "Process packet queue" << endl;
    Packet *p;
    hdr_cmn *ch;
    hdr_uwcbr *uwcbrh;
    while (!recv_queue.empty()) {
        p = recv_queue.top();
        ch = HDR_CMN(p);
        uwcbrh = HDR_UWCBR(p);

        if (debug_) cerr << "Top SN=" << uwcbrh->sn() << endl;

        if (uwcbrh->sn() != esn) {
            if (use_arq) break;
            else incrPktLost(uwcbrh->sn() - esn);
        }

        if (debug_) cerr << "Top has SN=esn=" << esn << endl;

        stats.update_delay(p);

        stats.pkts_proc++;

        double dt = Scheduler::instance().clock() - stats.lrtime;
        updateThroughput(ch->size(), dt);
        stats.lrtime = Scheduler::instance().clock();

        recv_queue.pop();
        if (use_arq) assert(esn == uwcbrh->sn());
        esn = uwcbrh->sn()+1;
        Packet::free(p);
    }
}

void UwCbrModule::slideTxWindow() {
    if (stopped) return;
    while (!send_queue.empty()) {
        Packet *q = send_queue.top();
        hdr_uwcbr *qh = HDR_UWCBR(q);
        hdr_cmn *ch = HDR_CMN(q);
        if (qh->sn() > max_tx_win_sn()) break;
        send_queue.pop();
        if (debug_) cerr << "Send a packet from the send_queue SN=" << qh->sn() << endl;
        double delay = 0;
        sendPkt(q, delay);
    }
}

void UwCbrModule::transmit() {
    sendPkt();
    sendTmr_.resched(getTimeBeforeNextPkt()); // schedule next transmission
}

void UwCbrModule::start() {
    stopped = false;
    sendTmr_.resched(getTimeBeforeNextPkt());
}

void UwCbrModule::stop() {
    sendTmr_.force_cancel();
    for (map<sn_t,UwRetxTimer*>::iterator i = packet_retx_timers.begin();
         i != packet_retx_timers.end();
         i++) {
        i->second->force_cancel();
    }
    stopped = true;
}

double UwCbrModule::GetRTT() const {
    return stats.rtt.avg();
}

double UwCbrModule::GetFTT() const {
    return stats.ftt.avg();
}

double UwCbrModule::GetRTTstd() const {
    return stats.rtt.stddev();
}

double UwCbrModule::GetFTTstd() const {
    return stats.ftt.stddev();
}

double UwCbrModule::GetPER() const {
    if (use_arq) return 0;
    if (drop_out_of_order_) {
        if ((stats.pkts_recv + stats.pkts_lost) > 0) {
            return ((double) stats.pkts_lost / (double) (stats.pkts_recv + stats.pkts_lost));
        } else {
            return 0;
        }
    } else {
        if (esn > 1)
            return (1 - (double) stats.pkts_recv / (double) (esn - 1));
        else
            return 0;
    }
}

double UwCbrModule::GetTHR() const {
    return ((stats.sumdt != 0) ? stats.sumbytes * 8 / stats.sumdt : 0);
}

void UwCbrModule::updateRTT(const double& rtt) {
    stats.rtt.update(rtt);
}

void UwCbrModule::updateFTT(const double& ftt) {
    stats.ftt.update(ftt);
}

void UwCbrModule::updateThroughput(const int& bytes, const double& dt) {
    stats.sumbytes += bytes;
    stats.sumdt += dt;

    if (debug_ > 1) {
        cerr << "bytes=" << bytes << "  dt=" << dt << endl;
    }
}

void UwCbrModule::incrPktLost(const int& npkts) {
    stats.pkts_lost += npkts;
}

void UwCbrModule::incrPktRecv() {
    stats.pkts_recv++;
}

void UwCbrModule::incrPktOoseq() {
    stats.pkts_ooseq++;
}

void UwCbrModule::incrPktInvalid() {
    stats.pkts_invalid++;
}

void UwCbrModule::resetStats() {
    stats.reset();
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

void UwCbrModule::retransmit_first() {
    stats.pkts_retx_timeout++;
    resendPkt(ack_sn);
}

avg_stddev_stat::avg_stddev_stat(){
    reset();
}

void avg_stddev_stat::update(const double &val) {
    sum += val;
    sum2 += val*val;
    samples++;
}

double avg_stddev_stat::avg() const {
    if (samples > 0) return sum / samples;
    else return 0;
}

double avg_stddev_stat::stddev() const {
    if (samples > 1) {
        double var = (sum2 - (sum*sum / samples)) / (samples-1);
        return var > 0 ? sqrt(var) : 0;
    }
    else return 0;
}

void avg_stddev_stat::reset() {
    sum = 0;
    sum2 = 0;
    samples = 0;
}

void uwcbr_stats::update_delay(const Packet *const &p) {
    hdr_uwcbr *uwcbrh = HDR_UWCBR(p);
    double d = Scheduler::instance().clock() - uwcbrh->gen_timestamp();
    delay.update(d);
}

void uwcbr_stats::update_ftt_rtt(const Packet *const &p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *uwcbrh = HDR_UWCBR(p);

    rftt = Scheduler::instance().clock() - ch->timestamp();
    ftt.update(rftt);

    if (uwcbrh->rftt_valid())
        rtt.update(rftt + uwcbrh->rftt());
}

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
