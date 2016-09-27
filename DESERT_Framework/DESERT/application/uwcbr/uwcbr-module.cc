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

#include <limits>
#include <rng.h>
#include <stdexcept>
#include <stdint.h>

extern packet_t PT_UWCBR;
int hdr_uwcbr::offset_; /// Offset used to access in <i>hdr_uwcbr</i> packets header.

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
stringstream UwCbrModule::nullss;

UwCbrModule::UwCbrModule() :
    tx_window(1),
    rx_window(1),
    next_ack(1),
    next_tx(1),
    next_sn(1),
    next_recv(1),
    dupack_thresh(1),
    timeout_(1),
    PoissonTraffic_(1),
    period_(60),
    pktSize_(500),
    drop_out_of_order_(1),
    sendTmr_(this),
    retxTimer(this)
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
    for (queue_t::const_iterator i = send_queue.begin();
         i != send_queue.end();
         i++) {
        Packet::free(i->second);
    }

    for (queue_t::const_iterator i = recv_queue.begin();
         i != recv_queue.end();
         i++) {
        Packet::free(i->second);
    }
}

void UwCbrModule::recv(Packet* p) {
    hdr_cmn* ch = HDR_CMN(p);
    hdr_uwcbr *cbrh = HDR_UWCBR(p);

    if (ch->ptype() != PT_UWCBR) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        stats.pkts_invalid++;
    }
    else if (cbrh->is_ack()) recvAck(p);
    else recvData(p);
}

void UwCbrModule::transmit() {
    transmit((char) priority_);
}

void UwCbrModule::transmit(char priority) {
    Packet *p = Packet::alloc();
    initPkt(p, priority);

    hdr_uwcbr *cbrh = HDR_UWCBR(p);
    log(DEBUG) << "Generated packet SN=" << cbrh->sn() << endl;
    send_queue[cbrh->sn()] = p;
    send_from_queue();
}

void UwCbrModule::retransmit_first(bool timeout) {
    if (!use_arq) throw logic_error("Attempted a retx with ARQ disabled");

    Packet *p = send_queue[next_ack]->copy();
    set_uid_and_times(p);

    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *cbrh = HDR_UWCBR(p);
    log(DEBUG) << "Retx a pkt uid="<<ch->uid()<<
        " SN="<<cbrh->sn()<<endl;
    if (timeout) {
        stats.pkts_retx_timeout++;
        dupack_backoff = false;
    }
    else {
        stats.pkts_retx_dupack++;
    }
    sendDown(p, 0);
}

void UwCbrModule::start() {
    sendTmr_.resched(getTimeBeforeNextPkt());
    log(INFO) << "Started" << endl;
}

void UwCbrModule::stop() {
    sendTmr_.force_cancel();
    log(INFO) << "Stopped" << endl;
}

bool UwCbrModule::stopped() {
    return sendTmr_.status() == TIMER_IDLE;
}

double UwCbrModule::getTimeBeforeNextPkt() {
    if (period_ <= 0) {
        fprintf(stderr, "%s : Error : period <= 0", __PRETTY_FUNCTION__);
        exit(1);
    }
    if (PoissonTraffic_) {
        double u = RNG::defaultrng()->uniform_double();
        double lambda = 1 / period_;
        return -std::log(u) / lambda;
    } else {
        // CBR
        return period_;
    }
}

double UwCbrModule::getRetxTimeout() {
    if (!use_rtt_timeout || stats.rtt.samples() < 1) return timeout_;
    double rtt = stats.rtt.avg();
    return rtt + 4 * stats.rtt.stddev();
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
            if (stats.rtt.samples() > 0)
                tcl.resultf("%f", GetRTT());
            else
                tcl.result("NaN");
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getftt") == 0) {
            if (stats.ftt.samples() > 0)
                tcl.resultf("%f", GetFTT());
            else
                tcl.result("NaN");
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getper") == 0) {
            tcl.resultf("%f", GetPER());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getthr") == 0) {
            tcl.resultf("%f", GetTHR());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getdelay") == 0) {
            if (stats.delay.samples() > 0)
                tcl.resultf("%f", stats.delay.avg());
            else
                tcl.result("NaN");
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getdelaystd") == 0) {
            if (stats.delay.samples() > 0)
                tcl.resultf("%f", stats.delay.stddev());
            else
                tcl.result("NaN");
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getcbrheadersize") == 0) {
            tcl.resultf("%d", getCbrHeaderSize());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrttstd") == 0) {
            if (stats.rtt.samples() > 0)
                tcl.resultf("%f", GetRTTstd());
            else
                tcl.result("NaN");
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getfttstd") == 0) {
            if (stats.ftt.samples() > 0)
                tcl.resultf("%f", GetFTTstd());
            else
                tcl.result("NaN");
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getsentpkts") == 0) {
            tcl.resultf("%d", GetSentPkts());
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
            tcl.resultf("%d", next_sn-1);
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
            tcl.resultf("%d", stats.acks_old);
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
            transmit();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "sendPktLowPriority") == 0) {
            transmit(0);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "sendPktHighPriority") == 0) {
            transmit(1);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "resetStats") == 0) {
            resetStats();
            fprintf(stderr, "CbrModule::command() resetStats %s, pkts_last_reset=%d, txsn=%d\n", tag_, stats.pkts_last_reset, next_sn);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "printidspkts") == 0) {
            this->printIdsPkts();
            return TCL_OK;
        }
    }
    return Module::command(argc, argv);
}

double UwCbrModule::GetRTT() const {
    return stats.rtt.avg();
}

double UwCbrModule::GetFTT() const {
    return stats.ftt.avg();
}

int UwCbrModule::GetSentPkts() const {
    return next_tx - 1;
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
        if (next_recv > 1)
            return (1 - (double) stats.pkts_recv / (double) (next_recv - 1));
        else
            return 0;
    }
}

double UwCbrModule::GetTHR() const {
    return stats.throughput();
}

double UwCbrModule::GetRTTstd() const {
    return stats.rtt.stddev();
}

double UwCbrModule::GetFTTstd() const {
    return stats.ftt.stddev();
}

void UwCbrModule::resetStats() {
    stats.reset();
}

void UwCbrModule::send_from_queue() {
    while (sendable_count() > 0) {
        queue_t::iterator i = send_queue.find(next_tx);
        Packet *p = i->second->copy();
        set_uid_and_times(p);

        hdr_cmn *ch = HDR_CMN(p);
        hdr_uwcbr *cbrh = HDR_UWCBR(p);
        log(DEBUG) << "Send a pkt uid="<<ch->uid()<<" SN="<<cbrh->sn()<<endl;

        sendDown(p, 0);
        next_tx++;

        if (!use_arq) {
            Packet::free(i->second);
            send_queue.erase(i);
            next_ack++;
            assert(next_tx == next_sn && next_ack == next_sn);
        }
        else if (retxTimer.status() == TIMER_IDLE) {
            log(DEBUG) << "First packet, start the retx timer" << endl;
            retxTimer.sched(getRetxTimeout());
        }
    }
}

int UwCbrModule::sendable_count() {
    int max_sendable = send_queue.size();
    if (use_arq) max_sendable = min(tx_window, max_sendable);
    int sent = next_tx - next_ack;
    return max_sendable - sent;
}

void UwCbrModule::initPkt(Packet *p, char priority) {
    if (next_sn >= numeric_limits<sn_t>::max())
        throw overflow_error("Reached the max value for the SN");

    hdr_cmn* ch = hdr_cmn::access(p);
    ch->ptype() = PT_UWCBR;
    ch->size()  = pktSize_ + getCbrHeaderSize();

    hdr_uwip* uwiph  = hdr_uwip::access(p);
    uwiph->daddr()   = (nsaddr_t) dstAddr_;
    uwiph->saddr() = 0;

    hdr_uwudp* uwudp = hdr_uwudp::access(p);
    uwudp->dport()   = (uint16_t) dstPort_;
    //uwudp->sport() = 0;

    hdr_uwcbr* uwcbrh  = hdr_uwcbr::access(p);
    uwcbrh->sn()       = next_sn++;
    uwcbrh->priority() = priority;
    uwcbrh->gen_timestamp() = Scheduler::instance().clock();
}

void UwCbrModule::set_uid_and_times(Packet *p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *cbrh = HDR_UWCBR(p);

    ch->uid()   = uidcnt_++;
    ch->timestamp() = Scheduler::instance().clock();

    if (stats.ftt.samples() > 0) {
        cbrh->rftt() = stats.ftt.last_sample();
        cbrh->rftt_valid() = true;
    } else {
        cbrh->rftt_valid() = false;
    }
}

void UwCbrModule::recvData(Packet *p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *cbrh = HDR_UWCBR(p);

    stats.update_ftt_rtt(p);
    sn_t sn = cbrh->sn();
    if (use_arq) {
        if (sn >= next_recv + rx_window) { // Out of the rx window
            log(DEBUG) << "Packet uid="<<ch->uid()<<" SN="<<sn<<
                " out of rx window ["<<next_recv<<", "
                       <<next_recv+rx_window-1<<"]"<<endl;
            sendAck(p);
            stats.acks_dup_sent++;
            drop(p, 1, UWCBR_DROP_REASON_OUT_OF_SEQUENCE);
            stats.pkts_ooseq++;
        }
        else if (sn < next_recv ||
                 recv_queue.find(sn) != recv_queue.end()) { // Duplicate packet
            stats.pkts_dup++;
            //stats.pkts_invalid++; // <- the old CBR counted duplicates as invalid
            log(DEBUG) << "Packet uid="<<ch->uid()<<" SN="<<sn<<
                " is duplicate" << endl;
            sendAck(p);
            stats.acks_dup_sent++;
            drop(p, 1, UWCBR_DROP_REASON_DUPLICATED_PACKET);
        }
        else { // New packet with SN in the rx window
            stats.pkts_recv++;
            log(DEBUG) << "Packet uid="<<ch->uid()<<" SN="<<sn<<
                " received" << endl;
            recv_queue[sn] = p->refcopy();
            pass_up_from_queue();
            sendAck(p);
            stats.acks_sent++;
            Packet::free(p);
        }
    }
    else { // recv data pkt without ARQ
        if (sn < next_recv) {
            stats.pkts_dup++;
            log(DEBUG) << "Packet uid="<<ch->uid()<<" SN="<<sn<<
                " is duplicate" << endl;
            drop(p, 1, UWCBR_DROP_REASON_DUPLICATED_PACKET);
        }
        else {
            stats.pkts_recv++;
            recv_queue[sn] = p;
            log(DEBUG) << "Packet uid="<<ch->uid()<<" SN="<<sn<<
                " received" << endl;
            pass_up_from_queue();
        }
    }
}

void UwCbrModule::pass_up_from_queue() {
    for (queue_t::iterator i = recv_queue.begin();
         i != recv_queue.end();
         i++) {
        sn_t sn = i->first;
        if (sn != next_recv) {
            if (use_arq) break;
            else {
                int lost = sn - next_recv;
                stats.pkts_lost += lost;
                log(DEBUG) << "Lost "<<lost<<" pkts before SN="<<sn<<endl;
            }
        }

        Packet *p = i->second;
        hdr_cmn *ch = HDR_CMN(p);
        hdr_uwcbr *cbrh = HDR_UWCBR(p);

        stats.update_delay(p);
        stats.update_throughput(p);
        stats.pkts_proc++;

        next_recv = sn + 1;
        log(DEBUG) << "Packet uid="<<ch->uid()<<" SN="<<cbrh->sn()<<
            " processed" << endl;
        recv_queue.erase(i);
        Packet::free(p);
    }
}

void UwCbrModule::recvAck(Packet *p) {
    if (!use_arq) throw logic_error("Received an ACK with ARQ disabled");

    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr *cbrh = HDR_UWCBR(p);

    stats.update_ftt_rtt(p);
    sn_t sn = cbrh->sn();
    if (sn < next_ack) {
        log(DEBUG) << "Received ACK for a packet before next_ack=" <<
            next_ack << ", SN=" << sn << endl;
        stats.acks_old++;
        drop(p, 1, UWCBR_OLD_ACK);
    }
    else if (sn == next_ack) {
        if (send_queue.empty()) {
            log(DEBUG) << "Duplicate ACK but empty queue SN="<< next_ack << endl;
            drop(p, 1, UWCBR_ACK_EMPTY);
        }
        else {
            log(DEBUG) << "Duplicate ACK for SN " << next_ack << endl;
            stats.acks_dup++;
            dupack_count++;
            if (dupack_count >= dupack_thresh) {
                dupack_count = 0;
                if (!stopped() && !dupack_backoff) {
                    dupack_backoff = true;
                    retransmit_first(false);
                    retxTimer.resched(getRetxTimeout());
                }
            }
            drop(p, 1, UWCBR_DUPACK);
        }
    }
    else {
        stats.acks_recv++;
        dupack_count = 0;
        dupack_backoff = false;
        log(DEBUG) << "Received ACK for SN " << sn << endl;
        for (queue_t::iterator i = send_queue.begin();
             i != send_queue.end();
             i++) {
            if (i->first >= sn) break;
            Packet::free(i->second);
            send_queue.erase(i);
            next_ack++;
        }
        Packet::free(p);
        if (send_queue.empty())
            retxTimer.cancel();
        else {
            send_from_queue();
            retxTimer.resched(getRetxTimeout());
        }
    }
}

void UwCbrModule::initAck(Packet *p, Packet *recvd) {
    hdr_cmn* ch = hdr_cmn::access(p);
    ch->ptype() = PT_UWCBR;
    ch->size()  = getCbrHeaderSize();

    hdr_uwip* uwiph  = hdr_uwip::access(p);
    hdr_uwip *uwiph_recvd = hdr_uwip::access(recvd);
    uwiph->daddr()   = uwiph_recvd->saddr();
    uwiph->saddr() = 0;

    hdr_uwudp* uwudp = hdr_uwudp::access(p);
    hdr_uwudp* uwudp_recvd = hdr_uwudp::access(recvd);
    uwudp->dport()   = uwudp_recvd->sport();
    //uwudp->sport() = 0;

    if (peer_addr == 0 && peer_port == 0) {
        peer_addr = uwiph_recvd->saddr();
        peer_port = uwudp_recvd->sport();
    }
    else {
        assert(peer_addr == uwiph_recvd->saddr());
        assert(peer_port == uwudp_recvd->sport());
    }

    hdr_uwcbr* uwcbrh  = HDR_UWCBR(p);
    //hdr_uwcbr* uwcbrh_recvd  = HDR_UWCBR(recvd);
    uwcbrh->is_ack() = true;
    uwcbrh->sn() = next_recv;
    uwcbrh->priority() = (char) priority_;
    uwcbrh->gen_timestamp() = Scheduler::instance().clock();
}

void UwCbrModule::sendAck(Packet *recvd) {
    Packet *ack = Packet::alloc();
    initAck(ack, recvd);
    set_uid_and_times(ack);

    hdr_cmn *ch = HDR_CMN(ack);
    hdr_uwcbr *cbrh = HDR_UWCBR(ack);
    log(DEBUG) << "Send an ACK uid="<<ch->uid()<<" SN="<<cbrh->sn()<<endl;

    sendDown(ack, 0);
}

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
