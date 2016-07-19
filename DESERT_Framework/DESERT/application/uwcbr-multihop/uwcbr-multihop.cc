//  -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-

#include "uwcbr-multihop.h"

#include <iterator>
#include <sstream>
#include <stdexcept>

std::string logprefix(const std::string &func) {
    std::ostringstream os;
    os << Scheduler::instance().clock() << " " << func << ": ";
    return os.str();
}
#define LOGPREFIX (logprefix(__PRETTY_FUNCTION__))

UwCbrMultihopSource::UwCbrMultihopSource() {}
UwCbrMultihopSource::~UwCbrMultihopSource() {}

int UwCbrMultihopSource::command(int argc, const char *const *argv) {
    Tcl &tcl = Tcl::instance();
    if (argc == 2) {
        if (strcasecmp(argv[1], "clearpath") == 0) {
            clear_path();
            return TCL_OK;
        }
    }
    if (argc == 4) {
	if (strcasecmp(argv[1], "appendtopath") == 0) {
	    nsaddr_t addr = (nsaddr_t) atoi(argv[2]);
	    uint16_t port = (uint16_t) atoi(argv[3]);
	    append_to_path(addr, port);
	    return TCL_OK;
	}
    }
    return UwCbrModule::command(argc, argv);
}

void UwCbrMultihopSource::append_to_path(const nsaddr_t &ipaddr, const uint16_t &port) {
    uwcbr_mh_addr addr;
    addr.ipaddr = ipaddr;
    addr.port = port;
    append_to_path(addr);
}

void UwCbrMultihopSource::append_to_path(const uwcbr_mh_addr &addr) {
    forward_path.push_back(addr);
    if (forward_path.size() == 1) {
	dstAddr_ = addr.ipaddr;
	dstPort_ = addr.port;
    }
}

void UwCbrMultihopSource::clear_path() {
    forward_path.clear();
}

void UwCbrMultihopSource::recv(Packet *p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    if (ch->ptype() != PT_UWCBR_MH) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        incrPktInvalid();
        return;
    }
    if (mhh->forward_path_begin() + 1 != mhh->forward_path_end()) {
    	if (debug_) cerr << LOGPREFIX << "Packet with more hops" << endl;
	incrPktInvalid();
    	drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
    	return;
    }
    ch->ptype() = PT_UWCBR;
    UwCbrModule::recv(p);
}

void UwCbrMultihopSource::initPkt(Packet *p) {
    UwCbrModule::initPkt(p);
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwip *iph = HDR_UWIP(p);
    hdr_uwudp *udph = HDR_UWUDP(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    ch->ptype() = PT_UWCBR_MH;
    hdr_uwcbr_mh_assign_path(mhh, forward_path.begin(), forward_path.end());

    if (debug_) {
	cerr << LOGPREFIX << "Init packet for ip " << (int) iph->daddr() <<
	    " port " << (int) udph->dport() << endl;
    }
}

void UwCbrMultihopSource::initAck(Packet *p, Packet *recvd) {
    throw logic_error("The source can't send ACKs");
}

UwCbrMultihopSink::UwCbrMultihopSink() {}
UwCbrMultihopSink::~UwCbrMultihopSink() {}

// int UwCbrMultihopSink::command(int argc, const char *const *argv) {
//     return UwCbrModule::command(argc, argv);
// }

void UwCbrMultihopSink::recv(Packet *p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwip *iph = HDR_UWIP(p);
    hdr_uwudp *udph = HDR_UWUDP(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    if (ch->ptype() != PT_UWCBR_MH) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        incrPktInvalid();
        return;
    }
    if (mhh->forward_path_begin() + 1 != mhh->forward_path_end()) {
    	if (debug_) cerr << LOGPREFIX << "Packet with more hops" << endl;
	incrPktInvalid();
    	drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
    	return;
    }
    if (debug_) cerr << LOGPREFIX << "Received a packet from " <<
		    (int) iph->saddr() << " port " <<
                    (int) udph->sport() << endl;
    hdr_uwcbr_mh_update_path(mhh, iph->saddr(), udph->sport());
    assert(mhh->forward_path_begin() == mhh->forward_path_end());
    ch->ptype() = PT_UWCBR;
    UwCbrModule::recv(p);
}

void UwCbrMultihopSink::initPkt(Packet *p) {
    throw logic_error("The sink can't send packets");
}

void UwCbrMultihopSink::initAck(Packet *p, Packet *recvd) {
    UwCbrModule::initAck(p, recvd);
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwip *iph = HDR_UWIP(p);
    hdr_uwudp *udph = HDR_UWUDP(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    hdr_uwcbr_mh *recvd_mhh = HDR_UWCBR_MH(recvd);
    ch->ptype() = PT_UWCBR_MH;

    uwcbr_mh_addr *begin = recvd_mhh->path();
    uwcbr_mh_addr *end = recvd_mhh->path() + recvd_mhh->forward_path_end();
    reverse_iterator<uwcbr_mh_addr*> rbegin(end);
    reverse_iterator<uwcbr_mh_addr*> rend(begin);
    hdr_uwcbr_mh_assign_path(mhh, rbegin, rend);

    if (debug_) {
	cerr << LOGPREFIX << "Init ACK for ip " << (int) iph->daddr() <<
	    " port " << (int) udph->dport() << endl;
    }
}

UwCbrMultihopRelay::UwCbrMultihopRelay() :
    debug_(0),
    dupack_count(0),
    dupack_thresh(1),
    first_unacked(1)
{
    bind("debug_", &debug_);
    bind("dupack_thresh", &dupack_thresh);
}

UwCbrMultihopRelay::~UwCbrMultihopRelay() {
    for (map<sn_t,Packet*>::iterator i = packet_buffer.begin();
         i != packet_buffer.end();
         i++) {
        Packet::free(i->second);
    }
    packet_buffer.clear();
}

int UwCbrMultihopRelay::command(int argc, const char *const *argv) {
    return Module::command(argc, argv);
}

void UwCbrMultihopRelay::recv(Packet *p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwip *iph = HDR_UWIP(p);
    hdr_uwudp *udph = HDR_UWUDP(p);
    hdr_uwcbr *cbrh = HDR_UWCBR(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    if (ch->ptype() != PT_UWCBR_MH) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        stats.pkts_invalid++;
        return;
    }
    if (mhh->forward_path_begin() + 1 == mhh->forward_path_end()) {
        if (debug_) cerr << LOGPREFIX << "Packet with no more hops" << endl;
        stats.pkts_invalid++;
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        return;
    }

    if (debug_) cerr << LOGPREFIX << "Received a packet from " <<
		    (int) iph->saddr() << " port " <<
                    (int) udph->sport() << " SN " <<
                    cbrh->sn() << endl;

    hdr_uwcbr_mh_update_path(mhh, iph->saddr(), udph->sport());
    uwcbr_mh_addr next_hop = mhh->path()[mhh->forward_path_begin()];
    iph->daddr() = next_hop.ipaddr;
    udph->dport() = next_hop.port;
    iph->saddr() = 0;
    udph->sport() = 0;

    if (cbrh->is_ack())
        recvAck(p);
    else
        recvPkt(p);
}

void UwCbrMultihopRelay::recvPkt(Packet *p) {
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwip *iph = HDR_UWIP(p);
    hdr_uwudp *udph = HDR_UWUDP(p);
    hdr_uwcbr *cbrh = HDR_UWCBR(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    Packet *p_buf = p->copy();
    pair<map<sn_t,Packet*>::iterator,bool> ret =
        packet_buffer.insert(pair<sn_t,Packet*>(cbrh->sn(), p_buf));
    if (!ret.second) {
        Packet::free(p_buf);
        stats.pkts_dup++;
        stats.pkts_invalid++;
        if (debug_) cerr << LOGPREFIX << "Received duplicate packet SN=" <<
                        cbrh->sn() << endl;
        drop(p, 1);
        return;
    }

    stats.pkts_recv++;

    if (debug_) cerr << LOGPREFIX << "Forward packet to " <<
                    (int) iph->daddr() << " port " <<
                    (int) udph->dport() << " SN " <<
                    cbrh->sn() << endl;
    double delay = 0;
    sendDown(p, delay);
}

void UwCbrMultihopRelay::recvAck(Packet *ack) {
    hdr_cmn *ch = HDR_CMN(ack);
    hdr_uwip *iph = HDR_UWIP(ack);
    hdr_uwudp *udph = HDR_UWUDP(ack);
    hdr_uwcbr *cbrh = HDR_UWCBR(ack);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(ack);

    // Check invalid ACK
    if (cbrh->sn() < first_unacked) {
        if (debug_)
            cerr << LOGPREFIX << "Invalid ACK for old packet SN = " <<
                cbrh->sn() << endl;
        stats.acks_invalid++;
        drop(ack, 1, UWCBR_INVALID_ACK);
        return;
    }

    // Check DUPACK
    if (cbrh->sn() == first_unacked) {
	if (debug_) cerr << LOGPREFIX << "Duplicate ACK SN= " <<
                        first_unacked << endl;
	stats.acks_dup++;
	dupack_count++;
        map<sn_t,Packet*>::const_iterator i = packet_buffer.find(first_unacked);
        if (i == packet_buffer.end()) {
            if (debug_) cerr << LOGPREFIX <<
                            "Duplicate ACK for an unknown packet SN " <<
                            first_unacked << endl;
            if (debug_) cerr << LOGPREFIX << "Forward ACK to " <<
                            (int) iph->daddr() << " port " <<
                            (int) udph->dport() << " SN " <<
                            cbrh->sn() << endl;
            double delay = 0;
            sendDown(ack, delay);
        }
        else if (dupack_count >= dupack_thresh) {
	    dupack_count = 0;
	    stats.pkts_retx_dupack++;
            Packet *retx = i->second->copy();
            hdr_cmn *retx_ch = HDR_CMN(retx);
            hdr_uwip *retx_iph = HDR_UWIP(retx);
            hdr_uwudp *retx_udph = HDR_UWUDP(retx);
            hdr_uwcbr *retx_cbrh = HDR_UWCBR(retx);
            if (debug_) cerr << LOGPREFIX << "Reforward packet to " <<
                            (int) retx_iph->daddr() << " port " <<
                            (int) retx_udph->dport() << " SN " <<
                            retx_cbrh->sn() << endl;
            double delay = 0;
            sendDown(retx, delay);
        }
        Packet::free(ack);
        return;
    }

    // Normal ACK
    dupack_count = 0;
    for (; first_unacked < cbrh->sn(); first_unacked++) {
	map<sn_t,Packet*>::iterator i = packet_buffer.find(first_unacked);
	assert (i != packet_buffer.end());
	Packet::free(i->second);
	packet_buffer.erase(i);
    }
    stats.acks_forw++;

    if (debug_) cerr << LOGPREFIX << "Forward ACK to " <<
                    (int) iph->daddr() << " port " <<
                    (int) udph->dport() << " SN " <<
                    cbrh->sn() << endl;
    double delay = 0;
    sendDown(ack, delay);
}
