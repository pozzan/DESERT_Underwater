//  -*- mode: c++; c-basic-offset: 4; -*-

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
	if (strcasecmp(argv[1], "clearhops") == 0) {
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
    	if (debug_) cerr << LOGPREFIX << "Packet with more hops reached a source" << endl;
    	drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
    	return;
    }
    
    ch->ptype() = PT_UWCBR;
    UwCbrModule::recv(p);
}

void UwCbrMultihopSource::initPkt(Packet *p) {
    UwCbrModule::initPkt(p);
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    ch->ptype() = PT_UWCBR_MH;
    hdr_uwcbr_mh_assign_path(mhh, forward_path.begin(), forward_path.end());

    if (debug_) {
	hdr_uwip *iph = HDR_UWIP(p);
	hdr_uwudp *udph = HDR_UWUDP(p);
	cerr << LOGPREFIX << "dest ip " << (int) iph->daddr() <<
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
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);    
    if (ch->ptype() != PT_UWCBR_MH) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        incrPktInvalid();
        return;
    }
    if (mhh->forward_path_begin() + 1 != mhh->forward_path_end()) {
    	if (debug_) cerr << LOGPREFIX << "Packet with more hops reached a sink" << endl;
    	drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
    	return;
    }

    hdr_uwip *iph = HDR_UWIP(p);
    hdr_uwudp *udph = HDR_UWUDP(p);
    if (debug_) cerr << LOGPREFIX << "Received a packet from " <<
		    (int) iph->saddr() << " port " << (int) udph->sport() << endl;
    hdr_uwcbr_mh_update_path(mhh, iph->saddr(), udph->sport());
    
    ch->ptype() = PT_UWCBR;
    UwCbrModule::recv(p);
}

void UwCbrMultihopSink::initPkt(Packet *p) {
    throw logic_error("The sink can't send packets");
}

void UwCbrMultihopSink::initAck(Packet *p, Packet *recvd) {
    UwCbrModule::initAck(p, recvd);
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);
    hdr_uwcbr_mh *recvd_mhh = HDR_UWCBR_MH(recvd);
    ch->ptype() = PT_UWCBR_MH;

    uwcbr_mh_addr *begin = recvd_mhh->path();
    uwcbr_mh_addr *end = recvd_mhh->path() + recvd_mhh->forward_path_end();
    reverse_iterator<uwcbr_mh_addr*> rbegin(end);
    reverse_iterator<uwcbr_mh_addr*> rend(begin);
    hdr_uwcbr_mh_assign_path(mhh, rbegin, rend);
}

UwCbrMultihopRelay::UwCbrMultihopRelay() : debug_(0) {
    bind("debug_", &debug_);
}

UwCbrMultihopRelay::~UwCbrMultihopRelay() {}

void UwCbrMultihopRelay::recv(Packet *p) {
    cerr << "pkt" << endl;
    hdr_cmn *ch = HDR_CMN(p);
    hdr_uwcbr_mh *mhh = HDR_UWCBR_MH(p);    
    if (ch->ptype() != PT_UWCBR_MH) {
        drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
        //incrPktInvalid();
        return;
    }
    if (mhh->forward_path_begin() + 1 == mhh->forward_path_end()) {
    	if (debug_) cerr << LOGPREFIX << "Packet with no more hops" << endl;
    	drop(p, 1, UWCBR_DROP_REASON_UNKNOWN_TYPE);
    	return;
    }

    hdr_uwip *iph = HDR_UWIP(p);
    hdr_uwudp *udph = HDR_UWUDP(p);
    if (debug_) cerr << LOGPREFIX << "Received a packet from " <<
		    (int) iph->saddr() << " port " << (int) udph->sport() << endl;
    hdr_uwcbr_mh_update_path(mhh, iph->saddr(), udph->sport());

    uwcbr_mh_addr next_hop = mhh->path()[mhh->forward_path_begin()];
    iph->daddr() = next_hop.ipaddr;
    udph->dport() = next_hop.port;
    iph->saddr() = 0;
    udph->sport() = 0;

    if (debug_) cerr << LOGPREFIX << "Forward packet to " << (int) iph->daddr() <<
		    " port " << (int) udph->dport() << endl;
    double delay = 0;
    sendDown(p, delay);
}
