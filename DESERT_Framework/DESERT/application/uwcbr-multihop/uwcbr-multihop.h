//  -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-

#ifndef UWCBR_MULTIHOP_H
#define UWCBR_MULTIHOP_H

#include "uwcbr-multihop-packet.h"

#include <uwcbr-module.h>
#include <uwip-module.h>
#include <uwudp-module.h>

#include <list>

class UwCbrMultihopSource : public UwCbrModule {
public:
    UwCbrMultihopSource();
    virtual ~UwCbrMultihopSource();

    virtual int command(int argc, const char *const *argv);

    virtual void append_to_path(const nsaddr_t &ipaddr, const uint16_t &port);
    virtual void append_to_path(const uwcbr_mh_addr &addr);
    virtual void clear_path();

    virtual void recv(Packet *p);
    virtual void initPkt(Packet *p);
    virtual void initAck(Packet *p, Packet *recvd);

protected:
    /** The route that the sent packets will follow */
    std::list<uwcbr_mh_addr> forward_path;
};

class UwCbrMultihopSink : public UwCbrModule {
public:
    UwCbrMultihopSink();
    virtual ~UwCbrMultihopSink();

    //virtual int command(int argc, const char *const *argv);

    virtual void recv(Packet *p);
    virtual void initPkt(Packet *p);
    virtual void initAck(Packet *p, Packet *recvd);
};

struct uwcbrmh_relay_stats {
    int pkts_dup;
    int pkts_invalid;
    int pkts_recv;
    int pkts_retx_dupack;

    int acks_dup;
    int acks_forw;
    int acks_invalid;

    uwcbrmh_relay_stats() {
        reset_except_total();
    }

private:
    void reset_except_total() {
        pkts_dup = 0;
        pkts_invalid = 0;
        pkts_recv = 0;
        pkts_retx_dupack = 0;

        acks_dup = 0;
        acks_forw = 0;
        acks_invalid = 0;
    }
};

class UwCbrMultihopRelay : public Module {
public:
    UwCbrMultihopRelay();
    virtual ~UwCbrMultihopRelay();

    virtual int command(int argc, const char *const *argv);

    virtual void recv(Packet *p);
    virtual void recvPkt(Packet *p);
    virtual void recvAck(Packet *ack);

protected:
    int debug_;

    /** Hold the packets until they are ACKed by the Sink */
    std::map<sn_t,Packet*> packet_buffer;

    /** Number of consecutive dupACKs seen */
    int dupack_count;

    /** Number of dupACKs before a retx occurs */
    int dupack_thresh;

    /** First unACKed sequence number */
    sn_t first_unacked;

    /** Statistics counters */
    uwcbrmh_relay_stats stats;
};

#endif
