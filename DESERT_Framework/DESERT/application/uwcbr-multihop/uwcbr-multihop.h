//  -*- mode: c++; c-basic-offset: 4; -*-

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
    std::list<uwcbr_mh_addr> forward_path; /**< The route that the sent packets will follow */
};

class UwCbrMultihopSink : public UwCbrModule {
public:
    UwCbrMultihopSink();
    virtual ~UwCbrMultihopSink();
    
    //virtual int command(int argc, const char *const *argv);

    virtual void recv(Packet *p);    
    virtual void initPkt(Packet *p);
    virtual void initAck(Packet *p, Packet *recvd);
    
private:
};

class UwCbrMultihopRelay : public Module {
public:
    UwCbrMultihopRelay();
    virtual ~UwCbrMultihopRelay();

    //virtual int command(int argc, const char *const *argv);
    
    virtual void recv(Packet *p);
protected:
    int debug_;
};

#endif // UWCBR_MODULE_H
