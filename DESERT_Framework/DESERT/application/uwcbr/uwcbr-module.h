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
 * @file   uwcbr-module.h
 * @author Giovanni Toso
 * @version 1.1.0
 *
 * \brief Provides the <i>UWCBR</i> packets header description and the definition of the class <i>UWCBR</i>.
 *
 * Provides the <i>UWCBR</i> packets header description and the definition of the class <i>UWCBR</i>.
 * <i>UWCBR</i> can manage no more than 2^16 packets. If a module generates more
 * than 2^16 packets, they will be dropped.
 */

#ifndef UWCBR_MODULE_H
#define UWCBR_MODULE_H

#include "uwcbr-packet.h"
#include "uwcbr-timers.h"
#include "uwcbr-stats.h"

#include <module.h>
#include <uwip-module.h>
#include <uwudp-module.h>

#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#define UWCBR_DROP_REASON_UNKNOWN_TYPE "UKT"      /**< Reason for a drop in a <i>UWCBR</i> module. */
#define UWCBR_DROP_REASON_OUT_OF_SEQUENCE "OOS"   /**< Reason for a drop in a <i>UWCBR</i> module. */
#define UWCBR_DROP_REASON_DUPLICATED_PACKET "DPK" /**< Reason for a drop in a <i>UWCBR</i> module. */
#define UWCBR_DROP_REASON_DUPACK "DUPACK"
#define UWCBR_DROP_REASON_UNKNOWN_ACK "UKACK"
#define UWCBR_INVALID_ACK "INVACK"

class uwcbr_sn_greater {
public:
    bool operator()(Packet const *const &a, Packet const *const &b) const {
        assert(a != 0);
        assert(b != 0);
        hdr_uwcbr *hdr_a = HDR_UWCBR(a);
        hdr_uwcbr *hdr_b = HDR_UWCBR(b);
        return hdr_a->sn() > hdr_b->sn();
    }
};

/**
 * UwCbrModule class is used to manage <i>UWCBR</i> packets
 */
class UwCbrModule : public Module {
public:
    /**
     * Constructor of UwCbrModule class.
     */
    UwCbrModule();

    /**
     * Destructor of UwCbrModule class.
     */
    virtual ~UwCbrModule();

    /**
     * Performs the reception of packets from upper and lower layers.
     *
     * @param Packet* Pointer to the packet will be received.
     */
    virtual void recv(Packet*);

    /**
     * Triggers the generation of a new packet.
     *
     * \see UwCbrModule::sendPkt()
     * \see UwSendTimer::expire()
     */
    virtual void transmit();

    /**
     * Triggers the retranmission of the first unACKed packet.
     *
     * \see UwCbrModule::resendPkt()
     * \see UwRetxTimer::expire()
     */
    virtual void retransmit_first();

    /**
     * Start the generation of packets.
     */
    virtual void start();

    /**
     * Stop the generation of packets.
     *
     * This will also prevent packet retransmissions and hold any
     * packets left in the send queue.
     */
    virtual void stop();
    
    /**
     * TCL command interpreter. It implements the following OTcl methods:
     *
     * @param argc Number of arguments in <i>argv</i>.
     * @param argv Array of strings which are the command parameters (Note that <i>argv[0]</i> is the name of the object).
     * @return TCL_OK or TCL_ERROR whether the command has been dispatched successfully or not.
     *
     */
    virtual int command(int argc, const char*const* argv);

    /**
     * Returns the mean Round Trip Time.
     *
     * @return Round Trip Time.
     */
    virtual double GetRTT() const;

    /**
     * Returns the mean Forward Trip Time.
     *
     * @return Forward Trip Time.
     */
    virtual double GetFTT() const;

    /**
     * Returns the mean Packet Error Rate.
     *
     * @return Packet Error Rate.
     */
    virtual double GetPER() const;

    /**
     * Returns the mean Throughput.
     *
     * @return Throughput.
     */
    virtual double GetTHR() const;

    /**
     * Returns the Round Trip Time Standard Deviation.
     *
     * @return Round Trip Time Standard Deviation.
     */
    virtual double GetRTTstd() const;

    /**
     * Returns the mean Forward Trip Time Standard Deviation.
     *
     * @return Forward Trip Time Standard Deviation.
     */
    virtual double GetFTTstd() const;

    /**
     * Resets all the statistics of the <i>UWCBR</i> module.
     */
    virtual void resetStats();

    /**
     * Prints the IDs of the packet's headers defined by UWCBR.
     */
    inline void printIdsPkts() const {
        std::cout << "UWCBR packets IDs:" << std::endl;
        std::cout << "PT_UWCBR: \t\t" << PT_UWCBR << std::endl;
    }
protected:
    static int uidcnt_;         /**< Unique id of the packet generated. */

    uwcbr_stats stats; /// Statistics counters

    int dstPort_;          /**< Destination port. */
    int dstAddr_;          /**< IP of the destination. */
    int priority_;             /**< Priority of the data packets. */

    nsaddr_t peer_addr; /// First source address seen
    uint16_t peer_port; /// First source port seen

    /** DEPRECATED Used to keep track of the packets already received. */
    std::vector<bool> sn_check;
    /** DEPRECATED Used to keep track of which packets have been ACKed */
    std::vector<bool> ack_check;

    
    /** Hold the packets that have not been ACKed yet, indexed by sn */
    std::map<sn_t, Packet*> packet_buffer;


    /** Type of the tx/rx packet queues,
     *  hold packets in a heap with the minimum SN on top
     */
    typedef std::priority_queue<Packet*,
                                std::vector<Packet*>,
                                uwcbr_sn_greater> pkt_queue_t;
    /** Hold the received packets until they can be processed in order */
    pkt_queue_t recv_queue;
    /** Hold the packets that exceed the transmission window */
    pkt_queue_t send_queue;


    
    /** Hold the number of consecutive dupACKs received */
    int dupack_count;
    /** Hold the maximum number of dupACKs before a retx */
    int dupack_thresh;

    int PoissonTraffic_;        /**< <i>1</i> if the traffic is generated according to a poissonian distribution, <i>0</i> otherwise. */
    double period_;             /**< Period between two consecutive packet transmissions. */
    int pktSize_;               /**< <i>UWCBR</i> packets payload size. */
    int debug_;                 /**< Flag to enable several levels of debug. */
    int drop_out_of_order_;     /**< Flag to enable or disable the check for out of order packets. */
    /** Enable the use of the estimated RTT as the retx timeout */
    int use_rtt_timeout;
    /** Timeout for the packet retransmission */
    double timeout_;

    UwSendTimer sendTmr_;       /**< Timer which schedules packet transmissions. */
    UwRetxTimer retxTimer;      /**< Timer that schedules the retransmissions */

    bool stopped;               /**< Flag to stop sending queued packets when the tx window slides foward */
    int use_arq;                /**< Flag to enable the ARQ */

    sn_t txsn;                  /**< Sequence number of the next packet to be transmitted. */
    sn_t ack_sn;                /**< Sequence number of the next packet to be ACKed */
    int tx_window;             /**< Size of the transmitter window */

    sn_t hrsn;                  /**< Highest received sequence number. */
    int rx_window;             /**< Size of the receiver window */
    sn_t esn;               /**< Expected serial number. */

    /**
     * Initializes a data packet passed as argument with the default values.
     *
     * @param Packet* Pointer to a packet already allocated to fill
     * with the right values.
     */
    virtual void initPkt(Packet* p);

    /**
     * Initialize a data packet with a non-default priority
     *
     * \param p Pointer to an already allocated packet
     * \param priority The priority to set on the packet
     */
    virtual void initPkt(Packet *p, char priority);

    virtual void newPkt(Packet *p);
    
    /** \brief Initialize an ACK packet.
     * @param p Pointer to the new, already allocated, packet.
     * @param recvd Pointer to the received packet that will be ACKed.
     */
    virtual void initAck(Packet *p, Packet *recvd);

    /**
     * Handle a received ACK packet
     */
    virtual void recvAck(Packet *p);

    /**
     * Process the packets in the recv_queue, ordered by SN, until there is a missing packet
     */
    virtual void processOrderedPackets();

    /**
     * Allocates, initialize and put in the send queue a packet with
     * the default priority flag set from tcl.
     *
     * @see UwCbrModule::initPkt()
     */
    virtual void sendPkt();

    /**
     * Set the uid and the timestamps, then send an already
     * constructed packet
     *
     * \param p The packet to send
     * \param delay Delay that the packet suffers when it is passed to
     * the lower layer
     */
    virtual void sendPkt(Packet *p, double delay);

    /**
     * Retransmit the packet with sequence number sn from the packet buffer
     */
    virtual void resendPkt(sn_t sn);


    /**
     * Send an ACK
     * @param recvd The received packet to ACK
     */
    virtual void sendAck(Packet *recvd);

    /**
     * Send packets from the send_queue until the tx window is full
     */
    virtual void slideTxWindow();

    /**
     * Allocates, initialize and sends a packet with the default priority flag set from tcl.
     *
     * @see UwCbrModule::initPkt()
     */
    virtual void sendPktLowPriority();

    /**
     * Allocates, initialize and sends a packet with the default priority flag set from tcl.
     *
     * @see UwCbrModule::initPkt()
     */
    virtual void sendPktHighPriority();

    /**
     * Updates the Round Trip Time.
     *
     * @param double& New Round Trip Time entry.
     */
    virtual void updateRTT(const double&);

    /**
     * Updates the Forward Trip Time.
     *
     * @param double& New Forward Trip Time entry.
     */
    virtual void updateFTT(const double&);

    /**
     * Updates the Throughput.
     *
     * @param int& Bytes of the payload of the last packet received.
     * @param double& Delay Time between the last two receipts.
     */
    virtual void updateThroughput(const int&, const double&);

    /**
     * Increases the number of packets lost.
     *
     * @param int& Number of packets lost.
     */
    virtual void incrPktLost(const int&);

    /**
     * Increases by one the number of received packets.
     */
    virtual void incrPktRecv();

    /**
     * Increases by one the number of out of sequence packets received.
     */
    virtual void incrPktOoseq();

    /**
     * Increases by one the number of invalid packets.
     */
    virtual void incrPktInvalid();

    /**
     * Returns the amount of time to wait before the next transmission. It depends on the PoissonTraffic_ flag.
     *
     * @return double Value to use as delay for the next transmission.
     * @see PoissonTraffic_
     */
    virtual double getTimeBeforeNextPkt();

    inline double getRetxTimeout() {
        if (!use_rtt_timeout) return timeout_;
        double rtt = GetRTT();
        return rtt > 0 ? rtt + 4 * GetRTTstd()  : timeout_;
    }

    inline sn_t max_tx_win_sn() {
        if (use_arq) return ack_sn + ((sn_t)tx_window) - 1;
        else return numeric_limits<sn_t>::max();
    }

    inline sn_t max_rx_win_sn() {
        if (use_arq) return esn + ((sn_t) rx_window) - 1;
        else return numeric_limits<sn_t>::max();
    }

    /**
     * Returns the size in byte of a <i>hdr_uwcbr</i> packet header.
     *
     * @return The size of a <i>hdr_uwcbr</i> packet header.
     */
    static inline int getCbrHeaderSize() { return sizeof(hdr_uwcbr); }
};

#endif // UWCBR_MODULE_H

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
