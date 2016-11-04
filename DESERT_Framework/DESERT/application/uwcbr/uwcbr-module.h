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
#include "uwcbr-stats.h"
#include "uwcbr-timers.h"

#include <module.h>
#include <uwip-module.h>
#include <uwudp-module.h>

#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#define UWCBR_DROP_REASON_UNKNOWN_TYPE "UKT"
#define UWCBR_DROP_REASON_OUT_OF_SEQUENCE "OOS"
#define UWCBR_DROP_REASON_DUPLICATED_PACKET "DPK"
#define UWCBR_OLD_ACK "OAK"
#define UWCBR_ACK_EMPTY "EAK"
#define UWCBR_DUPACK "DAK"

/**
 * UwCbrModule class is used to manage <i>UWCBR</i> packets
 */
class UwCbrModule : public Module {
public:
    /**
     * Return the size in bytes of a <i>hdr_uwcbr</i> packet header.
     */
    static int getCbrHeaderSize() { return sizeof(hdr_uwcbr); }

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
     * Triggers the generation of a new packet with default priority.
     *
     * \see UwSendTimer::expire()
     * \see UwCbrModule::transmit(char priority)
     */
    virtual void transmit();

    /**
     * Triggers the generation of a new packet with a specific
     * priority.

     * \param priority The priority to set on the packet
     *
     * \see UwSendTimer::expire()
     */
    virtual void transmit(char priority);

    /**
     * Triggers the retranmission of the first unACKed packet.
     *
     * \param timeout Set when the retransmission is due to a retx timeout
     * \see UwRetxTimer::expire()
     */
    virtual void retransmit_first(bool timeout);

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
     * Tell if this CBR module has been stopped
     */
    virtual bool stopped();

    /**
     * Returns the amount of time to wait before the next
     * transmission.
     *
     * @return double Value to use as delay for the next transmission.
     * @see PoissonTraffic_
     */
    virtual double getTimeBeforeNextPkt();

    /**
     * Return the amount of time to wait before a retransmission.
     */
    virtual double getRetxTimeout();

    /**
     * TCL command interpreter.
     *
     * @param argc Number of arguments in <i>argv</i>.
     * @param argv Array of strings which are the command parameters (Note that <i>argv[0]</i> is the name of the object).
     * @return TCL_OK or TCL_ERROR whether the command has been dispatched successfully or not.
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

    virtual int GetSentPkts() const;

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
    virtual void printIdsPkts() const {
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

    typedef std::map<sn_t, Packet*> queue_t; /// Type of the send/recv queues
    queue_t send_queue; /// Hold the packets that have not been ACKed
    queue_t recv_queue; /// Hold the packets that have not been passed up

    int tx_window; /// Size of the transmitter window
    int rx_window; /// Size of the receiver window

    sn_t next_ack; /// Sequence number of the next packet to be ACKed
    sn_t next_tx; /// SN of the next packet to be transmitted for the first time
    sn_t next_sn; /// Sequence number of the next packet to be generated
    sn_t next_recv; /// SN of the next packet to be received

    int use_arq; /// Flag to enable the ARQ
    int dupack_count; /// Hold the number of consecutive dupACKs received
    int dupack_thresh; /// Hold the maximum number of dupACKs before a retx
    bool dupack_backoff; /// True when the dupACKs must be ignored because the pkt was already retxd
    int use_rtt_timeout; /// Use the estimated RTT as the retx timeout
    double timeout_; /// Default timeout for the packet retransmission

    int PoissonTraffic_;        /**< <i>1</i> if the traffic is generated according to a poissonian distribution, <i>0</i> otherwise. */
    double period_;             /**< Period between two consecutive packet transmissions. */
    int pktSize_;               /**< <i>UWCBR</i> packets payload size. */

    int drop_out_of_order_;     /**< Flag to enable or disable the check for out of order packets. */

    UwSendTimer sendTmr_;       /**< Timer which schedules packet transmissions. */
    UwRetxTimer retxTimer;      /**< Timer that schedules the retransmissions */

    int debug_;                 /**< Flag to enable several levels of debug. */

    /**
     * Send packets from the send_queue until there are at most
     * tx_window packets in flight.
     */
    void send_from_queue();

    /**
     * Return the number of packets that can be sent from the
     * send_queue.
     */
    int sendable_count();

    /**
     * Initialize a data packet with a non-default priority
     *
     * \param p Pointer to an already allocated packet
     * \param priority The priority to set on the packet
     */
    virtual void initPkt(Packet *p, char priority);

    /**
     * Set the uid and the timing-related headers on a packet.
     *
     * This is called just before a sendDown() to have distinct uids
     * and to exclude the queueing and retx delay from the FTT/RTT
     * estimation.
     */
    void set_uid_and_times(Packet *p);

    /**
     * Handle a received ACK packet.
     */
    virtual void recvAck(Packet *p);

    /**
     * Handle a received data (non-ACK) packet.
     */
    virtual void recvData(Packet *p);

    /** Initialize an ACK packet.
     *
     * @param p Pointer to the new, already allocated, packet.
     * @param recvd Pointer to the received packet that will be ACKed.
     */
    virtual void initAck(Packet *p, Packet *recvd);

    /**
     * Process the packets in the recv_queue, ordered by SN, until
     * there is a missing packet.
     */
    void pass_up_from_queue();

    /**
     * Send an ACK
     * @param recvd The received packet to ACK
     */
    virtual void sendAck(Packet *recvd);

private:
    enum log_level {ERROR=1, WARNING, INFO, DEBUG=10};
    static std::stringstream nullss;
    ostream &log(log_level level) {
        if (level <= debug_) {
            return std::cerr << Scheduler::instance().clock() << " [CBR " <<
                getId() << "]: ";
        }
        else {
            nullss.seekp(0);
            nullss.str("");
            return nullss;
        }
    }
};

#endif // UWCBR_MODULE_H

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 4
// End:
