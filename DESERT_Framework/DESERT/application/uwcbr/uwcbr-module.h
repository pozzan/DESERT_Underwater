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

#include <module.h>
#include <uwip-module.h>
#include <uwudp-module.h>

#include <climits>
#include <iostream>
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

#define HDR_UWCBR(p)      (hdr_uwcbr::access(p))

extern packet_t PT_UWCBR;

typedef uint16_t sn_t; 

/**
 * <i>hdr_uwcbr</i> describes <i>UWCBR</i> packets.
 */
typedef struct hdr_uwcbr {
    sn_t sn_;       /**< Serial number of the packet. */
    float rftt_;        /**< Forward Trip Time of the packet. */
    bool rftt_valid_;   /**< Flag used to set the validity of the fft field. */
    char priority_;     /**< Priority flag: 1 means high priority, 0 normal priority. */
    bool is_ack_;       /**< Flag that indicates if this packet is an ACK */

    static int offset_; /**< Required by the PacketHeaderManager. */

    /**
     * Reference to the offset_ variable.
     */
    inline static int& offset() {
        return offset_;
    }

    inline static struct hdr_uwcbr * access(const Packet * p) {
        return (struct hdr_uwcbr*) p->access(offset_);
    }

    /**
     * Reference to the sn_ variable.
     */
    inline sn_t& sn() {
        return sn_;
    }
    
    /**
     * Reference to the rftt_valid_ variable.
     */
    inline bool& rftt_valid() {
        return rftt_valid_;
    }
    
    /**
     * Reference to the priority_ variable.
     */
    inline char& priority() {
        return priority_;
    }
    
    /**
     * Reference to the rftt_ variable.
     */
    inline float& rftt() {
        return (rftt_);
    }

    inline bool &is_ack() { return is_ack_; }
} hdr_uwcbr;

struct uwcbr_stats {    
    int pkts_last_reset;        /**< Used for error checking after stats are reset. Set to pkts_lost+pkts_recv each time resetStats is called. */
    int acks_last_reset;

    double lrtime;              /**< Time of last packet reception. */

    int acks_dup;                /**< Total number of duplicate ACKs received */
    int acks_invalid;
    int acks_recv;               /**< Total number of correct ACKs received */
    int pkts_dup;               /**< Total number of duplicate packets received */
    int pkts_invalid;           /**< Total number of invalid packets received. */
    int pkts_recv;              /**< Total number of packets received in order */

    int acks_sent;
    int acks_dup_sent;
    
    int pkts_ooseq;             /**< Total number of packets received out of sequence. */
    int pkts_lost;              /**< Total number of lost packets, including packets received out of sequence. */

    //double srtt;                /**< Smoothed Round Trip Time, calculated as for TCP. */
    //double sftt;                /**< Smoothed Forward Trip Time, calculated as srtt. */
    //double sthr;                /**< Smoothed throughput calculation. */
            
    double rftt;                /**< Forward Trip Time seen for last received packet. */
  
    /* Cumulative statistics */
    double sumrtt;              /**< Sum of RTT samples. */
    double sumrtt2;             /**< Sum of (RTT^2). */
    int rttsamples;             /**< Number of RTT samples. */

    double sumftt;              /**< Sum of FTT samples. */
    double sumftt2;             /**< Sum of (FTT^2). */
    int fttsamples;             /**< Number of FTT samples. */

    double sumbytes;            /**< Sum of bytes received. */
    double sumdt;               /**< Sum of the delays. */

    uwcbr_stats() : pkts_last_reset(0), acks_last_reset(0), lrtime(0) {
	reset_no_last();
    }
    
    inline void reset() {
	pkts_last_reset += pkts_recv + pkts_invalid;
	acks_last_reset += acks_recv + acks_dup + acks_invalid;
	reset_no_last();
    }
private:
    inline void reset_no_last() {
	acks_dup = 0;
	acks_invalid = 0;
	acks_recv = 0;
	pkts_dup = 0;
	pkts_invalid = 0;
	pkts_recv = 0;
	
	acks_sent = 0;
	acks_dup_sent = 0;
	
	pkts_ooseq = 0;
	pkts_lost = 0;
	
	//srtt = 0;
	//sftt = 0;    
	//sthr = 0;
	
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
};

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

class UwCbrModule;

/**
 * UwSendTimer class is used to handle the scheduling period of <i>UWCBR</i> packets.
 */
class UwSendTimer : public TimerHandler {
public:

    UwSendTimer(UwCbrModule *m) : TimerHandler() {
        module = m;
    }

protected:
    virtual void expire(Event *e);
    UwCbrModule* module;
};

/** UwRetxTimer is used to schedule the retransmission of packets after a timeout */
class UwRetxTimer : public TimerHandler {
public:
    UwRetxTimer(UwCbrModule *m, sn_t sn) : TimerHandler() {
	module = m;
	packet_sn = sn;
    }

    virtual void resched(double delay);
    virtual void sched(double delay);
    virtual void force_cancel();

protected:
    virtual void expire(Event *e);
    UwCbrModule* module;
    sn_t packet_sn;
};

/**
 * UwCbrModule class is used to manage <i>UWCBR</i> packets and to collect statistics about them.
 */
class UwCbrModule : public Module {
    friend class UwSendTimer;
    friend class UwRetxTimer;
    
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
    uwcbr_stats stats;
    
    static int uidcnt_;         /**< Unique id of the packet generated. */
    int dstPort_;          /**< Destination port. */
    int dstAddr_;          /**< IP of the destination. */
    int priority_;             /**< Priority of the data packets. */

    /** Used to keep track of the packets already received. */
    std::vector<bool> sn_check;
    /** Used to keep track of which packets have been ACKed */
    std::vector<bool> ack_check;
    /** Hold the packets that have not been ACKed yet, indexed by sn */
    std::map<sn_t, Packet*> packet_buffer;
    /** Hold the timers that schedule the retransmissions, indexed by sn */
    std::map<sn_t, UwRetxTimer*> packet_retx_timers;

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

    bool stopped;               /**< Flag to stop sending queued packets when the tx window slides foward */
    
    sn_t txsn;                  /**< Sequence number of the next packet to be transmitted. */
    sn_t ack_sn;                /**< Sequence number of the next packet to be ACKed */
    int tx_window;             /**< Size of the transmitter window */
    
    sn_t hrsn;                  /**< Highest received sequence number. */
    int rx_window;             /**< Size of the receiver window */
    sn_t esn;               /**< Expected serial number. */
        
    /**
     * Initializes a data packet passed as argument with the default values.
     * 
     * @param Packet* Pointer to a packet already allocated to fill with the right values.
     */
    virtual void initPkt(Packet* p);

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
     * Allocates, initialize and sends a packet with the default priority flag set from tcl.
     * 
     * @see UwCbrModule::initPkt()
     */
    virtual void sendPkt();

    /**
     * Send an already constructed packet
     * \param p The packet to send
     * \param delay Delay that the packet suffers when it is passed to the lower layer
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
     * Creates and transmits a packet and schedules a new transmission.
     * 
     * @see UwCbrModule::sendPkt()
     */
    virtual void transmit();
    
    /**
     * Start to send packets.
     */
    virtual void start();
    
    /**
     * Stop to send packets.
     */
    virtual void stop();
    
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
	return rtt > 0 ? rtt + 3 * GetRTTstd()  : timeout_;
    }

    inline sn_t max_tx_win_sn() { return ack_sn + ((sn_t)tx_window) - 1; }
    inline sn_t max_rx_win_sn() { return esn + ((sn_t) rx_window) - 1; }
    
    /**
     * Returns the size in byte of a <i>hdr_uwcbr</i> packet header.
     * 
     * @return The size of a <i>hdr_uwcbr</i> packet header.
     */
    static inline int getCbrHeaderSize() { return sizeof(hdr_uwcbr); }
};

#endif // UWCBR_MODULE_H
