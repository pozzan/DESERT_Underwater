//
// Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
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

/**
 * @file   msun.h
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Dinamic and Multisink Source Routing Protocol, this file contains Nodes specifications.
 * 
 * Dinamic and Multisink Source Routing Protocol, this file contains Nodes specifications.
 */

#ifndef __MSUN_NODE_H_
#define	__MSUN_NODE_H_

#include "msun-common.h"
#include "msun-hdr-ack.h"
#include "msun-hdr-data.h"
#include "msun-hdr-broadcastdata.h"
#include "msun-hdr-pathest.h"

// Desert headers
#include <uwip-module.h>
#include <uwip-clmsg.h>
#include <uwcbr-module.h>

// Ns2 Headers
#include <mphy.h>
#include <module.h>
#include <packet.h>
#include <tclcl.h>
#include <rng.h>

// C++ headers
#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <limits>
#include <queue>
#include <rng.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class MSun;

/**
 * BufferTimer class is used to handle the timer of the Buffer.
 */
class BufferTimer : public TimerHandler {
    
public:
    BufferTimer(MSun* m) : TimerHandler() {
        module = m;
    }

protected:
    MSun* module; /**< Pointer to an objet of type MSun. */
    
    /**
     * Method invoked when the BufferTimer timer expires.
     */
    virtual void expire(Event *e);
};

/**
 * SearchPathTimer class is used to handle the timer of Search Path requests.
 */
class SearchPathTimer : public TimerHandler {
    
public:
    SearchPathTimer(MSun* m) : TimerHandler() {
        module = m;
    }

protected:
    MSun* module; /**< Pointer to an objet of type MSun. */
    
    /**
     * Method invoked when the SearchPathTimer timer expires.
     */
    virtual void expire(Event *e);
};

/**
 * BroadcastDuplicatePacketsTimer class is used to handle the timer of Broadcast duplicate requests.
 */
class BroadcastDuplicatePacketsTimer : public TimerHandler {
    
public:
    BroadcastDuplicatePacketsTimer(MSun* m) : TimerHandler() {
        module = m;
    }

protected:
    MSun* module; /**< Pointer to an objet of type MSun. */
    
    /**
     * Method invoked when the BroadcastDuplicatePacketsTimer timer expires.
     */
    virtual void expire(Event *e);
};

/**
 * ErrorPacketsTimer class is used to handle the timer of Error Packets.
 */
class ErrorPacketsTimer : public TimerHandler {
    
public:
    ErrorPacketsTimer(MSun* m) : TimerHandler() {
        module = m;
    }

protected:
    MSun* module; /**< Pointer to an objet of type MSun. */
    
    /**
     * Method invoked when the ErrorPacketsTimer timer expires.
     */
    virtual void expire(Event *e);
};

/**
 * MSun class is used to represent the routing layer of a node.
 */
class MSun : public Module {
    
    /**
     * Friend class used to implement the timer on Acks.
     * 
     * @see SinkProbeTimer
     */
    friend class AckWaiting;
    
    /**
     * Friend class used to implement the timer on the Buffer.
     * 
     * @see BufferTimer
     */
    friend class BufferTimer;
    
    /**
     * Friend class used to implement the timer on the Search Path mechanism.
     * 
     * @see SearchPathTimer
     */
    friend class SearchPathTimer;
    
    /**
     * Friend class used to implement the timer on Broadcast packets when the mechanism duplicate them for each destination and does not have any sink available.
     * 
     * @see BroadcastDuplicatePacketsTimer
     */
    friend class BroadcastDuplicatePacketsTimer;
    
    /**
     * Friend class used to implement the timer on Error packets mechanism.
     * 
     * @see ErrorPacketTimer
     */
    friend class ErrorPacketsTimer;
   
public:
    /**
     * Constructor of MSun class.
     */
    MSun();
    
    /**
     * Destructor of MSun class.
     */
    virtual ~MSun();
    
protected:
    /*****************************
     |      Structs              |
     *****************************/

    /**
     * buffer_element describes an entry in the buffer used by <i>MSUN</i>.
     */
    struct buffer_element {
        Packet* p_;          /**< Pointer to the packet buffered. */
        uint32_t id_pkt_;    /**< ID of the packet buffered. */
        double t_reception_; /**< Time instant in which the packet was buffered. */
        double t_last_tx_;   /**< Time instant of the last transmission attempt. */
        int num_retx_;       /**< Number of retransmission tentatives. */
        int destination_;    /**< Define the destination type (UNICAST, BROADCAST, ANYCAST, UNKNOWN). */
        uint8_t source_;     /**< Id of the node that generated the packet. */
        bool valid_packet_;  /**< Flag to set the validity of the path in the header of the data packet. */

        buffer_element(Packet* _p, uint32_t _id_pkt_ = 0, double _t_reception_ = Scheduler::instance().clock(), double _t_last_tx_ = 0, int _num_retx_ = 0, int _destination_ = msun::MSUN_UNICAST, uint8_t _source_ = 0, bool _valid_packet_ = true) : p_(_p), id_pkt_(_id_pkt_), t_reception_(_t_reception_), t_last_tx_(_t_last_tx_), num_retx_(_num_retx_), destination_(_destination_), source_(_source_), valid_packet_(_valid_packet_) {
        } /**< Constructor for buffer_element. */
    };

    struct route_entry {
        double timestamp_;
        double quality_;
        std::vector< uint8_t > list_hops_;
    };

    struct route_container {
        route_entry hop_count_;
        route_entry snr_;
    };
    
    struct pathest_reply_element {
        double time_last_path_search_;      /**< Time instant of the last request replied. */
        double quality_last_path_search_;   /**< Quality of the last request replied. */
    };                                  /**< Element of the table used by sinks to manage the requests and replies to Path Establishment packets. */
    
    /*****************************
     |     Internal Functions    |
     *****************************/
    /**
     * TCL command interpreter. It implements the following OTcl methods:
     * 
     * @param argc Number of arguments in <i>argv</i>.
     * @param argv Array of strings which are the command parameters (Note that <i>argv[0]</i> is the name of the object).
     * @return TCL_OK or TCL_ERROR whether the command has been dispatched successfully or not.
     * 
     */
    virtual int command(int, const char*const*);
    
    /**
     * Performs the reception of packets from upper and lower layers.
     * 
     * @param Packet* Pointer to the packet will be received.
     */
    virtual void recv(Packet*);
    
    /**
     * Cross-Layer messages synchronous interpreter.
     * 
     * @param ClMessage* an instance of ClMessage that represent the message received
     * @return <i>0</i> if successful.
     */
    virtual int recvSyncClMsg(ClMessage*);
    
    /**
     * Cross-Layer messages asynchronous interpreter. Used to retrive the IP
     * od the current node from the IP module.
     * 
     * @param ClMessage* an instance of ClMessage that represent the message received and used for the answer.
     * @return <i>0</i> if successful.
     */
    virtual int recvAsyncClMsg(ClMessage*);
    
    /**
     * Initializes a MSun node.
     * It sends to the lower layers a Sync message asking for the IP of the node.
     * 
     * @see UWIPClMsgReqAddr(int src)
     * @see sendSyncClMsgDown(ClMessage* m)
     */
    virtual void initialize();
    
    /**
     * Clears all the route information of the current node.
     */
    virtual void clearRoutingTable();
    
    /**
     * Resets the data buffer of the node that contains packet generated by the node itself.
     */
    virtual void clearBuffer();
    
    /**
     * Resets the data buffer of the node that contains packet to forward.
     */
    virtual void clearBufferForwarded();
    
    /**
     * Print the hop count of the current node.
     */
    virtual void printLowestHopCount() const;
    
    /**
     * Prints in the stdout the routing table of the current node.
     */
    virtual void printRoutingTable() const;
    
    /**
     * Returns the number of packets stored in the buffer of the node.
     */
    virtual uint32_t getBuffersSize() const;
    
    /**
     * Returns the mean number of retransmission for each packet sent by the node.
     */
    virtual double getMeanRetx() const;
    
    /**
     * Returns a string containing the number of packet sent to a specific destination for each hop count value.
     * 
     * @param Address of the destination.
     * @return string with the values separated by a <i>tab<i>
     */
    virtual string getStatsTx(const uint8_t&) const;
    
    /**
     * Returns a string containing the number of packet received from a specific source for each hop count value.
     * 
     * @param Address of the source.
     * @return string with the values separated by a <i>tab<i>
     */
    virtual string getStatsRx(const uint8_t&) const;
    
    /**
     * Returns a string with an IP in the classic form "x.x.x.x" converting an ns2 nsaddr_t address.
     * 
     * @param nsaddr_t& ns2 address
     * @return String that contains a printable IP in the classic form "x.x.x.x"
     */
    virtual string printIP(const nsaddr_t&) const;
    
    /**
     * Returns a string with an IP in the classic form "x.x.x.x" converting an ns2 nsaddr_t address.
     * 
     * @param nsaddr_t& ns2 address
     * @return String that contains a printable IP in the classic form "x.x.x.x"
     */
    virtual string printIP(const uint8_t&) const;
    
    /**
     * Returns a string with an IP in the classic form "x.x.x.x" converting an ns2 ns_addr_t address.
     * 
     * @param ns_addr_t& ns2 address
     * @return String that contains a printable IP in the classic form "x.x.x.x"
     */
    virtual string printIP(const ns_addr_t&) const;
    
    /**
     * Returns the index of the current node searching for it in the header of a Packet. 0 if not found.
     * 
     * @param Packet in which the method has to search.
     * @return the index of the current node in the hop list of the header. 0 if not found.
     */
    virtual int getIndexInPacket(const Packet*) const;
    
    /**
     * Returns the value to add to the number of retransmission of the packet.
     * 
     * @param Packet in which the method has to search.
     * @return number of extra retransmission.
     */
    virtual int getPlusNumRetx(const Packet*) const;
    
    /**
     * Prints the IDs of the packet's headers defined by Msun.
     */
    virtual void printIdsPkts() const;
    
    /**
     * Returns a delay value to use in transmission. The delay follow a
     * uniform distribution.
     * 
     * @param period
     * @return Transmission delay.
     */
    inline double getDelay(const double& _period) const {
        if (!this->isZero(delay_data_)) {
	    return (_period * RNG::defaultrng()->uniform_double()); 
        } else {
            return 0;
        }
    }
    
    /**
     * Returns the SNR of a packet received.
     * @param _p Packet
     * @return SNR
     */
    inline double getSnr(Packet* _p) const {
        hdr_MPhy* ph = HDR_MPHY(_p);
        if (this->isZero(ph->Pn)) {
            return msun::MIN_SNR;
        } else {
            return 10*log10(ph->Pr/ph->Pn);
        }
    }
    
    /**
     * Returns the SINR of a packet received.
     * @param _p Packet
     * @return SINR
     */
    inline double getSinr(Packet* _p) const {
        hdr_MPhy* ph = HDR_MPHY(_p);
        if (this->isZero(ph->Pi + ph->Pn)) {
            return msun::MIN_SNR;
        } else {
            return 10*log10(ph->Pr / (ph->Pi + ph->Pn));
        }
    }
    
    /**
     * Evaluates is the number passed as input is equal to zero. When C++ works with
     * double and float number you can't compare them with 0. If the absolute
     * value of the number is smaller than eplison that means that the number is
     * equal to zero.
     * 
     * @param double& Number to evaluate.
     * @return <i>true</i> if the number passed in input is equal to zero, <i>false</i> otherwise.
     * @see std::numeric_limits<double>::epsilon()
     */
    inline bool isZero(const double& value) const {
        return std::fabs(value) < std::numeric_limits<double>::epsilon();
    }
    
    /*****************************
     | Path Establishment Search |
     *****************************/
    /**
     * Sends a Path Establishment Packet with the option field sets to Search.
     * It also remove all the information about current routes to the sink.
     * 
     * @param Address to search for, the default value is UWIP_BROADCAST.
     * @see MSun::clearHops()
     * @see MSun::setNumberOfHopToSink(const int&)
     * @see MSun::initPktPathEstSearch(Packet*)
     */
    virtual void searchPath(const uint8_t& _ip = UWIP_BROADCAST);
    
    /**
     * Initializes a Path Establishment Search packet (previously allocated).
     * 
     * @param Packet* Pointer to a Path Establishment Packet with the option field set to Search to initialize.
     * @param Addres to set as destination, the deafult value is UWIP_BROADCAST.
     */
    virtual void initPktPathEstSearch(Packet*, uint8_t _ip = UWIP_BROADCAST) const;
    
    /**
     * Forward to Path Establishment Search packets.
     * It adds in the packet the IP of the current node and forwards or replies to the request.
     * 
     * @param Packet* Pointer to a Path Establishment Search packets to process.
     * @see MSun::isMyIpInList(const Packet*)
     * @see MSun::addMyIpInList(Packet*)
     * @see MSun::updateQuality(Packet*)
     * @see MSun::getNumberOfHopToSink()
     * @see MSun::answerPath(const Packet*)
     */
    virtual void forwardPathEstSearch(Packet*);
    
    /**
     * Adds the IP of the current node in the header of a Path Establishment
     * packet passed as argument. It can do it if there is at least one free block.
     * The function returns <i>true</i> if it added the IP, <i>false</i> otherwise.
     * 
     * @param Packet* Pointer to a Path Establishment Search packet in which to add an IP.
     * @return <i>true</i> if the IP was added, <i>false</i> otherwise.
     */
    virtual bool addMyIpInList(Packet*) const;
    
    /**
     * Checks if the IP of the current node is in the header of the packet
     * passed as argument. If yes it returns <i>true</i>, otherwise it return <i>false</i>.
     * 
     * @param Packet* Pointer to a Path Establishment packet to analyze.
     * @return <i>true</i> if the IP of the current node is in the header, otherwise <i>false</i>.
     */
    virtual bool isMyIpInList(const Packet*) const;
    
    /**
     * Checks if the IP of the current node is in the header of a data packet
     * passed as argument. If yes it returns <i>true</i>, otherwise it return <i>false</i>.
     * 
     * @param Packet* Pointer to a Data packet to analyze.
     * @return <i>true</i> if the IP of the current node is in the header, otherwise <i>false</i>.
     */
    virtual bool isMyIpInListData(const Packet*) const;
    
    /**
     * Updates the field quality in the packet passed as parameter.
     * The value written in the packet depends on the metric field.
     * 
     * @param Packet* Pointer to a packet in which to update the quality field.
     */
    virtual void updateQuality(Packet*) const;
    
    /*****************************
     | Path Establishment Answer |
     *****************************/
    /**
     * Creates and sends an Path Establishment Answer packet.
     * 
     * @param Packet* Pointer to a Path Establishment packet with the option field set to Search.
     * @see MSun::getNumberOfHopToSink()
     * @see MSun::initPktPathEstAnswer(Packet*, const Packet*)
     */
    virtual void answerPath(const Packet* const); //invoked only by a node with ho count = 1
    
    /**
     * Initializes a Path Establishment Answer packet (previously allocated)
     * retrieving the information from a Path Establishment Request packet.
     * 
     * @param Packet* Pointer to a Path Establishment packet with the option field set to Answer to initialize.
     * @param Packet* Pointer to a Path Establishment packet with the option field set to Search.
     */
    virtual void initPktPathEstAnswer(Packet*, const Packet* const); //invoked only by a node with ho count = 1
    
    /**
     * Forwards a Path Establishment Answer Packet. Adds the information about
     * the route in the routing table of the current node.
     * 
     * @param Packet* Pointer to a Path Establishment packet with option field set to ANSWER to forward.
     * @see MSun::clearHops()
     * @see MSun::setNumberOfHopToSink(const int&)
     */
    virtual void forwardPathEstAnswer(Packet*);
    
    /**
     * Equivalent to evaluatePath but used by forwarding nodes.
     * 
     * @param Packet* to evaluate.
     */
    virtual void evaluateForwardedPathEstAnswer(const Packet*);
    
    /**
     * Evaluates the route information contained in a Path Establishment packet,
     * and according to different metrics it evaluates if the path contained in
     * the packet is the new best route.
     * 
     * @param Packet* Pointer to a Path Establishment packet to evaluate.
     * @return Number of hops that separate the current node to the sink.
     * @see MSun::clearHops()
     * @see MSun::setNumberOfHopToSink(const int&)
     * @see MSun::getNumberOfHopToSink()
     */
    virtual int evaluatePath(const Packet*);
    
    /*****************************
     |           Data            |
     *****************************/
    /**
     * Forwards a data packet to the next hop. All the information to route
     * the packet are contained in the packet. If the current node is in the coverage
     * are of the sink it will forward the packet directly to the sink, otherwise
     * the packet will be forwarded to the next hop.
     * 
     * @param Packet* Pointer to a Data packet to forward.
     */
    virtual void forwardDataPacket(Packet*);

    /*****************************
     |           Acks            |
     *****************************/
    /**
     * Creates an ack packet and sends it to the previous hop using the information
     * contained in the header of the data packet passed as input parameter. It is
     * an ack to the previous hop, and not to the source of the packet.
     * 
     * @param Packet* Pointer to a Data packet to acknowledge.
     * @see MSun::initPktAck()
     * @see MSun::clearHops()
     * @see MSun::setNumberOfHopToSink(const int&)
     */
    virtual void sendBackAck(const Packet*);
    
    /**
     * Initializes an ack packet passed as argument with the default values.
     * 
     * @param Packet* Pointer to a packet already allocated to fill with the right values.
     * @see MSun::initPktAck(Packet*)
     */
    virtual void initPktAck(Packet*);
        
    /**
     * Process an ack packet received from a node and update the buffer with the information contained.
     * 
     * @param Ack packet to process.
     */
    virtual void processAck(Packet*);
    
    /*****************************
     |           Errors           |
     *****************************/
    /**
     * Used to create a route error packet. This packet will be
     * sent by the current node to the source of the data packet that generated the
     * error.
     * 
     * @param Packet* Pointer to a Data packet that the current node is unable to forward.
     * @param Packet* Pointer to a Route error packet already allocated to initialize with proper values.
     * @see MSun::initPktPathEstSearch(Packet*)
     */
    virtual void createRouteError(const Packet*, Packet*);
    
    /**
     * Send back an error packet to the previous hop. It uses the information
     * contained in the header of the packet.
     * 
     * @param Packet* Pointer to a Packet to forward to the next hop.
     */
    virtual void sendRouteErrorBack(Packet*);
    
    /**
     * Remove an entry from the routing table after the reception of a Path Error.
     * 
     * @param Packet* to analize.
     */
    virtual void updateRoutingTableAfterError(const Packet*);
    
    /*****************************
     |         Buffering         |
     *****************************/
    /**
     * Manage the buffer of the data packets.
     */
    virtual void bufferManager();
    
    /*****************************
     |        Statistics         |
     *****************************/
    /**
     * Returns the size in byte of a <i>hdr_msun_ack</i> packet header.
     * 
     * @return The size of a <i>hdr_msun_ack</i> packet header.
     */
    static inline int getAckHeaderSize() { return sizeof(hdr_msun_ack); }
    
    /**
     * Returns the size in byte of a <i>hdr_msun_data</i> packet header.
     * 
     * @return The size of a <i>hdr_msun_data</i> packet header.
     */
    static inline int getDataPktHeaderSize() { return sizeof(hdr_msun_data); }
    
    /**
     * Returns the size in byte of a <i>hdr_msun_path_est</i> packet header.
     * 
     * @return The size of a <i>hdr_msun_path_est</i> packet header.
     */
    static inline int getPathEstHeaderSize() { return sizeof(hdr_msun_path_est); }
    
    /**
     * Returns the number of Path Establishment Search packets transmitted by the current node.
     * 
     * @return Number of Path Establishment Search packets transmitted by the current node.
     */
    inline const uint32_t& getNumPathestSearchTx() const {return num_pathest_search_tx_; }
    
    /**
     * Returns the number of Path Establishment Search packets forwarded by the current node.
     * 
     * @return Number of Path Establishment Search packets forwarded by the current node.
     */
    inline const uint32_t& getNumPathestSearchFw() const {return num_pathest_search_fw_; }
    
    /**
     * Returns the number of Path Establishment Search packets received by the current node.
     * 
     * @return Number of Path Establishment Search packets received by the current node.
     */
    inline const uint32_t& getNumPathestSearchRx() const {return num_pathest_search_rx_; }
    
    /**
     * Returns the number of Path Establishment Search packets received by the current node for which the destination is the current node.
     * 
     * @return Number of Path Establishment Search packets received by the current node for which the destination is the current node.
     */
    inline const uint32_t& getNumPathestSearchMe() const {return num_pathest_search_me_; }
    
    /**
     * Returns the number of Path Establishment Answer packets transmitted by the current node.
     * 
     * @return Number of Path Establishment Answer packets transmitted by the current node.
     */
    inline const uint32_t& getNumPathestAnswerTx() const {return num_pathest_answer_tx_; }
    
    /**
     * Returns the number of Path Establishment Answer packets forwarded by the current node.
     * 
     * @return Number of Path Establishment Answer packets forwarded by the current node.
     */
    inline const uint32_t& getNumPathestAnswerFw() const {return num_pathest_answer_fw_; }
    
    /**
     * Returns the number of Path Establishment Answer packets received by the current node.
     * 
     * @return Number of Path Establishment Answer packets received by the current node.
     */
    inline const uint32_t& getNumPathestAnswerRx() const {return num_pathest_answer_rx_; }
    
    /**
     * Returns the number of Path Establishment Answer packets received by the current node for which the destination is the current node.
     * 
     * @return Number of Path Establishment Answer packets received by the current node for which the destination is the current node.
     */
    inline const uint32_t& getNumPathestAnswerMe() const {return num_pathest_answer_me_; }
    
    /**
     * Returns the number of Data packets transmitted by the current node.
     * 
     * @return Number of Data packets transmitted by the current node.
     */
    inline const uint32_t& getNumDataTx() const {return num_data_tx_; }
    
    /**
     * Returns the number of Data packets forwarded by the current node.
     * 
     * @return Number of Data packets forwarded by the current node.
     */
    inline const uint32_t& getNumDataFw() const {return num_data_fw_; }
    
    /**
     * Returns the number of Data packets received by the current node.
     * 
     * @return Number of Data packets received by the current node.
     */
    inline const uint32_t& getNumDataRx() const {return num_data_rx_; }
    
    /**
     * Returns the number of Data packets received by the current node for which the destination is the current node.
     * 
     * @return Number of Data packets received by the current node for which the destination is the current node.
     */
    inline const uint32_t& getNumDataMe() const {return num_data_me_; }
    
    /**
     * Returns the number of Data packets acked.
     * 
     * @return Number of Data packets acked.
     */
    inline const uint32_t& getNumDataAcked() const {return num_data_acked_; }
    
    /**
     * Returns the number of Data packets stored in the buffer by the current node.
     * 
     * @return Number of Data packets stored in the buffer by the current node.
     */
    inline const uint32_t& getNumDataStored() const {return num_data_stored_; }
    
    /**
     * Returns the number of Ack packets transmitted by the current node.
     * 
     * @return Number of Ack packets transmitted by the current node.
     */
    inline const uint32_t& getNumAckTx() const {return num_ack_tx_; }
    
    /**
     * Returns the number of Ack packets received by the current node.
     * 
     * @return Number of Ack packets received by the current node.
     */
    inline const uint32_t& getNumAckRx() const {return num_ack_rx_; }
    
    /**
     * Returns the number of Ack packets received by the current node for which the destination is the current node.
     * 
     * @return Number of Ack packets received by the current node for which the destination is the current node.
     */
    inline const uint32_t& getNumAckMe() const {return num_ack_me_; }
    
    /**
     * Returns the number of Path Establishment Error packets transmitted by the current node.
     * 
     * @return Number of Path Establishment Error packets transmitted by the current node.
     */
    inline const uint32_t& getNumErrorTx() const {return num_error_tx_; }
    
    /**
     * Returns the number of Path Establishment Error packets forwarded by the current node.
     * 
     * @return Number of Path Establishment Error packets forwarded by the current node.
     */
    inline const uint32_t& getNumErrorFw() const {return num_error_fw_; }
    
    /**
     * Returns the number of Path Establishment Error packets received by the current node.
     * 
     * @return Number of Path Establishment Error packets received by the current node.
     */
    inline const uint32_t& getNumErrorRx() const {return num_error_rx_; }
    
    /**
     * Returns the number of Path Establishment Error packets received by the current node for which the destination is the current node.
     * 
     * @return Number of Path Establishment Error packets received by the current node for which the destination is the current node.
     */
    inline const uint32_t& getNumErrorMe() const {return num_error_me_; }
    
    /**
     * Returns the number of Data packets dropped by the current node because the buffer was full.
     * 
     * @return Number of Data packets dropped by the current node because the buffer was full.
     */
    inline const uint32_t& getNumDropBuffFull() const {return num_drop_buff_full_; }
    
    /**
     * Returns the number of Data packets dropped by the current node because it retransmitted the packet the maximum number of time allowed.
     * 
     * @return Number of Data packets dropped by the current node because it retransmitted the packet the maximum number of time allowed.
     */
    inline const uint32_t& getNumDropMaxRetx() const {return num_drop_maxretx_; }
    
    /*****************************
     |          Timers           |
     *****************************/
    /**
     * This function enables the possibility to send a new Seath Path packet.
     * It is used to avoid too many control packets in network in a high load condition.
     */
    virtual void searchPathExpire();
    
    /**
     * This function gives the possibility to duplicate a broadcast packet for each
     * destination in the routing table after then a Path Establishment packet was sent.
     * Used to wait the right amount of time for the paths.
     */
    virtual void broadcastDuplicatePacketsExpire();
    
    /**
     * This function enables the possibility to send a new Error packet.
     * It is used to avoid too many control packets in network in a high load condition.
     */
    virtual void errorPacketsExpire();
    
    /*****************************
     |         Trace file         |
     *****************************/
    /**
     * Traces a packet. 
     * 
     * @param Packet to be traced.
     * @param String optional for the packet.
     */
    virtual void tracePacket(const Packet* const, const string& position = "UNDEF___");
    
    /**
     * Function that accepts a list of string and create an entry for the trace file.
     */
    virtual const string createTraceString(const string&, const double&, const int&, const int&, const int&, const int&, const int&, const int&, const int&, const int&, const int&) const;
    
    /**
     * Opens the trace file, writes the string passed as input and closes the file.
     * 
     * @param String to write in the trace file.
     */
    virtual void writeInTrace(const string&);
    
    /**
     * Writes in the Path Trace file the path contained in the Packet
     * 
     * @param Packet to analyze.
     */
    virtual void writePathInTrace(const Packet*);
    
    /*****************************
     |        Routing Table       |
     *****************************/
    /**
     * Return the IP of the sink with the Path with the lowest hop count value.
     * 
     * @return IP of the Sink.
     */
    virtual uint8_t ipLowestHopCountPath() const;
    
    /**
     * Return the IP of the sink with the Path with the MaxMin SNR value.
     * 
     * @return IP of the Sink.
     */
    virtual uint8_t ipMaxMinSNRPath() const;

    /**
     * Return the hop count to a specific sink IP, 0 if no path valid to the sink is known.
     * 
     * @param IP of the sink.
     * @return the hop count to the sink, 0 if it does not exist.
     */
    virtual int hcToIp(const uint8_t&) const;
    
    /**
     * Return the maxmin snr of the path to a specific sink IP, -MIN_INT if no path valid to the sink is known.
     * 
     * @param IP of the sink.
     * @return the maxmin snr of the path to the sink, -MIN_INT if it does not exist.
     */
    virtual double snrToIp(const uint8_t&) const;
    
    /**
     * Return true if the specific element in the routing table is valid by checking the timestamp.
     * 
     * @param the element to check.
     * @return true if the entry is not expired, false otherwise.
     */
    virtual bool testValidTimestamp(const double&) const;
    
    
    /*****************************
     |       Init Data Packet     |
     *****************************/
    /**
     * Used to initialize a data packet send to Anycast, that means with that the destination will be the best sink reacheable.
     * 
     * @param Packet to initialize.
     * @return 0 if there were problems during the initialization, 1 otherwise.
     */
    virtual int initPacketAnycast(Packet*);
    
    /**
     * Used to initialize a data packet send to Unicast, that means with that the destination is a specific IP.
     * 
     * @param Packet to initialize.
     * @return 0 if there were problems during the initialization, 1 otherwise.
     */
    virtual int initPacketUnicast(Packet*);
    
private:    
    // Variables
    uint8_t ipAddr_;                    /**< IP of the current node. */
    int metric_;                        /**< Metric used by the current node. */
    double delay_status_;               /**< Maximum delay added when transmitting control packets. */
    double delay_data_;                 /**< Maximum delay added when transmitting data packets in the buffer. */
    std::map<uint8_t, route_container> routing_table; /**< Data structure containing the routing table of the node. */
    int printDebug_;                    /**< Flag to enable or disable dirrefent levels of debug. */
    bool timer_search_path_enabled_;    /**< Flag to enable or disable the possibility to send Path Search packets. */
    bool timer_error_packets_enabled_;  /**< Flag to enable or disable the possibility to send Path Error packets. */
    int disable_path_error_;            /**< Flag to enable or disable the possibility to send <i>Path Error</i> packets. */
    int disable_route_error_;           /**< Flag to enable or disable errors in the routing table. */
    double min_sinr_path_request_;      /**< If != 0 a Path Establishment request packet is processed only if the SINR of the packet is greater than this value. */
    queue<Packet*> bcast_queue_;        /**< Queue used to store temporary broadcast packets to copy for multiple directions. */
    
    // Acks
    int num_retx_;                      /**< Maximum number of Ack errors tollerated by the node. */
    
    // Broadcast
    int restricted_broadcast_;          /**< Enables a semi-broadcast mode that uses the enhanced mechanism of MSUN. */
    int ttl_;                           /**< Time to leave of the <i>PT_MSUN_BROADCASTDATA</i> packets. */
    double maximum_cache_time_;         /**< Validity time of a packet entry. */
    typedef std::map<uint16_t, double> map_packets;               /**< Typedef for a packet id: (serial_number, timestamp). */
    typedef std::map<uint8_t, map_packets> map_forwarded_packets; /**< Typedef for a map of the packet forwarded (saddr, map_packets). */
    map_forwarded_packets my_forwarded_packets_;                  /**< Map of the packet forwarded. */
    
    // Sink
    int enable_sink_;                   /**< Flag used to set the sink capabilities. It equals to 1 that means that the node acts also as sink. */
    std::map<uint8_t, pathest_reply_element> pathest_reply_table;  /**< Table used to manage replies to Path Establishment requests. */
    
    // Buffer
    std::vector<buffer_element> buffer_data;         /**< Buffer used to store data packets generated by the current node. */
    std::vector<buffer_element> buffer_data_forward; /**< Buffer used to store data packets generated by other nodes. */
    uint32_t buffer_max_size_;          /**< Maximum length of the data buffer. */
    int adaptive_timer_buffer_;         /**< Enables a mechanism used to modify the <i>timer_buffer_</i> in case of the sending time is shorter than the time needed to receive acks. */
    double alpha_data_;                 /**< Correlation used for the dinamic timer of buffer processing. */
    double wait_path_answer_;           /**< Delay to add when a path to a specific node is requested after that a not initialized packet is processed. */
    int enable_tweak_retx_;             /**< Enable a mechanism that give the possibility to the protocol to set dinamically the number of retransmission to forwarded packets. */
    int prevent_from_drop_;             /**< If enabled the source of a packet will never drop a data packet in case of error. The node will search for new paths. Can cause starvation if the destination is always unreachable. */
    double max_buffer_tmr_;           /**< Maximum value for the timer_buffer_ parameter. */
    double min_buffer_tmr_;           /**< Maximum value for the timer_buffer_ parameter. */
    
    // Timers
    double timer_route_validity_;       /**< Maximum validity time for a route entry. */
    double timer_buffer_;               /**< Timer for buffer management. */
    double timer_search_path_;          /**< Timer for the search path mechanism. */
    double timer_answer_path_;          /**< If a sink receives more than one path search request from the same node during this interval, it replies only if the new request contains a best path. */
    double timer_error_packets_;        /**< Timer for error packets. */
    
    BufferTimer bufferTmr_;             /**< BufferTimer object. */
    SearchPathTimer searchPathTmr_;     /**< SearchPathTimer object. */
    BroadcastDuplicatePacketsTimer broadcastDuplicatePacketsTimer_; /** BroadcastDuplicatePacketsTimer object. */
    ErrorPacketsTimer errorPacketsTimer_; /**< ErrorPacketTimer. */
    
    // Trace
    bool trace_;                        /**< Flag used to enable or disable the trace file of the current node, */
    bool trace_path_;                   /**< Flag used to enable or disable the trace path file of the current node, */
    char* trace_file_name_;             /**< Name of the trace file used for the current node. */
    char* trace_file_path_name_;        /**< Name of the trace file used for the current node to keep trace of the paths of the data packets received. */
    ofstream trace_file_;               /**< Ofstream used to write the trace file in the disk. */
    ofstream trace_file_path_;          /**< Ofstream used to write the trace file in the disk. */
    char trace_separator_;              /**< Used as separator among elements in an entr of the tracefile. */
    
    // Statistics
    std::map <uint8_t, std::vector<int> > data_tx_;        /**< Contains, for each destination, the hop count value for each packet sent. */
    std::map <uint8_t, std::vector<int> > data_rx_;        /**< Contains, for each source, the hop count value for each packet received. */
    uint32_t num_pathest_search_tx_;       /**< Nnumber of Path Establishment Search packets transmitted by the node. */
    uint32_t num_pathest_search_fw_;       /**< Number of Path Establishment Search packets forwarded by the node. */
    uint32_t num_pathest_search_rx_;       /**< Number of Path Establishment Search packets received by the node. */
    uint32_t num_pathest_search_me_;       /**< Number of Path Establishment Search packets received by the node and for him. */
    uint32_t num_pathest_answer_tx_;       /**< Number of Path Establishment Answer packets transmitted by the node. */
    uint32_t num_pathest_answer_fw_;       /**< Number of Path Establishment Answer packets forwarded by the node. */
    uint32_t num_pathest_answer_rx_;       /**< Number of Path Establishment Answer packets received by the node. */
    uint32_t num_pathest_answer_me_;       /**< Number of Path Establishment Answer packets received by the node and for him. */
    uint32_t num_data_tx_;                 /**< Number of Data packets transmitted by the node. */
    uint32_t num_data_fw_;                 /**< Number of Data packets forwarded by the node. */
    uint32_t num_data_rx_;                 /**< Number of Data packets received by the node. */
    uint32_t num_data_me_;                 /**< Number of Data packets received by the node and for him. */
    uint32_t num_data_acked_;              /**< Number of Data packets corretly acked. */
    uint32_t num_data_stored_;             /**< Number of Data packets stored in the buffer. */
    uint32_t num_ack_tx_;                  /**< Number of Ack packets transmitted by the node. */
    uint32_t num_ack_rx_;                  /**< Number of Ack packets received by the node. */
    uint32_t num_ack_me_;                  /**< Number of Ack packets received by the node and for him. */
    uint32_t num_error_tx_;                /**< Number of Path Error packets transmitted by the node. */
    uint32_t num_error_fw_;                /**< Number of Path Error packets forwarded by the node. */
    uint32_t num_error_rx_;                /**< Number of Path Error packets received by the node. */
    uint32_t num_error_me_;                /**< Number of Path Error packets received by the node and for him. */
    uint32_t num_drop_buff_full_;          /**< Number of packets dropped by the node, reason: the buffer is full. */
    uint32_t num_drop_maxretx_;            /**< Number of packets dropped by the node, reason: max number of retransmission reached. */
    
    /**
     * Copy constructor declared as private. It is not possible to create a new MSun object passing to its constructor another MSun object. 
     * 
     * @param MSun& MSun object.
     */
    MSun(const MSun&);
};

#endif // __MSUN_NODE_H_
