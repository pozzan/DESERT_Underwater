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
 * @file mdriverFSK_WHOI_MM.h
 * @author Riccardo Masiero, Matteo Petrani
 * \version 1.0.0
 * \brief Header of the class derived from UWMdriver to interface ns2/NS-Miracle with the FSK WHOI Micro-Modems.
 */

#ifndef UWMDRIVERFSKWHOIMM_H
#define UWMDRIVERFSKWHOIMM_H

#include "mserial.h"
#include "minterpreterNMEA.h"
#include <uwmdriver.h>
#include <queue>

// MACROS for the transmission management of NMEA messages
// MESSAGES FLAGS
#define _UBIN 0 /**< Flag to use binary data messages (see MdriverFSK_WHOI_MM::umessage)*/
#define _UMP 1 /**< Flag to use minipacket messages (see MdriverFSK_WHOI_MM::umessage)*/
//TX CONFIGURATION MESSAGES
#define _SETID 1 /**< Status 1 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): set modem ID. */
#define _SETIDS 2 /**< Status 2 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): modem ID sent. */
// TX CYCLE INIT
#define _CINIT 11 /**< Status 11 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): send cycle init. */
#define _CINITS 12 /**< Status 12 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): cycle init request sent to modem. */
#define _CINITR 13 /**< Status 13 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): cycle init request received by the modem. */
#define _CINITST 14 /**< Status 14 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): start tx cycle init. */
#define _CINITET 15 /**< Status 15 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): end tx cycle init. */
#define _CINITSS 16 /**< Status 16 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): statistics of the tx cycle init. */
// TX BINARY DATA
#define _BIN 21 /**< Status 21 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): free to send binary data. */
#define _BINS 22 /**< Status 22 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): binary data sent to modem. */
#define _BINSR 23 /**< Status 23 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): binary data received by the modem. */
#define _BINST 24 /**< Status 24 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): start tx binary data. */
#define _BINET 25 /**< Status 25 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): end tx binary data. */
// TX MINIPACKET
#define _MP 31 /**< Status 31 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): send minipacket. */
#define _MPS 32 /**< Status 32 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): minipacket sent to modem. */
#define _MPR 33 /**< Status 33 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): minipacket received by the modem. */
#define _MPST 34 /**< Status 34 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): start tx minipacket. */
#define _MPET 35 /**< Status 35 of the driver's NMEA-complaint TX state machine (see MdriverFSK_WHOI_MM::m_status_tx): end tx minipacket. */
// RX DATA
#define _RXS -1 /**< Status -1 of the driver's NMEA-complaint RX state machine (see MdriverFSK_WHOI_MM::m_status_rx): acoustic signal detected. */
#define _RXCINIT -2 /**< Status -2 of the driver's NMEA-complaint RX state machine (see MdriverFSK_WHOI_MM::m_status_rx): reception of a cycle-init message. */
#define _RXCINITST -3 /**< Status -2 of the driver's NMEA-complaint RX state machine (see MdriverFSK_WHOI_MM::m_status_rx): reception of a cycle-init message. */
#define _RXBIN -4 /**< Status -3 of the driver's NMEA-complaint RX state machine (see MdriverFSK_WHOI_MM::m_status_rx): reception of a binary data message HEX coded. */
#define _RXMP -5 /**< Status -4 of the driver's NMEA-complaint RX state machine (see MdriverFSK_WHOI_MM::m_status_rx): reception of a minipacket. */
#define _ERR -6 /**< Status -5 of the driver's NMEA-complaint RX state machine (see MdriverFSK_WHOI_MM::m_status_rx): bad crc of packet timeout. */

/**
 * Class containing the basic functions to drive the FSK WHOI micromodem transmissions/receptions (this class is a derived class of UWMdriver).           
 */
class MdriverFSK_WHOI_MM : public UWMdriver {
	MinterpreterNMEA mInterpreter; /** < Object that builds/parses NMEA messages. */
	Mserial mConnector; /** < Object that handles the physical host to modem communications via RS-232 serial cables. */

	int m_status_tx; /**< TX status for the transmission manager methods, see methods MdriverFSK_WHOI_MM::modemTxManager and MdriverFSK_WHOI_MM::updateStatus. */

	bool ongoing_tx; /**< Overall TX status for a further check done by the transmission manager method, see method MdriverFSK_WHOI_MM::modemTxManager. */

	int m_status_rx; /**< RX status for the MdriverFSK_WHOI_MM::updateStatus() method. */

	int umessage; /**< Flag to enable/disable the use of minipacket in place of the binary transmission. */

	queue<std::string> queue_tx; /**< Queue used to buffer incoming strings for tx messages.*/

	queue<std::string> queue_rx; /**< Queue used to buffer incoming strings for rx messages.*/

public:
	/** 
	 * Class constructor.
	 * 
	 * @param pmModem_ pointer to the UWMPhy_modem object to link with this UWMdriver object.
	 */
	MdriverFSK_WHOI_MM(UWMPhy_modem*);

	/**
	 * Class destructor.
	 */
	~MdriverFSK_WHOI_MM();

	/**
	 *  Driver starter. This method starts the driver performing all the needed operations 
	 *  to open an host-modem connection. 
	 */
	virtual void start();

	/**
	 *  Driver stopper. This method should be used before stopping the simulation. It closes and, if needed, 
	 *  resets all the opened files and ports.
	 */
	virtual void stop();

	/**
	 *  Method to notify to the driver that there is a packet to be sent via modem.
	 *  NOTE: when this function is called (by an UWMPhy_modem object), the driver's status must be set to TX_ and the packet must be sent immediately to the modem.
	 */
	virtual void modemTx();

	/** 
	 *  Method to update modem status. This method has to update the modem status according to the  messages 
	 *  received from the modem/channel (e.g., after a check of the modem buffer's output). NOTE: This method may return after an arbitrary period if nothing has happened, but it must return immediately after a change of UWMdriver::status.
	 * 
	 *  @return UWMdriver::status, the updated modem's status.
	 */
	virtual int updateStatus();
        
        virtual void emptyModemQueue();


protected:


	/** 
	 * Method to manage modem to host and host to modem communications. This method has to handle the different transmissions cases and corresponding protocol messages to be generated according to the tcl-user choices and modem firmware, respectively.
	 */
	virtual void modemTxManager();

};
#endif	/* UWMDRIVERFSKWHOIMM_H */
