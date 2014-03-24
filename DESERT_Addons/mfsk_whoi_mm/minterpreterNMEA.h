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
 * @file mfsk_whoi_mm/minterpreterNMEA.h
 * @author Riccardo Masiero, Matteo Petrani
 * \version 1.0.0
 * \brief Header of the class that is in charge of building/parsing the necessary messages to make the UWMdriver able to communicate with the modem according to the NMEA standard.
 */


#ifndef MINTERPRETERNMEA_H
#define MINTERPRETERNMEA_H

#include <uwminterpreter.h>

#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <cstring>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cmath>

using namespace std;


/** Class used to build or parse NMEA messages (this class derives UWMinterpreter); currently, this class implements methods 
 * to build/parse:
 *  - simple configuration messages (e.g., setting of the modem ID);
 *  - messages involved in the transmission of a Mini-Packet (see NMEA $CCMUC command);
 *  - messages involved in the transmission of a Binary Packet (see NMEA $CCTXD command)
 * NOTE: for a detailed description of all the messages built and parsed in this class (and of those that can be added),
 * please refer to the Micro-Modem Software Interface guide available at http://acomms.whoi.edu/documents/uModem.
 */
class MinterpreterNMEA : public UWMinterpreter
{
	public:
	  
	/** 
	 * Class constructor.
	 * 
	 * @param pmDriver_  pointer to the UWMdriver object to link with this UWMinterpreter object.
	 */
	MinterpreterNMEA(UWMdriver*);
		
	/**
	 * Class destructor.
	 */
	~MinterpreterNMEA();
	
	// METHODS to BUILD MESSAGES
	
	/** 
	 * Method to build a NMEA $CCCFG message (configuration message).
	 * 
         * @param _param the modem parameter to configure (e.g., SRC for the modem ID. See NMEA manual for more details).
	 * @param _value the value to be assigned at _param (e.g., the ID of the modem for SRC).
	 * @return nmea_string, the requested NMEA message.
	 */
	std::string build_cccfg(std::string _param, int _value);
	
	/** 
	 * Method to build a NMEA $CCCYC message (cycle-init message). 
         * 
	 * @param _src the ID of the modem that must transmit the packet.
	 * @param _dest the ID of the modem that must receive the packet.
	 * @param _packetype type of packet to send, according to the modem hardware 
	 *                    (FSK or different PSK). See the NMEA manual for more details.
	 * @param _nframe number of frames per packet to send, according to the modem hardware 
	 *                 and the chosen packet type. See the NMEA manual for more details.
	 * @return nmea_string, the requested NMEA message.
	 */
	std::string build_cccyc(int _src, int _dest, int _packtype, int _nframes);
	
	/** 
	 * Method to build a NMEA $CCTXD message (binary message). 
         * 
	 * @param _src the ID of the modem that must transmit the packet.
	 * @param _dest the ID of the modem that must receive the packet.
	 * @param _ack flag to enable the modem ARQ mechanism (0 or 1).
	 * @param _payload hex coded data bytes of the form HH..HH (two characters each, e.g., 00-FF).
	 * @return nmea_string, the requested NMEA message.
	 */
	std::string build_cctxd(int _src, int _dest, int _ack, std::string _payload);
        
        std::string build_cctxa(int _src, int _dest, int _ack, std::string _payload);
	
	/** 
	 * Method to build a NMEA $CCMUC message (minipacket message).
	 * 
         * @param _src the ID of the modem that must transmit the packet.
	 * @param _dest the ID of the modem that must receive the packet.
	 * @param _payload hex coded data bytes of the form BHHH (B in {0,1}, H in {0, 1, ..., D, F}, a total of 13 bits).
	 * @return nmea_string, the requested NMEA message.
	 */
	std::string build_ccmuc(int _src, int _dest, std::string _payload);
	
	
	// METHODS to PARSE MESSAGES
	// NOTE: These methods must know and use the reception variable of the linked UWdriver object and the corresponding methods to update them
	
	/** 
	 * Method to parse a NMEA $CACFG message (confirmation of a configuration message). 
         * @param nmea_string the received string.
	 */
	void parse_cacfg(std::string nmea_string);

	/**
	 * Method to parse a NMEA $CACYC message (reception of a cycle-init message). 
         * @param nmea_string the received string.
	 */
	void parse_cacyc(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CAMUC message (sent minipacket's confirmation). 
         * @param nmea_string the received string.
	 */
	void parse_camuc(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CATXP message (packet transmission started). 
         * @param nmea_string the received string.
	 */
	void parse_catxp(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CATXF message (packet transmission ended). 
         * @param nmea_string the received string.
	 */
	void parse_catxf(std::string nmea_string);

	void parse_catxd(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CAXST message (statistics of the transmitted packet). 
         * @param nmea_string the received string.
	 */
	void parse_caxst(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CADRQ message (free-to-send signal, for binary packet). 
         * @param nmea_string the received string.
	 */
	void parse_cadrq(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CADQF message (quality factor notification for an FSK packet). 
         * @param nmea_string the received string.
	 */
	void parse_cadqf(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CAMUA message (minipacket received). 
	 * NOTE: this method calls UWMdriver::updateRx(int,int,std::string). 
         * @param nmea_string the received string.
	 */
	void parse_camua(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CARXA message (binary packet received, with ASCII payload). 
         * @param nmea_string the received string.
	 */
	void parse_carxa(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CARXD message (binary packet received, with HEX payload).
	 * NOTE: this method calls UWMdriver::updateRx(int,int,std::string). 
         * @param nmea_string the received string.
	 */
	void parse_carxd(std::string nmea_string);
	
	/**
	 * Method to parse a NMEA $CACST message (statistics of the received packet). 
         * @param nmea_string the received string.
	 */
	void parse_cacst(std::string nmea_string);

	size_t undump(unsigned char* buffer,size_t offset, void* val, size_t h);
	
};
#endif	/* MINTERPRETERNMEA_H */
