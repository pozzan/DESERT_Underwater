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
 * @file mfsk_whoi_mm/minterpreterNMEA.cc
 * @author Riccardo Masiero, Matteo Petrani
 * \version 1.0.0
 * \brief Implementation of the MinterpreterNMEA class.
 */

#include "minterpreterNMEA.h"
#include "mdriverFSK_WHOI_MM.h"
#include <uwmdriver.h>


MinterpreterNMEA::MinterpreterNMEA(UWMdriver* pmDriver_) : UWMinterpreter(pmDriver_)
{
	 if (debug_)
	 {
		  cout << this << ": in constructor of MinterpreterNMEA which points to driver: " << pmDriver << "\n";
	 }
}

MinterpreterNMEA::~MinterpreterNMEA()
{
}

// METHODS to BUILD MESSAGES

static std::string hexdumplog(std::string str) {
    int len = str.size();
    const char *data = str.c_str();
    std::string str_out = "";

    for (int i = 0; i < len; i++) {
        if (std::isalnum(data[i]) || std::ispunct(data[i]))
            str_out += data[i];
        else {
            std::string str;
            std::stringstream sstr("");
            sstr << "[" << std::hex << (unsigned int) (unsigned char) data[i] << std::dec << "]";
            sstr >> str;
            str_out += str;
        }
    }
    return str_out;
}

std::string MinterpreterNMEA::build_cccfg(std::string _param, int _value)
{
	 // NMEA string to build
	 string nmea_string;
	 // Create stringstream to build the nmea_string
	 stringstream nstr("");
	 // Build the string and put it in nmea_string
	 nstr << "$CCCFG," << _param << "," << _value;
	 nstr >> nmea_string;
	 // Return the created nmea_string
	 return nmea_string;
}

std::string MinterpreterNMEA::build_cccyc(int _src, int _dest, int _packtype, int _nframes)
{
	 // NMEA string to build
	 string nmea_string;
	 // Create stringstream to build the nmea_string
	 stringstream nstr("");
	 // Build the string and put it in nmea_string
	 nstr << "$CCCYC,1," << _src << "," << _dest << "," << _packtype << ",0," << _nframes;
	 nstr >> nmea_string;
	 // Return the created nmea_string
	 return nmea_string;
}

std::string MinterpreterNMEA::build_cctxd(int _src, int _dest, int _ack, std::string _payload)
{
	// NMEA string to build
	string nmea_string;
	// Create stringstream to build the nmea_string
	stringstream nstr("");
	// Build the string and put it in nmea_string
	nstr << "$CCTXD," << _src << "," << _dest << "," << _ack << ",";
	nstr >> nmea_string;

	//------------ conversion of payload
	const char *payload_char = _payload.c_str();
	std::string str_out;
	for(int i = 0; i < _payload.size(); i++)
	{
        std::string str;
        std::stringstream sstr("");
        //sstr << std::hex << (unsigned int) (unsigned char) payload_char[i] << std::dec;
        sstr << (unsigned int) payload_char[i];
        sstr >> str;
        str_out += str;
	}

	// ---- padding 
	if (str_out.size() % 2 != 0) 
	{
	 	cout << "Doing padding!! " << endl;
	 	str_out += "0";
	}
	//cout << "Payload transmitted " << str_out << endl;
	nmea_string = nmea_string + str_out ;
	// --------

	cout << ">> CCTXD message = " << nmea_string << endl;
	cout << endl;
	// Return the created nmea_string
	return nmea_string;
}

std::string MinterpreterNMEA::build_cctxa(int _src, int _dest, int _ack, std::string _payload)
{
	 // NMEA string to build
	 string nmea_string;
	 // Create stringstream to build the nmea_string
	 stringstream nstr("");
	 // Build the string and put it in nmea_string
	 nstr << "$CCTXA," << _src << "," << _dest << "," << _ack << ",";
	 nstr >> nmea_string;
         cout << "Payload transmitted " << hexdumplog(_payload) << endl;
	 nmea_string = nmea_string + _payload;
	 // Return the created nmea_string
	 return nmea_string;
}



std::string MinterpreterNMEA::build_ccmuc(int _src, int _dest, std::string _payload)
{
	 // NMEA string to build
	 string nmea_string;
	 // Create stringstream to build the nmea_string
	 stringstream nstr("");
	 // Build the string and put it in nmea_string
	 nstr << "$CCMUC," << _src << "," << _dest << "," << _payload;
	 nstr >> nmea_string;
	 // Return the created nmea_string
	 return nmea_string;
}

// METHODS to PARSE MESSAGES
// NOTE: These methods must know and use the reception variable of the linked UWdriver object and the corresponding methods to update them 

void MinterpreterNMEA::parse_cacfg(std::string nmea_string)
{
	 // Function unused at the moment
}

void MinterpreterNMEA::parse_cacyc(std::string nmea_string)
{
    cout << "<< NMEA CAYC string " << nmea_string << endl;
}

void MinterpreterNMEA::parse_camuc(std::string nmea_string)
{
	 // Function unused at the moment
}

void MinterpreterNMEA::parse_catxp(std::string nmea_string)
{
	 // Function unused at the moment
}

void MinterpreterNMEA::parse_catxf(std::string nmea_string)
{
	 // Function unused at the moment
}

void MinterpreterNMEA::parse_caxst(std::string nmea_string)
{
	 // Function unused at the moment
}

void MinterpreterNMEA::parse_cadrq(std::string nmea_string)
{
	 cout << "<< NMEA CADRQ string " << nmea_string << endl;
}

void MinterpreterNMEA::parse_cadqf(std::string nmea_string)
{
	 // Function unused at the moment
}

void MinterpreterNMEA::parse_camua(std::string nmea_string)
{
	 // All the fields contained in the message to parse
	 std::string _prefix, _src, _dest, _payload;
	 // Input stringstream to be associated with nmea_string
	 istringstream instr(nmea_string);
	 // Parse nmea_string.
	 getline(instr, _prefix, ',');
	 getline(instr, _src, ',');
	 getline(instr, _dest, ',');
	 getline(instr, _payload, '*');
	 
	 // Update the RX fields of the associated UWMdriver
	 pmDriver -> updateRx(atoi(_src.c_str()), atoi(_dest.c_str()), _payload);
}

void MinterpreterNMEA::parse_carxa(std::string nmea_string)
{
    // All the fields contained in the message to parse
	std::string _prefix, _src, _dest, _ack, _frameN, _payload;
	 // Input stringstream to be associated with nmea_string
	istringstream instr(nmea_string);
	 // Parse nmea_string.
	 getline(instr, _prefix, ',');
	 getline(instr, _src, ',');
	 getline(instr, _dest, ',');
	 getline(instr, _ack, ',');
	 getline(instr, _frameN, ',');
	 getline(instr, _payload, '*');
     cout << "PREFIX " << _prefix << endl;
     cout << "SRC " << _src << endl;
     cout << "DST " << _dest << endl;
     cout << "ACK" << _ack << endl;
     cout << "FRAME " << _frameN << endl;
	 cout << "PAYLOAD " << hexdumplog(_payload) << endl;
	 // Update the RX fields of the associated UWMdriver
	 //pmDriver -> updateRx(atoi(_src.c_str()), atoi(_dest.c_str()), _payload);
}

void MinterpreterNMEA::parse_catxd(std::string nmea_string)
{
	std::string _prefix, _src, _dest, _ack, _nbytes;

	istringstream instr(nmea_string);

	cout << "<< NMEA CATXD string " << nmea_string << endl;

	getline(instr, _prefix, ',');
	getline(instr, _src, ',');
	getline(instr, _dest, ',');
	getline(instr, _ack, ',');
	getline(instr, _nbytes, '*');

	//cout << "Nbytes transmitted" << _nbytes << endl;

}

void MinterpreterNMEA::parse_carxd(std::string nmea_string)
{
	// All the fields contained in the message to parse
	std::string _prefix, _src, _dest, _ack, _frameN, _payload;
	// Input stringstream to be associated with nmea_string
	istringstream instr(nmea_string);
	// Parse nmea_string.
	getline(instr, _prefix, ',');
	getline(instr, _src, ',');
	getline(instr, _dest, ',');
	getline(instr, _ack, ',');
	getline(instr, _frameN, ',');
	getline(instr, _payload, '*');
	//std::string _payload_sub = _payload.substr(0,_payload.size() - 1);
	cout << "PREFIX " << _prefix << endl;
    cout << "SRC " << _src << endl;
    cout << "DST " << _dest << endl;
    cout << "ACK" << _ack << endl;
    cout << "FRAME " << _frameN << endl;
	//Update the RX fields of the associated UWMdriver
	//pmDriver -> updateRx(atoi(_src.c_str()), atoi(_dest.c_str()), _payload_sub);
	pmDriver -> updateRx(atoi(_src.c_str()), atoi(_dest.c_str()), _payload);
}

void MinterpreterNMEA::parse_cacst(std::string nmea_string)
{
	 // Function unused at the moment
}


size_t MinterpreterNMEA::undump(unsigned char* buffer,size_t offset, void* val, size_t h) 
{
	//
}