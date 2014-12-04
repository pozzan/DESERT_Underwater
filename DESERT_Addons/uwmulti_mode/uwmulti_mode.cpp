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
//

/**
 * @file   uwmulti_mode.cpp
 * @author Filippo Campagnaro
 * @version 1.0.0
 * 
 * \brief Provides the implementation of the class <i>UwMultiMode</i>.
 * 
 */

#include "uwmulti_mode.h"

#include <phymac-clmsg.h>

#include <mac.h>
#include <iostream>

#include <stdint.h>

static class UwMultiModeClass : public TclClass {
public:

	/**
	 * Constructor of the class
	*/
	UwMultiModeClass() : TclClass("Module/UW/MULTI/MODE") {}

	/**
	* Creates the TCL object needed for the tcl language interpretation
	* @return Pointer to an TclObject
	*/
	TclObject* create(int, const char*const*) {
		return (new UwMultiMode());
	}

} class_uwmulti_mode;

map< UwMultiMode::UWMULTI_MODE_STATUS, string> UwMultiMode::status_info;

UwMultiMode::UwMultiMode() 
:
MMac(),
initialized(false),
current_state(UWMULTI_MODE_STATE_IDLE),
recv_physical_id(0)
{
    bind("send_physical_id", &send_physical_id);
    bind("debug_", &debug_);
    bind("switch_mode", &switch_mode);
    initInfo();
}

UwMultiMode::~UwMultiMode() {
}

void UwMultiMode::initInfo() {

    initialized = true;

    status_info[UWMULTI_MODE_STATE_IDLE] = "STATE_IDLE";
    status_info[UWMULTI_MODE_STATE_TX] = "STATE_TX";
    status_info[UWMULTI_MODE_STATE_RX] = "STATE_RX";
    status_info[UWMULTI_MODE_STATE_RX_TX] = "STATE_RX_TX";
    
}

int UwMultiMode::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
	if (argc == 2) {
		if(strcasecmp(argv[1], "setAutomaticSwitch") == 0) {
     		switch_mode = UW_AUTOMATIC_SWITCH;
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setManualSwitch") == 0) {
     		recv_physical_id = UW_MANUAL_SWITCH;
			return TCL_OK;
		}
		else if (strcasecmp(argv[1], "initialize") == 0) {
            if (initialized == false) initInfo();
			return TCL_OK;
		}
		else if (strcasecmp(argv[1], "getBufferSize") == 0) {
            tcl.resultf("%d", buffer.size());
			return TCL_OK;
		}
	}
	else if (argc == 3) {
		if(strcasecmp(argv[1], "setSendPhysicalId") == 0){
     		send_physical_id = atoi(argv[2]);
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setSwitchMode") == 0) {
     		switch_mode = atoi(argv[2]);
			return TCL_OK;
		}
	}
	else if (argc == 4) {
		if(strcasecmp(argv[1], "addPhysical") == 0){
     		addPhysical(atof(argv[2]),(atoi(argv[3])));
			return TCL_OK;
		}
	}
	return MMac::command(argc, argv);
}

// ACTIONS

void UwMultiMode::Mac2PhyStartTx(Packet* p)
{

	if(!send_physical_id)
		MMac::Mac2PhyStartTx(p);
	else
		MMac::Mac2PhyStartTx(send_physical_id, p);
}

void UwMultiMode::stateTxData()
{
	if(buffer.size() > 0) {
		current_state = UWMULTI_MODE_STATE_TX;
		Mac2PhyStartTx(buffer.front());
		buffer.pop();
		if (debug_)
			std::cout << NOW << " MultiMode(" << addr << ") stateTxData, sending, state: " 
				<< status_info[current_state] <<  std::endl;
	}
	else{
		current_state = UWMULTI_MODE_STATE_IDLE;
		if (debug_)
			std::cout << NOW << " MultiMode(" << addr << ") stateTxData, nothing to send, state: " 
				<< status_info[current_state] <<  std::endl;
	}

}
void UwMultiMode::stateRxTx() // only stateRx --> stateRxTx
{
	if(buffer.size() > 0) {
		current_state = UWMULTI_MODE_STATE_RX_TX;
		Mac2PhyStartTx(buffer.front());
		buffer.pop();
		if (debug_)
			std::cout << NOW << " MultiMode(" << addr << ") stateRxTx, sending, state: " 
				<< status_info[current_state] <<  std::endl;
	}
	else {
		current_state = UWMULTI_MODE_STATE_RX;
		if (debug_)
			std::cout << NOW << " MultiMode(" << addr << ") stateRxTx, nothing to send, state: " 
				<< status_info[current_state] <<  std::endl;
	}
}

//EVENTS 

void UwMultiMode::recvFromUpperLayers(Packet* p)
{
	buffer.push(p);
	switch (current_state) {
		case UWMULTI_MODE_STATE_IDLE :{
			stateTxData();
		}
			break;
		case UWMULTI_MODE_STATE_RX :{
			if(recv_physical_id != send_physical_id)
				stateRxTx();
		}
			break;
		default :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") recvFromUpperLayers and waiting to transmit, state: " 
	    			<< status_info[current_state] <<  std::endl;
		}
	}
}

void UwMultiMode::Phy2MacEndTx(const Packet* p){
	if (debug_)
		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndTx correctly, state: " 
			<< status_info[current_state] <<  std::endl;
	switch (current_state) {
		case UWMULTI_MODE_STATE_TX :{
			stateTxData();
		}
			break;
		case UWMULTI_MODE_STATE_RX_TX :{
			if ( recv_physical_id != send_physical_id)
				stateRxTx();
		}
			break;
		default :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndTx in an unexpected state, state: " 
	    			<< status_info[current_state] <<  std::endl;
		}
	}
}

void UwMultiMode::Phy2MacStartRx(const Packet* p, int idSrc)
{
	recv_physical_id = idSrc;
	switch (current_state) {
		case UWMULTI_MODE_STATE_IDLE :{
			current_state = UWMULTI_MODE_STATE_RX;
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx correctly, state: " 
	    			<< status_info[current_state] <<  std::endl;			
		}
			break;
		case UWMULTI_MODE_STATE_TX :{
			if (recv_physical_id != send_physical_id){
				current_state = UWMULTI_MODE_STATE_RX_TX;
				if (debug_)
		    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx correctly, state: " 
		    			<< status_info[current_state] <<  std::endl;
			}
			else {
				current_state = UWMULTI_MODE_STATE_IDLE;
				error_pkts_rx ++;
				if (debug_)
		    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx error, state: " 
		    			<< status_info[current_state] <<  std::endl;
			}
		}
			break;
		default :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx in an unexpected state, state: " 
	    			<< status_info[current_state] <<  std::endl;
		}
	}
}

void UwMultiMode::Phy2MacEndRx(Packet* p, int idSrc){
	hdr_cmn* ch = HDR_CMN(p);
	hdr_mac* mach = HDR_MAC(p);
	int source_mac = mach->macSA();
	int dest_mac = mach->macDA();
	if (ch->error() || idSrc != recv_physical_id || (dest_mac != addr && dest_mac == MAC_BROADCAST)){
		if (debug_)
	    	std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx packet error: " 
	    		<< ch->error() << ", idSrc: " << idSrc << ", recv_physical_id: " 
	    		<< recv_physical_id << ", send_physical_id: " << send_physical_id  
	    		<< ", dest_mac: " << dest_mac << "my addr" << addr << ", source_mac: " 
	    		<< source_mac << std::endl;
    	drop(p, 1, "ERR");
    /*	switch (current_state) {
    		case UWMULTI_MODE_STATE_RX : {
    			stateTxData();
    		}
    			break;
    		case UWMULTI_MODE_STATE_RX_TX :{
    			current_state = UWMULTI_MODE_STATE_TX;
    		}
    			break;*/
	}
	else
	switch (current_state) {
		case UWMULTI_MODE_STATE_RX :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx correctly, state: " 
	    			<< status_info[current_state] <<  std::endl;
			sendUp(p);
			stateTxData();
		}
			break;
		case UWMULTI_MODE_STATE_RX_TX :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx correctly, state: " 
	    			<< status_info[current_state] <<  std::endl;
			sendUp(p);
			current_state = UWMULTI_MODE_STATE_TX;
		}
			break;
		default :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx in an unexpected state, state: " 
	    			<< status_info[current_state] <<  std::endl;
		}
	}
}

void UwMultiMode::recv(Packet *p, int idSrc){
	hdr_cmn *ch = HDR_CMN(p);
  	if(ch->direction() == hdr_cmn::DOWN)
    {
    	recvFromUpperLayers(p);
    }
  	else
    {
      	Phy2MacEndRx(p, idSrc);
    }
}

// ClMessage manager

int UwMultiMode::recvSyncClMsg(ClMessage* m)
{
	if (m->type() == CLMSG_PHY2MAC_STARTRX)
    {
      	Phy2MacStartRx(((ClMsgPhy2MacEndTx*)m)->pkt, m->getSource());
      	return 0;
    }
  	else return MMac::recvSyncClMsg(m);
}

/*
*
*	CONTROLLER STUFF
*	TODO: decide the policy and implement it
*
*/

void UwMultiMode::addPhysical(double distance, int phyId){
	physical_map.insert(std::pair<char,int>(distance,phyId));
}
