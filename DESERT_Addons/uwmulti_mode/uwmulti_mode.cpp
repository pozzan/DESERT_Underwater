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

void PhyMultiRecvSet::add(int id){
	if (! contains(id))
		recv_physical_.insert(std::pair<int,int>(id,1));
	else{
		int new_value = find(id)+1;
		recv_physical_.erase(id);
		recv_physical_.insert(std::pair<int,int>(id,new_value));
	}
}
void PhyMultiRecvSet::remove(int id){
	if (! contains(id))
		return;
	if (find(id)>1){
		int new_value = find(id)-1;
		recv_physical_.erase(id);
		recv_physical_.insert(std::pair<int,int>(id,new_value));
	}
	else
		recv_physical_.erase(id);
}

map< UwMultiMode::UWMULTI_MODE_STATE, string> UwMultiMode::state_info;

UwMultiMode::UwMultiMode() 
:
MMac(),
recv_physical_ids(),
/*gioco(),*/
initialized(false),
current_state(UWMULTI_MODE_STATE_IDLE)
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

    state_info[UWMULTI_MODE_STATE_IDLE] = "STATE_IDLE";
    state_info[UWMULTI_MODE_STATE_TX] = "STATE_TX";
    state_info[UWMULTI_MODE_STATE_RX] = "STATE_RX";
    state_info[UWMULTI_MODE_STATE_RX_TX] = "STATE_RX_TX";

   /* gioco.add(1);
    if (gioco.contains(1))
    	std::cout << "game1 works" << std::endl;
    else
    	std::cout << "game1 doesn't work" << std::endl;
    if (! gioco.contains(2))
    	std::cout << "game2 works" << std::endl;
    else
    	std::cout << "game2 doesn't work" << std::endl;
    gioco.add(1);
    gioco.remove(1);
    if (gioco.contains(1))
    	std::cout << "game3 works" << std::endl;
    else
    	std::cout << "game3 doesn't work" << std::endl;
    gioco.remove(1);
    if (! gioco.contains(1))
    	std::cout << "game4 works" << std::endl;
    else
    	std::cout << "game4 doesn't work" << std::endl;*/
    
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
     		switch_mode = UW_MANUAL_SWITCH;
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
				<< state_info[current_state] <<  std::endl;
	}
	else{
		current_state = UWMULTI_MODE_STATE_IDLE;
		if (debug_)
			std::cout << NOW << " MultiMode(" << addr << ") stateTxData, nothing to send, state: " 
				<< state_info[current_state] <<  std::endl;
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
				<< state_info[current_state] <<  std::endl;
	}
	else {
		current_state = UWMULTI_MODE_STATE_RX;
		if (debug_)
			std::cout << NOW << " MultiMode(" << addr << ") stateRxTx, nothing to send, state: " 
				<< state_info[current_state] <<  std::endl;
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
			if( ! recv_physical_ids.contains(send_physical_id) )
				stateRxTx();
		}
			break;
		default :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") recvFromUpperLayers and waiting to transmit, state: " 
	    			<< state_info[current_state] <<  std::endl;
		}
	}
}

void UwMultiMode::Phy2MacEndTx(const Packet* p){
	if (debug_)
		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndTx correctly, state: " 
			<< state_info[current_state] <<  std::endl;
	switch (current_state) {
		case UWMULTI_MODE_STATE_TX :{
			stateTxData();
		}
			break;
		case UWMULTI_MODE_STATE_RX_TX :{
			if ( ! recv_physical_ids.contains(send_physical_id) )
				stateRxTx();
		}
			break;
		default :{
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndTx in an unexpected state, state: " 
	    			<< state_info[current_state] <<  std::endl;
		}
	}
}

void UwMultiMode::Phy2MacStartRx(const Packet* p, int idSrc)
{
	recv_physical_ids.add(idSrc);
	switch (current_state) {
		case UWMULTI_MODE_STATE_IDLE :{
			current_state = UWMULTI_MODE_STATE_RX;
			if (debug_)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx correctly, state: " 
	    			<< state_info[current_state] <<  std::endl;			
		}
			break;
		case UWMULTI_MODE_STATE_TX :{
			if (! recv_physical_ids.contains(send_physical_id)){
				current_state = UWMULTI_MODE_STATE_RX_TX;
				if (debug_)
		    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx correctly, state: " 
		    			<< state_info[current_state] <<  std::endl;
			}
			else {
				current_state = UWMULTI_MODE_STATE_IDLE;
				if (debug_)
		    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx error, state: " 
		    			<< state_info[current_state] <<  std::endl;
			}
		}
			break;
		default :{
			if (debug_ && recv_physical_ids.find(idSrc)>1)
	    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacStartRx in an unexpected state, state: " 
	    			<< state_info[current_state] <<  std::endl;
		}
	}
}

void UwMultiMode::Phy2MacEndRx(Packet* p, int idSrc){
	hdr_cmn* ch = HDR_CMN(p);
	hdr_mac* mach = HDR_MAC(p);
	int source_mac = mach->macSA();
	int dest_mac = mach->macDA();
	if (ch->error()){
		recv_physical_ids.remove(idSrc);
		if (debug_)
	    	std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx packet error: channel error= " 
	    		<< ch->error() << std::endl;
    	drop(p, 1, "ERR");
    	error_pkts_rx ++;
    	switch (current_state) {
    		case UWMULTI_MODE_STATE_RX : {
    			if( recv_physical_ids.isEmpty() )
					stateTxData();
				else if( ! recv_physical_ids.contains(send_physical_id) )
					stateRxTx();
    		}
    			break;
    		case UWMULTI_MODE_STATE_RX_TX :{
    			if( recv_physical_ids.isEmpty() )
					current_state = UWMULTI_MODE_STATE_TX;
    		}
    			break;
    	}
	}
	else if ((dest_mac != addr && dest_mac != MAC_BROADCAST)){
		if (debug_)
	    	std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx packet error: " 
	    		<< "packet not for me, dest_mac: " << dest_mac << ", my addr: " << addr 
	    		<< ", source_mac: " << source_mac << std::endl;
    	drop(p, 1, "ERR");
		recv_physical_ids.remove(idSrc);
    	switch (current_state) {
    		case UWMULTI_MODE_STATE_RX : {
    			if( recv_physical_ids.isEmpty() )
					stateTxData();
				else if( ! recv_physical_ids.contains(send_physical_id) )
					stateRxTx();
    		}
    			break;
    		case UWMULTI_MODE_STATE_RX_TX :{
    			if( recv_physical_ids.isEmpty() )
					current_state = UWMULTI_MODE_STATE_TX;
    		}
    			break;
    		default :{
    			if (debug_)
	    			std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx in an unexpected state, state: " 
	    				<< state_info[current_state] <<  std::endl;
    		}
    			break;
    	}
	}
	else if(! recv_physical_ids.contains(idSrc)){
		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx in an unexpected state, state due to idSrc: " 
	    	<< state_info[current_state] <<  std::endl;
	}
	else{
		recv_physical_ids.remove(idSrc);
		switch (current_state) { // the packet is mine and it's correctely received
			case UWMULTI_MODE_STATE_RX :{
				if (debug_)
		    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx correctly, state: " 
		    			<< state_info[current_state] <<  std::endl;
				sendUp(p);
				if( recv_physical_ids.isEmpty() )
					stateTxData();
				else if( ! recv_physical_ids.contains(send_physical_id) )
					stateRxTx();
			}
				break;
			case UWMULTI_MODE_STATE_RX_TX :{
				if (debug_)
		    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx correctly, state: " 
		    			<< state_info[current_state] <<  std::endl;
				sendUp(p);
				if( recv_physical_ids.isEmpty() )
					current_state = UWMULTI_MODE_STATE_TX;
			}
				break;
			default :{
				//if (debug_)
		    		std::cout << NOW << " MultiMode(" << addr << ") Phy2MacEndRx goes in default in an unexpected state, state: " 
		    			<< state_info[current_state] <<  std::endl;
			}
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
	send_physical_map.insert(std::pair<double,int>(distance,phyId));
}
