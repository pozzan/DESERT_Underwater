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
 * @file   uwtdma.h
 * @author Filippo Campagnaro, Roberto Francescon
 * @version 1.0.0
 * 
 * @brief Provides the implementation of the class <i>UWTDMA</i>.
 * 
 */

#include "uwtdma.h"
#include <iostream>
#include <stdint.h>
#include <mac.h>

void UwTDMATimer::expire(Event *e) {
    ((UwTDMA *)module)->change_tdma_status();
}

/*void BufferTimer::expire(Event *e) {
    ((UwTDMA *)module)->stateTxData();
}*/

UwTDMA::UwTDMA() : MMac(), tdma_timer(this) {//, buffer_timer(this) {
  bind("slot_status", (int*)& slot_status);
  bind("slot_duration", (double*)& slot_duration);
  bind("frame_time", (double*)& frame_duration);
  bind("guard_time", (double*)& guard_time);
  bind("debug_", (int*) & debug_);
  channel_status=UW_CHANNEL_IDLE;
}

UwTDMA::~UwTDMA() {}

int UwTDMA::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
 	if (argc==2){
    	if(strcasecmp(argv[1], "start") == 0){
	      start();
	      return TCL_OK;
	    }
	    else if(strcasecmp(argv[1], "stop") == 0){
	      tdma_timer.cancel();
	      return TCL_OK;
	    } 
	    else if (strcasecmp(argv[1], "get_buffer_size") == 0){
	    	tcl.resultf("%d", buffer.size());
		  	return TCL_OK;
	    }
	    else if (strcasecmp(argv[1], "get_upper_data_pkts_rx") == 0){
	      tcl.resultf("%d", up_data_pkts_rx);
	      return TCL_OK;
	    }
  	}		
	else if (argc==3){
		if(strcasecmp(argv[1], "setSlotStatus") == 0){
     		slot_status=atoi(argv[2]);
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setHostId") == 0){
			host_id=atoi(argv[2]);
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setSlotDuration") == 0){
			slot_duration=atof(argv[2]);
			return TCL_OK;
		}
                else if(strcasecmp(argv[1], "setFrameDuration") == 0){
			frame_duration=atof(argv[2]);
			return TCL_OK;
		}
	}
	return MMac::command(argc, argv);
}

void UwTDMA::recvFromUpperLayers(Packet* p){
 	buffer.push(p);
	incrUpperDataRx();
  	txData();
}

void UwTDMA::stateTxData(){
	channel_status=UW_CHANNEL_IDLE;
	txData();
}

void UwTDMA::txData(){
	if(slot_status==UW_TDMA_STATUS_MY_SLOT && channel_status==UW_CHANNEL_IDLE){
	     if(buffer.size()>0){
	    	Packet* p = buffer.front();
	     	buffer.pop();
	      	Mac2PhyStartTx(p);
	     	incrDataPktsTx();
    	     }
  	}
  	else if(debug_<-5){
    	if(slot_status!=UW_TDMA_STATUS_MY_SLOT)
      		std::cout << NOW << " Wait my slot to send id " << host_id << "" << std::endl;
    	else
      		std::cout << NOW << " Wait earlier packet expires to send the current one id " 
 	     		 << host_id << "" << std::endl;
	}
}

void UwTDMA::Mac2PhyStartTx(Packet* p)
{
  channel_status=UW_CHANNEL_BUSY;
  MMac::Mac2PhyStartTx(p);
  //buffer_timer.resched(Mac2PhyTxDuration(p)*1.001);
  if(debug_<-5)
    std::cout << NOW << " Send packet id " << host_id << "" << std::endl;
}

void UwTDMA::Phy2MacEndTx(const Packet* p)
{
  // stateTxData();
}

void UwTDMA::Phy2MacStartRx(const Packet* p)
{
  // channel_status=UW_CHANNEL_BUSY;
}

void UwTDMA::Phy2MacEndRx(Packet* p)
{
  hdr_cmn* ch = HDR_CMN(p);
  hdr_mac* mach = HDR_MAC(p);
  int dest_mac = mach->macDA();
  // channel_status=UW_CHANNEL_IDLE;
  // if(slot_status==UW_TDMA_STATUS_MY_SLOT)
  // {
  //   txData();
  // }
  if ( ch->error() ){
    if (debug_) 
      cout << NOW << "  TDMA(" << addr 
	   << ")::Phy2MacEndRx() dropping corrupted pkt " << endl;
    rxPacketNotForMe(NULL);
  }
  else { 
    if ( dest_mac != addr || dest_mac != MAC_BROADCAST ) {
      rxPacketNotForMe(p);
    }
    else {
      sendUp(p);
      incrDataPktsRx();
    }
  }

}

void UwTDMA::change_tdma_status()
{
  if(slot_status==UW_TDMA_STATUS_MY_SLOT){
    tdma_timer.resched(frame_duration-slot_duration+guard_time);
    slot_status=UW_TDMA_STATUS_NOT_MY_SLOT;
    if(debug_<-5)
      std::cout << NOW << " Off id " << host_id << " " 
		<< frame_duration-slot_duration+guard_time << "" << std::endl;
  }
  else{
    tdma_timer.resched(slot_duration-guard_time);
    slot_status=UW_TDMA_STATUS_MY_SLOT;
    if(debug_<-5)
      std::cout << NOW << " On id " << host_id << " " << slot_duration-guard_time 
		<< "" << std::endl;
    stateTxData();
  }
}

void UwTDMA::rxPacketNotForMe(Packet* p)
{
  if ( p != NULL ) 
    Packet::free(p);
  else if (debug_)
    std::cout << NOW << "Host ID: " << host_id << " Not intended recipient" 
  	      << endl;
}

void UwTDMA::start()
{
  if(slot_status==UW_TDMA_STATUS_MY_SLOT)
    tdma_timer.resched(1); // go off
  else
    tdma_timer.resched(1+guard_time); // go on and start to transmit
  if(debug_<-5)
    std::cout << NOW << " Status " << slot_status << " id " << host_id << "" << std::endl;
}
