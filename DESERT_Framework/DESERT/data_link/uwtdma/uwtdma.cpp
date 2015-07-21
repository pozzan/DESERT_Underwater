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
 * @file   uwtdma.h
 * @author Filippo Campagnaro
 * @author Roberto Francescon
 * @version 1.0.1
 * 
 * @brief Provides the implementation of the class <i>UWTDMA</i>.
 * 
 */

#include "uwtdma.h"
#include <iostream>
#include <stdint.h>
#include <mac.h>

void UwTDMATimer::expire(Event *e)
{
    ((UwTDMA *)module)->changeStatus();
}

UwTDMA::UwTDMA() 
:
  MMac(), tdma_timer(this), 
  slot_status(UW_TDMA_STATUS_NOT_MY_SLOT), 
  channel_status(UW_CHANNEL_IDLE),
  tdma_sent_pkts(0),
  tdma_recv_pkts(0) 

{
  bind("slot_status", (int*)& slot_status);
  bind("slot_duration", (double*)& slot_duration);
  bind("frame_duration", (double*)& frame_duration);
  bind("guard_time", (double*)& guard_time);
  bind("debug_", (int*) & debug_);
}

UwTDMA::~UwTDMA() {}

void UwTDMA::recvFromUpperLayers(Packet* p)
{
  initPkt(p);
  buffer.push(p);
  incrUpperDataRx();
  txData();
}

void UwTDMA::stateTxData()
{
  channel_status=UW_CHANNEL_IDLE;
  txData();
}

void UwTDMA::txData()
{
  if(slot_status==UW_TDMA_STATUS_MY_SLOT && channel_status==UW_CHANNEL_IDLE){
    if(buffer.size()>0){
      Packet* p = buffer.front();
      buffer.pop();
      Mac2PhyStartTx(p);
      incrDataPktsTx();
    }
  }
  else if(debug_<-5)
  {
    if(slot_status!=UW_TDMA_STATUS_MY_SLOT)
      std::cout << NOW << " ID " << addr << ": Wait my slot to send" 
		<< std::endl;
    else
      std::cout << NOW << "ID " << addr 
                << ": Wait earlier packet expires to send the current one" 
		<< std::endl;
  }
}

void UwTDMA::Mac2PhyStartTx(Packet* p)
{
  channel_status=UW_CHANNEL_BUSY;
  MMac::Mac2PhyStartTx(p);
  if(debug_<-5)
    std::cout << NOW <<" ID "<< addr << ": Sending packet" << std::endl;
}

void UwTDMA::Phy2MacEndTx(const Packet* p)
{
  stateTxData();
}

void UwTDMA::Phy2MacStartRx(const Packet* p)
{
  channel_status=UW_CHANNEL_BUSY;
}

void UwTDMA::Phy2MacEndRx(Packet* p)
{
  hdr_cmn* ch = HDR_CMN(p);
  hdr_mac* mach = HDR_MAC(p);
  int dest_mac = mach->macDA();
  int src_mac = mach->macSA();
  channel_status=UW_CHANNEL_IDLE;

  if ( ch->error() ){
    if (debug_) 
      cout << NOW << "  TDMA(" << addr 
	   << ")::Phy2MacEndRx() dropping corrupted pkt " << endl;
    rxPacketNotForMe(NULL);
  }
  else { 
    if ( dest_mac != addr && dest_mac != MAC_BROADCAST ) {
      rxPacketNotForMe(p);
      if (debug_<-5)
	std::cout<<NOW<<" ID "<<addr<<": packet was for "<<dest_mac<<std::endl;
    }
    else {
      sendUp(p);
      incrDataPktsRx();
      if (debug_<-5)
	std::cout<<NOW<<" ID "<<addr<<": Received packet from "
                                    << src_mac<<std::endl;
    }
  }
  if(slot_status==UW_TDMA_STATUS_MY_SLOT){
    txData();
  }

}

void UwTDMA::initPkt(Packet* p)
{
  hdr_cmn* ch = hdr_cmn::access(p);
  hdr_mac* mach = HDR_MAC(p);

  int curr_size = ch->size();

  ch->size() = curr_size + HDR_size;
}

void UwTDMA::rxPacketNotForMe(Packet* p)
{
  if ( p != NULL ) 
    Packet::free(p);
}

void UwTDMA::changeStatus()
{
  if(slot_status==UW_TDMA_STATUS_MY_SLOT){
    
    slot_status=UW_TDMA_STATUS_NOT_MY_SLOT;
    tdma_timer.resched(frame_duration-slot_duration+guard_time);
    
    if(debug_<-5)
      std::cout << NOW << " Off id " << addr << " " 
		<< frame_duration-slot_duration+guard_time << "" << std::endl;
  } else {

    slot_status=UW_TDMA_STATUS_MY_SLOT;
    tdma_timer.resched(slot_duration-guard_time);

    if(debug_<-5)
      std::cout <<NOW<< " On id " << addr << " " << slot_duration-guard_time 
		<< "" << std::endl;
    stateTxData();
  }
}

void UwTDMA::start(float delay)
{
  // if(slot_status=UW_TDMA_STATUS_MY_SLOT){
    
  //   tdma_timer.sched(slot_duration-guard_time);
  // }
  // else{
  //   slot_status=UW_TDMA_STATUS_NOT_MY_SLOT;
  //   tdma_timer.sched(abs(slot_number-start_slot)*slot_duration);
  // }
  // if(debug_<-5)
  //   std::cout << NOW << " Status " << slot_status << " id " << addr 
  // 	      << "" << std::endl;

  tdma_timer.sched(delay);

  if(debug_<-5)
    std::cout << NOW << " Status " << slot_status << " on ID " << addr 
  	      << "" << std::endl;
}
