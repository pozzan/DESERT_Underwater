//
// Copyright (c) 2014 Regents of the SIGNET lab, University of Padova.
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
 * @file   uwmulti-traffic-range-crt.cc
 * @author Filippo Campagnaro, Federico Guerra
 * @version 1.0.0
 *
 * \brief Implementation of UwMultiTrafficControl class.
 *
 */

#include "uwmulti-traffic-range-crt.h"
#include "uwip-clmsg.h"
#include <iostream>

extern packet_t PT_MUTLI_TR_PROBE;
extern packet_t PT_MUTLI_TR_PROBE_ACK;

int hdr_uwm_tr::offset_;

void UwMultiTrafficRangeCtr::UwCheckRangeTimer::expire(Event *e) 
{ 
  UwCheckRangeTimer::num_expires++; 
  if (UwCheckRangeTimer::num_expires > UwCheckRangeTimer::max_increment)
    UwCheckRangeTimer::num_expires = UwCheckRangeTimer::max_increment;
  UwCheckRangeTimer::module->timerExpired(UwCheckRangeTimer::traffic); 
}

/**
 * Class that represents the binding with the tcl configuration script 
 */
static class UwMultiTrafficRangeCtrClass : public TclClass 
{
public:
  /**
   * Constructor of the class
   */
  UwMultiTrafficRangeCtrClass() : TclClass("Module/UW/MULTI_TRAFFIC_RANGE_CTR") {}
  /**
   * Creates the TCL object needed for the tcl language interpretation
   * @return Pointer to an TclObject
   */
  TclObject* create(int, const char*const*) 
  {
    return (new UwMultiTrafficRangeCtr);
  }
} class_tr_range_control;

UwMultiTrafficRangeCtr::UwMultiTrafficRangeCtr() 
: 
  UwMultiTrafficControl(),
  check_to_period(0),
  signaling_pktSize(1),
  status(),
  timers()
{ 
  bind("check_to_period_", &check_to_period);
  bind("signaling_pktSize_", &signaling_pktSize);
}

int UwMultiTrafficRangeCtr::command(int argc, const char*const* argv) 
{
	if (argc == 4) 
  {
    if(strcasecmp(argv[1], "addRobustLowLayer") == 0)
    {
      addLowLayerFromTag(atoi(argv[2]),argv[3],ROBUST);
      return TCL_OK;
    }
    else if(strcasecmp(argv[1], "addFastLowLayer") == 0)
    {
      addLowLayerFromTag(atoi(argv[2]),argv[3],CHECK_RANGE);
      return TCL_OK;
    }
	}	
  return UwMultiTrafficControl::command(argc, argv);     
} /* UwMultiTrafficRangeCtr::command */

void UwMultiTrafficRangeCtr::recv(Packet* p, int idSrc)
{
  hdr_cmn *ch = HDR_CMN(p);
  if (ch->direction() == hdr_cmn::UP) {
    if (ch->ptype() == PT_MUTLI_TR_PROBE_ACK) {
      hdr_uwip* iph  = HDR_UWIP(p);
      if (debug_)
        std::cout << "UwMultiTrafficRangeCtr::recv PT_MUTLI_TR_PROBE_ACK from:" 
                  << (int)(iph->saddr()) << " in layer " << idSrc << std::endl;
      manageCheckedLayer(HDR_UWMTR(p)->traffic() , iph->saddr(), true);      
      Packet::free(p);
    }
    else if (ch->ptype() == PT_MUTLI_TR_PROBE) {
      UWIPClMsgSendAddr msg;
      sendSyncClMsgUp(&msg);
      hdr_uwip* iph  = HDR_UWIP(p);
      if (iph->daddr() ==  msg.getAddr() || iph->saddr() == UWIP_BROADCAST) {
        ch->ptype() = PT_MUTLI_TR_PROBE_ACK;
        if (debug_)
          std::cout << "UwMultiTrafficRangeCtr::recv PT_MUTLI_TR_PROBE from:" 
                   << (int)iph->saddr() << " in layer " << idSrc << std::endl;
        iph->daddr() = iph->saddr();
        iph->saddr() =  msg.getAddr();
        sendDown(idSrc, p);
      }
      else {
        Packet::free(p);
      }
    }
    else
      UwMultiTrafficControl::recv(p);
  }
  else
    UwMultiTrafficControl::recv(p);
}

void UwMultiTrafficRangeCtr::manageCheckedLayer(int traffic, uint8_t destAdd, bool in_range)
{
  std::map<int, UwCheckRangeTimer*>::iterator it_t = timers.find(traffic);
  if (it_t == timers.end()) 
    return;
  UwCheckRangeTimer *to = it_t->second;
  if(to->status() == TIMER_IDLE) {
    return; // nothing to check is pending
  }
  else {
    StatusMap::iterator it_s = status.find(traffic);
    if (it_s == status.end()) 
      return;
    if(status[traffic].status == RANGE_CNF_WAIT){
      if(in_range) {
        Packet *p = NULL;
        while(true){
          p = removeFromBuffer(traffic);
          /*if(p != NULL && HDR_UWIP(p)->daddr() != destAdd) {
            Packet *p0 = p;
            do {
              insertInBuffer(p,traffic);
              p = removeFromBuffer(traffic);
            } while (p != NULL && p != p0 && (HDR_UWIP(p)->daddr() != destAdd ));
          }*/
          if(p != NULL && HDR_UWIP(p)->daddr() == destAdd) {
            to->force_cancel();
            to->num_expires = 0;
            status[traffic].status = IDLE;
            //do {
              sendDown(status[traffic].module_id,p);
              //p = removeFromBuffer(traffic);
            //} while (p != NULL && HDR_UWIP(p)->daddr() == destAdd);
          }
          else if(p == NULL) {
            if(debug_)
              std::cout << "UwMultiTrafficRangeCtr::manageCheckedLayer nothing to send to no one" << std::endl;
            break;
          }

          else{
            insertInBuffer(p,traffic);
            if(debug_)
              std::cout << "UwMultiTrafficRangeCtr::manageCheckedLayer wrong daddr " << (int)HDR_UWIP(p)->daddr() 
                        << ", " << (int)destAdd << std::endl;
            break;
          }
        }
      }
      else {
        if(status[traffic].robust_id) {
          status[traffic].status = IDLE;
          to->force_cancel();
          to->num_expires = 0;
          sendDown(status[traffic].module_id,removeFromBuffer(traffic));
        }
        else {
          to->force_cancel();
          ++to->num_expires;
          checkRange(traffic, status[traffic].module_id);
        }
      }
    }
  }
}


void UwMultiTrafficRangeCtr::manageBuffer(int traffic)
{
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end() && ! (it->second)->empty()) {
    int l_id = getBestLowerLayer(traffic,(it->second)->front());
    if (status[traffic].status == IDLE) {
      return l_id ? sendDown(l_id,removeFromBuffer(traffic)) 
                  : sendDown(removeFromBuffer(traffic));
    }
  }
}
  
int UwMultiTrafficRangeCtr::getBestLowerLayer(int traffic, Packet *p) 
{
  if (debug_)
    std::cout << "UwMultiTrafficRangeCtr::getBestLowerLayer(" << traffic << ")" << std::endl;
  DownTrafficMap::iterator it = down_map.find(traffic); 
  if (it != down_map.end()) {
    StatusMap::iterator it_s = status.find(traffic);
    if (it_s == status.end()) {
      initStatus(traffic);
    }
    if (status[traffic].status == RANGE_CNF_WAIT) {// I'm checking the range
      if (debug_)
         std::cout << "UwMultiTrafficRangeCtr::getBestLowerLayer(" << traffic 
                   << ") status == RANGE_CNF_WAIT" << std::endl;
      return 0;
    }
    BehaviorMap temp = it->second;
    BehaviorMap::iterator it_b = temp.begin();
    for (; it_b!=temp.end(); ++it_b)
    {
      switch (BehaviorItem(it_b->second).second)
      {
        case(CHECK_RANGE):
        {
          if (debug_)
            std::cout << "UwMultiTrafficRangeCtr::getBestLowerLayer(" << traffic << "): CHECK_RANGE" << std::endl;
          checkRange(traffic, BehaviorItem(it_b->second).first, HDR_UWIP(p)->daddr());
          break;
        }
        case(ROBUST):
        {
          if (debug_)
            std::cout << "UwMultiTrafficRangeCtr::getBestLowerLayer(" << traffic << "):  ROBUST" << std::endl;
          status[traffic].robust_id = BehaviorItem(it_b->second).first;
          break;
        }
        default:
        {
          std::cout << "UwMultiTrafficRangeCtr:: DEFAULT NOT ALLOWED" << std::endl;
          break;
        }
      }
    }
    if (status[traffic].status == IDLE && status[traffic].robust_id)
      return status[traffic].robust_id;
  }
  return 0;
}

void UwMultiTrafficRangeCtr::checkRange(int traffic, int module_id, uint8_t destAdd) 
{
  StatusMap::iterator it_s = status.find(traffic);
  if (it_s == status.end()) {
    initStatus(traffic);
  }
  if(status[traffic].status == RANGE_CNF_WAIT){//already checking 
    std::cout << "ALREADY CHECKING" << endl;
    return;
  }
  status[traffic].status = RANGE_CNF_WAIT;
  status[traffic].module_id = module_id;
  //TODO_1: check_range via CHECK packet
  Packet* p = Packet::alloc();
  hdr_cmn* ch = hdr_cmn::access(p);
  ch->ptype() = PT_MUTLI_TR_PROBE;
  ch->size() = signaling_pktSize;
  hdr_uwm_tr* tr = HDR_UWMTR(p);
  tr->traffic() = traffic;
  hdr_uwip* iph  = HDR_UWIP(p);
  //TODO: retreive my addr
  UWIPClMsgSendAddr msg;
  sendSyncClMsgUp(&msg);
  iph->saddr() =  msg.getAddr();
  iph->daddr() = destAdd;
  sendDown(module_id, p);

  if (debug_)
    std::cout << "UwMultiTrafficRangeCtr(" << (int)iph->saddr() << ")::checkRange " 
              << "sending PT_MUTLI_TR_PROBE to " << (int) iph->saddr() << std::endl;

  //Taking care about timer
  std::map<int, UwCheckRangeTimer*>::iterator it_t = timers.find(traffic);
  if (it_t == timers.end()) {
    UwCheckRangeTimer *t_o = new UwCheckRangeTimer(this,traffic);
    t_o->resched(check_to_period * (t_o->num_expires + 1));
    timers.insert(std::pair<int,UwCheckRangeTimer*>(traffic, t_o));
    //std::cout << NOW <<" RESCHEDULE A NEW ONE IN " << check_to_period << "*" << t_o->num_expires + 1 << endl;
  }
  else{
    it_t->second->resched(check_to_period * (it_t->second->num_expires + 1));
    //std::cout << "RESCHEDULE" << endl;
  }
}

void UwMultiTrafficRangeCtr::timerExpired(int traffic) 
{
  if(debug_)
    std::cout << NOW << "UwMultiTrafficRangeCtr::timerExpired(" << traffic << ")" << std::endl;
  StatusMap::iterator it_s = status.find(traffic);
  if (it_s == status.end())
    return;
  if (status[traffic].status != RANGE_CNF_WAIT)
    return;
  if (status[traffic].robust_id)
  {
    std::map<int, UwCheckRangeTimer*>::iterator it_t = timers.find(status[traffic].module_id);
    if (it_t != timers.end()) {
      it_t->second->num_expires = 0;
    }
    sendDown(status[traffic].robust_id,removeFromBuffer(traffic));
    status[traffic].status = IDLE;
  }
  else 
  {
    status[traffic].status = IDLE;
    checkRange(traffic, status[traffic].module_id, UWIP_BROADCAST);
  }
}

void UwMultiTrafficRangeCtr::initStatus(int traffic)
{
  check_status default_st;
  default_st.status = IDLE;
  default_st.module_id = 0;
  default_st.robust_id = 0;
  status[traffic] = default_st;
}