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
 * @file   uwmulti-stack-Control.cc
 * @author Filippo Campagnaro, Federico Guerra
 * @version 1.0.0
 *
 * \brief Implementation of UwMultiTrafficControl class.
 *
 */

#include "uwmulti-traffic-range-crt.h"

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
} class_stack_Control;

UwMultiTrafficRangeCtr::UwMultiTrafficRangeCtr() 
: 
  UwMultiTrafficControl(),
  check_to(0),
  status(),
  timers()
{ 
  bind("check_to_", &check_to);
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

void UwMultiTrafficRangeCtr::manageBuffer(int traffic)
{
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end()) {
    int l_id = getBestLowerLayer(traffic);
    if (status[traffic].status == IDLE) {
      sendDown(l_id,removeFromBuffer(traffic));
    }
  }
}
  
int UwMultiTrafficRangeCtr::getBestLowerLayer(int traffic) 
{
  DownTrafficMap::iterator it = down_map.find(traffic); 
  if (it != down_map.end()) {
    StatusMap::iterator it_s = status.find(traffic);
    if (it_s == status.end()) {
      initStatus(traffic);
    }
    if (status[traffic].status == RANGE_CNF_WAIT) // I'm checking the range
      return 0;
    BehaviorMap temp = it->second;
    BehaviorMap::iterator it_b = temp.begin();
    for (; it_b!=temp.end(); ++it_b)
    {
      switch (BehaviorItem(it_b->second).second == CHECK_RANGE)
      {
        case(CHECK_RANGE):
        {
          checkRange(traffic, it_b->first,BehaviorItem(it_b->second).first);
          break;
        }
        case(ROBUST):
        {
          status[traffic].robust_id = BehaviorItem(it_b->second).first;
          break;
        }
      }
    }
    if (status[traffic].status == IDLE && status[traffic].robust_id)
      return status[traffic].robust_id;
  }
  return 0;
}

void UwMultiTrafficRangeCtr::checkRange(int traffic, int stack_id, int module_id) 
{
  StatusMap::iterator it_s = status.find(traffic);
  if (it_s == status.end()) {
    initStatus(traffic);
  }
  if(status[traffic].status == RANGE_CNF_WAIT)//already checking
    return;
  status[traffic].status = RANGE_CNF_WAIT;
  status[traffic].stack_id = stack_id;
  status[traffic].module_id = module_id;
  //TODO_1: check_range with fgue via clmesgs

  //TODO_2: take care about timer
  std::map<int, UwCheckRangeTimer>::iterator it_t = timers.find(stack_id);
  if (it_t == timers.end()) {
    //initTimer(stack_id);
    UwCheckRangeTimer *t_o = new UwCheckRangeTimer(this,traffic);
    timers.insert(std::pair<int,UwCheckRangeTimer>(stack_id, *t_o));
  }
  it_t->second.resched(check_to * (it_t->second.num_expires + 1));
}

void UwMultiTrafficRangeCtr::timerExpired(int traffic) 
{
  if (status[traffic].status != RANGE_CNF_WAIT)
    return;
  if (status[traffic].robust_id)
  {
    std::map<int, UwCheckRangeTimer>::iterator it_t = timers.find(stack_id);
    if (it_t != timers.end()) {
      it_t->second.num_expires = 0;
    }
    sendDown(status[traffic].robust_id,removeFromBuffer(traffic));
  }
  else 
  {
    checkRange(traffic, status[traffic].stack_id, status[traffic].module_id);
  }
}

void UwMultiTrafficRangeCtr::initStatus(int traffic)
{
  stack_status default_st;
  default_st.status = IDLE;
  default_st.stack_id = 0;
  default_st.module_id = 0;
  default_st.robust_id = 0;
  status[traffic] = default_st;
}