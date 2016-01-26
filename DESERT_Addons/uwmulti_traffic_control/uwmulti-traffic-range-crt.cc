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

#include "uwmulti-traffic-control.h"

/**
 * Class that represents the binding with the tcl configuration script 
 */
static class UwMultiTrafficControlClass : public TclClass 
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
  UwMultiTrafficControl()
{ 

}

int UwMultiTrafficRangeCtr::command(int argc, const char*const* argv) 
{
	if (argc == 4) 
  {
    /**
     * parameters: layer_id, layer_order, a positive and
     * unique integer to order the set if physical ids
    */
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

void UwMultiTrafficControl::recv(Packet* p)
{
  hdr_cmn *ch = HDR_CMN(p);
  if(ch->direction() == hdr_cmn::UP)
  {   
    hdr_uwcbr *ah = HDR_UWCBR(p);
    // TODO: wait cbr cointains traffic type
    int app_type = 0; // ah.getTipe();
    sendUp(getUpperLayer(app_type), p);
  }
  else
  {
    //direction DOWN: packet is coming from upper layers
    recvFromUpperLayers(p);
  }
}

void UwMultiTrafficRangeCtr::manageBuffer(int traffic)
{
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end()) {
    int l_id = getBestLowerLayer(traffic);
    if (status == IDLE) {
      sendDown(l_id,removeFromBuffer(traffic));
    }
  }
}
  
int UwMultiTrafficRangeCtr::getBestLowerLayer(int traffic) 
{
  DownTrafficMap::iterator it = down_map.find(traffic); 
  if (it != down_map.end()) {
    BehaviorMap temp = it->second;
    BehaviorMap::iterator it_b = temp.begin();
    for (; it_b!=temp.end(); ++it_b)
    {
      if (BehaviorItem(it_b->second).second == CHECK_RANGE)
      {
        //TODO: check_range, status = RANGE_CNF_WAIT
      }
    }
    if (status == IDLE)
      return (--it_b)->first;
  }
  return 0;
}
