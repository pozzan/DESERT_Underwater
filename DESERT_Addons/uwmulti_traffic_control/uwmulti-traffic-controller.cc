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
 * @file   uwmulti-stack-controller.cc
 * @author Filippo Campagnaro, Federico Guerra
 * @version 1.0.0
 *
 * \brief Implementation of UwMultiTrafficController class.
 *
 */

#include "uwmulti-traffic-controller.h"
#include <clmsg-discovery.h>

/**
 * Class that represents the binding with the tcl configuration script 
 */
static class UwMultiTrafficControllerClass : public TclClass 
{
public:
  /**
   * Constructor of the class
   */
  UwMultiTrafficControllerClass() : TclClass("Module/UW/MULTI_TRAFFIC_CONTROLLER") {}
  /**
   * Creates the TCL object needed for the tcl language interpretation
   * @return Pointer to an TclObject
   */
  TclObject* create(int, const char*const*) 
  {
    return (new UwMultiTrafficController);
  }
} class_stack_controller;

UwMultiTrafficController::UwMultiTrafficController() 
: 
  Module(),
  debug_(0),
  status(IDLE),
  up_map(),
  down_map(),
  down_buffer()
{
	bind("debug_", &debug_);
}

int UwMultiTrafficController::command(int argc, const char*const* argv) 
{
	if (argc == 4) 
  {
    /**
     * parameters: layer_id, layer_order, a positive and
     * unique integer to order the set if physical ids
    */
		if(strcasecmp(argv[1], "addUpLayer") == 0)
    {
      addUpLayerFromName(atoi(argv[2]),argv[3]);
			return TCL_OK;
		}
    else if(strcasecmp(argv[1], "addRobustLowLayer") == 0)
    {
      addLowLayerFromName(atoi(argv[2]),argv[3],ROBUST);
      return TCL_OK;
    }
    else if(strcasecmp(argv[1], "addFastLowLayer") == 0)
    {
      addLowLayerFromName(atoi(argv[2]),argv[3],CHECK_RANGE);
      return TCL_OK;
    }
	}
  else if (argc == 4) 
  {
    if(strcasecmp(argv[1], "addLowLayer") == 0)
    {
      addLowLayerFromName(atoi(argv[2]),argv[3],atoi(argv[4]));
      return TCL_OK;
    }
  }
	
  return Module::command(argc, argv);     
} /* UwMultiTrafficController::command */

void UwMultiTrafficController::addUpLayerFromName(int traffic_id, std::string tcl_name) {
  ClMsgDiscovery m;
  m.addSenderData((const PlugIn*) this, getLayer(), getId(), getStackId(), name() , getTag());
  sendSyncClMsgUp(&m);
  DiscoveryStorage up_layer_storage = m.findTclName(tcl_name.c_str());
  if (up_layer_storage.getSize() == 1) 
  {
    insertTraffic2UpLayer(traffic_id,(*up_layer_storage.begin()).first);
  }  
}

void UwMultiTrafficController::addLowLayerFromName(int traffic_id, std::string tcl_name, int behavior) {
  assert(behavior==ROBUST || behavior==CHECK_RANGE);
  ClMsgDiscovery m;
  m.addSenderData((const PlugIn*) this, getLayer(), getId(), getStackId(), name() , getTag());
  sendSyncClMsgDown(&m);
  DiscoveryStorage low_layer_storage = m.findTclName(tcl_name.c_str());
  DiscoveryData low_layer = (*low_layer_storage.begin()).second;
  if (low_layer_storage.getSize() == 1) 
  {
    insertTraffic2LowerLayer(traffic_id,low_layer.getId(),low_layer.getStackId(),behavior);
  }  
}

void UwMultiTrafficController::recv(Packet* p)
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

void UwMultiTrafficController::recvFromUpperLayers(Packet *p)
{
  hdr_cmn *ch = HDR_CMN(p);
  assert(ch->direction() == hdr_cmn::DOWN);

  hdr_uwcbr *ah = HDR_UWCBR(p);
  // TODO: wait cbr cointains traffic type
  int app_type = 0; // ah.getTipe();

  //TODO:queue push
  insertInBuffer(p,app_type);
  //TODO:queue manager
  manageBuffer(app_type);
}

void UwMultiTrafficController::insertInBuffer(Packet *p, int traffic) 
{
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end()) {
    it->second.push(p);
  }
  else {
    std::queue<Packet*> *q = new std::queue<Packet*>;
    q->push(p);
    down_buffer[traffic] = *q;
  }
}

void UwMultiTrafficController::manageBuffer(int traffic)
{
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end()) {
    int l_id = getBestLowerLayer(traffic);
    if (status == IDLE) {
      sendDown(l_id,removeFromBuffer(traffic));
    }
  }
}

Packet * UwMultiTrafficController::removeFromBuffer(int traffic) 
{
  Packet * p = NULL;
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end()) {
    p = it->second.front();
    it->second.pop();
  }
  return p;
}

  
int UwMultiTrafficController::getBestLowerLayer(int traffic) 
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

int UwMultiTrafficController::getUpperLayer(int traffic) 
{
  UpTrafficMap::iterator it = up_map.find(traffic); 
  if (it != up_map.end()) {
    return it->second;
  }
  return 0;
}

void UwMultiTrafficController::eraseTraffic2LowerLayer(int traffic, int lower_layer_stack)
{
  DownTrafficMap::iterator it = down_map.find(traffic); 
  if (it != down_map.end()) {
    BehaviorMap behav = it->second;
    BehaviorMap::iterator it_layer = behav.find(lower_layer_stack);
    if(it_layer != behav.end())
      behav.erase(lower_layer_stack);
    if(behav.size() == 0)
      down_map.erase(traffic);
  }
}

void UwMultiTrafficController::eraseTraffic2Low(int traffic)
{
  UpTrafficMap::iterator it = up_map.find(traffic); 
  if (it != up_map.end()) {
    up_map.erase(traffic);
  }
}

void UwMultiTrafficController::eraseTraffic2Up(int traffic)
{
  UpTrafficMap::iterator it = up_map.find(traffic); 
  if (it != up_map.end()) {
    up_map.erase(traffic);
  }
}