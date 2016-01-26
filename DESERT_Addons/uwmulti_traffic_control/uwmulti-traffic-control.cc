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
#include <clmsg-discovery.h>

/**
 * Class that represents the binding with the tcl configuration script 
 */
static class UwMultiTrafficControlClass : public TclClass 
{
public:
  /**
   * Constructor of the class
   */
  UwMultiTrafficControlClass() : TclClass("Module/UW/MULTI_TRAFFIC_CONTROL") {}
  /**
   * Creates the TCL object needed for the tcl language interpretation
   * @return Pointer to an TclObject
   */
  TclObject* create(int, const char*const*) 
  {
    return (new UwMultiTrafficControl);
  }
} class_stack_Control;

UwMultiTrafficControl::UwMultiTrafficControl() 
: 
  Module(),
  debug_(0),
  up_map(),
  down_map(),
  down_buffer()
{
	bind("debug_", &debug_);
}

int UwMultiTrafficControl::command(int argc, const char*const* argv) 
{
	if (argc == 4) 
  {
    /**
     * parameters: layer_id, layer_order, a positive and
     * unique integer to order the set if physical ids
    */
		if(strcasecmp(argv[1], "addUpLayer") == 0)
    {
      addUpLayerFromTag(atoi(argv[2]),argv[3]);
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "addLowLayer") == 0)
    {
      addLowLayerFromTag(atoi(argv[2]),argv[3],DEFAULT);
      return TCL_OK;
    }
	}
  else if (argc == 5) 
  {
    if(strcasecmp(argv[1], "addLowLayer") == 0)
    {
      addLowLayerFromTag(atoi(argv[2]),argv[3],atoi(argv[4]));
      return TCL_OK;
    }
  }
	
  return Module::command(argc, argv);     
} /* UwMultiTrafficControl::command */

void UwMultiTrafficControl::addUpLayerFromTag(int traffic_id, std::string tag) {
  ClMsgDiscovery m;
  m.addSenderData((const PlugIn*) this, getLayer(), getId(), getStackId(), name() , getTag());
  sendSyncClMsgUp(&m);
  DiscoveryStorage up_layer_storage = m.findTag(tag.c_str());
  if (up_layer_storage.getSize() == 1) 
  {
    insertTraffic2UpLayer(traffic_id,(*up_layer_storage.begin()).first);
  }  
}

void UwMultiTrafficControl::addLowLayerFromTag(int traffic_id, std::string tag, int behavior) {
  ClMsgDiscovery m;
  m.addSenderData((const PlugIn*) this, getLayer(), getId(), getStackId(), name() , getTag());
  sendSyncClMsgDown(&m);
  DiscoveryStorage low_layer_storage = m.findTag(tag.c_str());
  DiscoveryData low_layer = (*low_layer_storage.begin()).second;
  if (low_layer_storage.getSize() == 1) 
  {
    insertTraffic2LowerLayer(traffic_id,low_layer.getId(),low_layer.getStackId(),behavior);
  }  
}

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

void UwMultiTrafficControl::recvFromUpperLayers(Packet *p)
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

void UwMultiTrafficControl::insertInBuffer(Packet *p, int traffic) 
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

void UwMultiTrafficControl::manageBuffer(int traffic)
{
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end()) {
    sendDown(getBestLowerLayer(traffic),removeFromBuffer(traffic));
  }
}

Packet * UwMultiTrafficControl::removeFromBuffer(int traffic) 
{
  Packet * p = NULL;
  DownTrafficBuffer::iterator it = down_buffer.find(traffic);
  if (it != down_buffer.end()) {
    p = it->second.front();
    it->second.pop();
  }
  return p;
}

  
int UwMultiTrafficControl::getBestLowerLayer(int traffic) 
{
  DownTrafficMap::iterator it = down_map.find(traffic); 
  if (it != down_map.end()) {
    BehaviorMap temp = it->second;
    BehaviorMap::iterator it_b = temp.begin();
    for (; it_b!=temp.end(); ++it_b)
    {
      if (BehaviorItem(it_b->second).second == DEFAULT)
      {
        return it_b->first;
      }
    }
  }
  return 0;
}

int UwMultiTrafficControl::getUpperLayer(int traffic) 
{
  UpTrafficMap::iterator it = up_map.find(traffic); 
  if (it != up_map.end()) {
    return it->second;
  }
  return 0;
}

void UwMultiTrafficControl::eraseTraffic2LowerLayer(int traffic, int lower_layer_stack)
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

void UwMultiTrafficControl::eraseTraffic2Low(int traffic)
{
  UpTrafficMap::iterator it = up_map.find(traffic); 
  if (it != up_map.end()) {
    up_map.erase(traffic);
  }
}

void UwMultiTrafficControl::eraseTraffic2Up(int traffic)
{
  UpTrafficMap::iterator it = up_map.find(traffic); 
  if (it != up_map.end()) {
    up_map.erase(traffic);
  }
}