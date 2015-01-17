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
 * @file   uwmulti-stack-controller-phy-master.cc
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Implementation of UwMultiStackControllerPhyMaster class.
 *
 */

#include "uwmulti-stack-controller-phy-master.h"

#include <mphy_pktheader.h>

static class UwMultiStackControllerPhyMasterClass : public TclClass 
{
  public:
  UwMultiStackControllerPhyMasterClass() : TclClass("Module/UW/MULTI_STACK_CONTROLLER_PHY_MASTER") {}
  TclObject* create(int, const char*const*) 
  {
    return (new UwMultiStackControllerPhyMaster);
  }
} class_uwmulti_stack_controller_phy_master;

UwMultiStackControllerPhyMaster::UwMultiStackControllerPhyMaster() 
: 
  UwMultiStackControllerPhy(),
  last_layer_used_(0),
  short_range_layer_(0),
  default_layer_(0),
  power_statistics_(0),
  alpha_(0.5)
{ 
  bind("alpha_", &alpha_);
}

int UwMultiStackControllerPhyMaster::command(int argc, const char*const* argv) 
{
  Tcl& tcl = Tcl::instance();
  if (argc == 3) 
  {
    if(strcasecmp(argv[1], "setAlpha") == 0)
    {
      alpha_ = atof(argv[2]);
      return TCL_OK;
    }
    else if(strcasecmp(argv[1], "setDefaultId") == 0)
    {
      default_layer_ = atoi(argv[2]);
      last_layer_used_ = default_layer_;
      return TCL_OK;
    }
    else if(strcasecmp(argv[1], "setShortRangeId") == 0)
    {
      short_range_layer_ = atoi(argv[2]);
      return TCL_OK;
    }
  }
  return UwMultiStackControllerPhy::command(argc, argv);     
} /* UwMultiStackControllerPhyMaster::command */

void UwMultiStackControllerPhyMaster::addLayer(int id, const string& layer_name, double target, double hysteresis)
{
  layer_map.erase(id);
  
  if (layer_map.size() < 2)
  {
    UwMultiStackController::addLayer(id, layer_name, target, hysteresis);
  }
}

void UwMultiStackControllerPhyMaster::recv(Packet *p, int idSrc)
{
  updateMasterStatistics(p,idSrc);
  UwMultiStackControllerPhy::recv(p, idSrc);
}

int UwMultiStackControllerPhyMaster::getBestLayer(Packet *p) 
{
  //TODO: define if doing it directly for doubble physical or in a more general way.
  assert(switch_mode_ == UW_AUTOMATIC_SWITCH);

  int mac_addr = -1;
  ClMsgPhy2MacAddr msg;
  sendSyncClMsg(&msg);
  mac_addr = msg.getAddr();

  if (debug_)
  {
    std::cout << NOW << " ControllerPhyMaster("<< mac_addr <<")::getBestLayer(Packet *p), power_statistics_=" 
              << power_statistics_ << std::endl;
  }
  
  Stats info = layer_map.find(last_layer_used_)->second;
  
  if(last_layer_used_ == default_layer_)
  {
    last_layer_used_ = (power_statistics_ > info.metrics_target_+info.hysteresis_size_/2) ? 
                        short_range_layer_ : last_layer_used_;
  }
  else
  {
    last_layer_used_ = (power_statistics_ < info.metrics_target_-info.hysteresis_size_/2) ? 
                        default_layer_ : last_layer_used_;
  }
  
  power_statistics_ = 0;
  lower_id_active_ = last_layer_used_;
  return last_layer_used_;
}

void UwMultiStackControllerPhyMaster::updateMasterStatistics(Packet *p, int idSrc)
{
  int mac_addr = -1;
  ClMsgPhy2MacAddr msg;
  sendSyncClMsg(&msg);
  mac_addr = msg.getAddr();

  hdr_mac* mach = HDR_MAC(p);
  hdr_MPhy* ph = HDR_MPHY(p);


  
  if (debug_)
  {
    std::cout << NOW << " ControllerPhyMaster("<< mac_addr <<")::updateMasterStatistics(Packet *p, int idSrc), Pr = " 
              << ph->Pr << std::endl;
  }
  
  if (mach->macDA() == mac_addr && idSrc == last_layer_used_)
  {
    power_statistics_ = power_statistics_ ? (1-alpha_)*power_statistics_ + alpha_*ph->Pr : ph->Pr;
  }
}

