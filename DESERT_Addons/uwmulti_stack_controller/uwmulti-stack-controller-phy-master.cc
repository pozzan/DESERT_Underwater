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

/*const double UwMultiStackControllerPhyMaster::min_value = -numeric_limits<double>::max();//-10*pow(10.0,100);
const double UwMultiStackControllerPhyMaster::max_value = numeric_limits<double>::max();//10*pow(10.0,100);*/

UwMultiStackControllerPhyMaster::UwMultiStackControllerPhyMaster() 
: 
  UwMultiStackControllerPhy(),
  last_layer_used_(0),
  power_statistics_(0),
  alpha_(0.5)
{ 
  bind("alpha_", &alpha_);
}

int UwMultiStackControllerPhyMaster::command(int argc, const char*const* argv) 
{
  if (argc == 3) 
  {
    if(strcasecmp(argv[1], "setAlpha") == 0)
    {
      alpha_ = atof(argv[2]);
      return TCL_OK;
    }
		else if(strcasecmp(argv[1], "setManualLowerlId") == 0)
    {
      lower_id_active_ = atoi(argv[2]);
      last_layer_used_ = lower_id_active_;
			return TCL_OK;
		}
  }
  return UwMultiStackControllerPhy::command(argc, argv);     
} /* UwMultiStackControllerPhyMaster::command */

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
    std::cout << NOW << " ControllerPhyMaster("<< mac_addr 
              <<")::getBestLayer(Packet *p), power_statistics_=" 
              << power_statistics_ << std::endl;
  }

  int id_short_range = getShorterRangeLayer(last_layer_used_);
  int id_long_range = getLongerRangeLayer(last_layer_used_);
  double upper_threshold= getThreshold(last_layer_used_,id_short_range);
  double lower_threshold = getThreshold(last_layer_used_,id_long_range);

  last_layer_used_ = (upper_threshold != UwMultiStackController::threshold_not_exist 
                      && power_statistics_ > upper_threshold) ?
                      id_short_range : last_layer_used_;

  last_layer_used_ = (lower_threshold != UwMultiStackController::threshold_not_exist 
                      && power_statistics_ < lower_threshold) ?
                      id_long_range : last_layer_used_;
  power_statistics_ = 0;
  lower_id_active_ = last_layer_used_;
  std::cout << last_layer_used_<<std::endl;
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
    std::cout << NOW << " ControllerPhyMaster("<< mac_addr <<
              ")::updateMasterStatistics(Packet *p, int idSrc), Pr = " 
              << ph->Pr << std::endl;
  }
  
  if (mach->macDA() == mac_addr && idSrc == last_layer_used_)
  {
    power_statistics_ = power_statistics_ ? (1-alpha_)*power_statistics_ + alpha_*ph->Pr 
                        : ph->Pr;
  }
}

