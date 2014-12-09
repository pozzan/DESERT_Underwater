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
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Implementation of UwOptical class.
 *
 */

#include "uwmulti-stack-controller.h"

packet_t NOT_CONTROLLED;
packet_t CONTROLLED;

static class UwMultiStackControllerClass : public TclClass {
public:
    UwMultiStackControllerClass() : TclClass("Module/UW/MULTI_STACK_CONTROLLER") {}
    TclObject* create(int, const char*const*) {
        return (new UwMultiStackController);
    }
} class_module_optical;

UwMultiStackController::UwMultiStackController() 
: 
Module(),
debug_(0),
min_delay_(0),
switch_mode(UW_MANUAL_SWITCH),
manual_lower_id_(0),
optical_id_(0),
acoustic_id_(0),
optical_minimal_target_(0),
optical_hysteresis_size_(0),
acoustic_maximum_target_(0),
acoustic_hysteresis_size_(0),
optical_on_(false)
{
	bind("debug_", &debug_);
	bind("min_delay_", &min_delay_);
	bind("switch_mode", &switch_mode);
	bind("manual_lower_id_", &manual_lower_id_);
	bind("optical_id_", &optical_id_);
	bind("acoustic_id_", &acoustic_id_);
	bind("optical_minimal_target_", &optical_minimal_target_);
	bind("optical_hysteresis_size_", &optical_hysteresis_size_);
	bind("acoustic_maximum_target_", &acoustic_maximum_target_);
	bind("acoustic_hysteresis_size_", &acoustic_hysteresis_size_);
}

int UwMultiStackController::command(int argc, const char*const* argv) {
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
	}
	else if (argc == 3) {
		if(strcasecmp(argv[1], "setManualLowewlId") == 0){
     		manual_lower_id_ = atoi(argv[2]);
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setSwitchMode") == 0) {
     		switch_mode = atoi(argv[2]);
			return TCL_OK;
		}
	}
	else if (argc == 4) {
		if(strcasecmp(argv[1], "setOpticalId") == 0){
     		setOpticalId(atoi(argv[2]),(atof(argv[3])));
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setAcousticId") == 0){
     		setAcousticId(atoi(argv[2]),(atof(argv[3])));
			return TCL_OK;
		}
	}
    return Module::command(argc, argv);     
} /* UwMultiStackController::command */

void UwMultiStackController::recv(Packet* p)
{
 	hdr_cmn *ch = HDR_CMN(p);
  	if(ch->direction() == hdr_cmn::UP)
    {
      	sendUp(p, min_delay_);
    }
  	else
    {
      	//direction DOWN: packet is coming from upper layers
      	recvFromUpperLayers(p);
    }
}

void UwMultiStackController::recvFromUpperLayers(Packet *p)
{
	hdr_cmn *ch = HDR_CMN(p);

	if(switch_mode == UW_AUTOMATIC_SWITCH && ch->ptype() == CONTROLLED)
		sendDown(bestLowerLayer(p), p, min_delay_);
	else 
		sendDown(manual_lower_id_, p, min_delay_);
}

int UwMultiStackController::bestLowerLayer(Packet *p){
	if (opticalAvailable(p)){
		optical_on_=true;
		return optical_id_;
	}
	else{
		optical_on_=false;
		return acoustic_id_;//at least send in acoustic
	}
	
}

bool UwMultiStackController::opticalAvailable(Packet *p){
	if(!optical_id_)
		return false;
  	ClMsgController m(optical_id_, p);
 	sendSyncClMsgDown(&m);
 	if(optical_on_)
 		return(m.getMetrics()>optical_minimal_target_ - optical_hysteresis_size_/2);
 	else
 		return(m.getMetrics()>acoustic_maximum_target_ + acoustic_hysteresis_size_/2);
	//TODO: check via ClMessage the status of lower layers and choose the best one.
}

void UwMultiStackController::setOpticalId(int id, double minimalTarget){
	optical_id_ = id;
	optical_minimal_target_ = minimalTarget;
}

void UwMultiStackController::setAcousticId(int id, double minimalTarget){
	acoustic_id_ = id;
	acoustic_maximum_target_ = minimalTarget;
}