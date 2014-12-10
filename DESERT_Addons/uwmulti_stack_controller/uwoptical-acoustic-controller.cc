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
 * @file   uwoptical-acoustic-controller.cc
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Implementation of UwOpticalAcousticController class.
 *
 */

#include "uwoptical-acoustic-controller.h"

static class UwOpticalAcousticControllerClass : public TclClass {
public:
    UwOpticalAcousticControllerClass() : TclClass("Module/UW/OPTICAL_ACOUSTIC_CONTROLLER") {}
    TclObject* create(int, const char*const*) {
        return (new UwOpticalAcousticController);
    }
} class_optical_acoustic_controller;

UwOpticalAcousticController::UwOpticalAcousticController() 
: 
UwMultiStackController(),
optical_on_(false),
acoustic_name_("acoustic"),
optical_name_("optical")/*,
acoustic_id_(0),
optical_id_(0)
*/
{ }

int UwOpticalAcousticController::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();
    if (argc == 5) {
		if(strcasecmp(argv[1], "setOpticalLayer") == 0){
     		setOpticalLayer(atoi(argv[2]),atof(argv[3]),atof(argv[4]));
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setAcousticLayer") == 0){
     		setAcousticLayer(atoi(argv[2]),atof(argv[3]),atof(argv[4]));
			return TCL_OK;
		}
	}
    return UwMultiStackController::command(argc, argv);     
} /* UwOpticalAcousticController::command */

void UwOpticalAcousticController::recv(Packet* p)
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

void UwOpticalAcousticController::recvFromUpperLayers(Packet *p)
{
	hdr_cmn *ch = HDR_CMN(p);

	if(switch_mode_ == UW_AUTOMATIC_SWITCH && ch->ptype() == CONTROLLED)
		sendDown( getBestLayer(p), p, min_delay_);
	else 
		sendDown(manual_lower_id_, p, min_delay_);
}

int UwOpticalAcousticController::getBestLayer(Packet *p){
	if (isOpticalAvailable(p)){
		optical_on_=true;
		return 0;//optical_id_;
	}
	else{
		optical_on_=false;
		return 0;//acoustic_id_;//at least send in acoustic
	}
	
}

bool UwOpticalAcousticController::isOpticalAvailable(Packet *p){
	if(!isLayerAvailable(optical_name_))
		return false;

	//TODO: check (??by ClMessage or using past packet heder???) the status of lower layers and get the metric.
  	/*ClMsgController m(optical_id_, p);
 	sendSyncClMsgDown(&m);*/
	//TODO: update the statistics with the new metrics (avarage of the last N samples?)
	/*TODO: choose the best layer checking the statistics bound in the two cases:
	 *	if(optical_on) { return (optical_statistic > optical_power+optical_hysteresis/2);}
	 *  else {return (acoustic_statistic > acoustic_power+acoustic_hysteresis/2);}
	*/
 	if(optical_on_)
 		return true;//return(m.getMetrics()>optical_minimal_target_ - optical_hysteresis_size_/2);
 	else
 		return false;//return(m.getMetrics()>acoustic_maximum_target_ + acoustic_hysteresis_size_/2);
}

void UwOpticalAcousticController::setOpticalLayer(int id, double target, double hysteresis){
	UwMultiStackController::addLayer(id,optical_name_,target,hysteresis);
	//optical_id_ = id;
}

void UwOpticalAcousticController::setAcousticLayer(int id, double target, double hysteresis){
	UwMultiStackController::addLayer(id,acoustic_name_,target,hysteresis);
	//acoustic_id_=id;
}