//
// Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Padova (SIGNET lab) nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.
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
* @file uwrovctr-module.cc
* @author Filippo Campagnaro
* @version 1.0.0
*
* \brief Provides the <i>UWROVCtr</i> class implementation.
*
* Provides the <i>UWROVCtr</i> class implementation.
*/
#include "uwrovctr-module.h"
#include <iostream>
#include <rng.h>
#include <stdint.h>
extern packet_t PT_UWCBR;

/**
* Adds the module for UwROVModuleClass in ns2.
*/

static class UwROVCtrModuleClass : public TclClass {
public:
	UwROVCtrModuleClass() : TclClass("Module/UW/ROV/CTR") {
	}
	TclObject* create(int, const char*const*) {
		return (new UwROVCtrModule());
	}
} class_module_uwROV_ctr;

UwROVCtrModule::UwROVCtrModule(Position p) : UwCbrModule(), sn(0) {
	posit=p;speed=1;
}
UwROVCtrModule::UwROVCtrModule() : UwCbrModule(), sn(0) {
	posit = Position();
	speed = 1;
}

UwROVCtrModule::~UwROVCtrModule() {}

int UwROVCtrModule::command(int argc, const char*const* argv) {
	Tcl& tcl = Tcl::instance();
	if(argc == 2){
		if (strcasecmp(argv[1], "getROVMonheadersize") == 0) {
			tcl.resultf("%d", this->getROVMonHeaderSize());
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "getROVctrheadersize") == 0) {
			tcl.resultf("%d", this->getROVCTRHeaderSize());
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "getX") == 0) {
			tcl.resultf("%f", posit.getX());
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "getY") == 0) {
			tcl.resultf("%f", posit.getY());
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "getZ") == 0) {
			tcl.resultf("%f", posit.getZ());
			return TCL_OK;
		}
	}
	else if(argc == 3){
		if (strcasecmp(argv[1], "setPosition") == 0) {
			Position* p = dynamic_cast<Position*> (tcl.lookup(argv[2]));
			posit = *p;
			return TCL_OK;
		} else if (strcasecmp(argv[1], "setSpeed") == 0) {
			speed = atof(argv[2]);
			return TCL_OK;
		}
	}
	else if(argc == 5){
		if (strcasecmp(argv[1], "sendPosition") == 0) {
			newX = atof(argv[2]);
			newY = atof(argv[3]);
			newZ = atof(argv[4]);
			this->reset_retx();
			this->transmit();
			tcl.resultf("%s", "position Setted");
			return TCL_OK;
		}
	}else if(argc == 6){
		if (strcasecmp(argv[1], "sendPosition") == 0) {
			newX = atof(argv[2]);
			newY = atof(argv[3]);
			newZ = atof(argv[4]);
			speed = atof(argv[5]);
			this->reset_retx();
			this->transmit();
			tcl.resultf("%s", "position Setted");
			return TCL_OK;
		}
	}
	return UwCbrModule::command(argc,argv);
}

void UwROVCtrModule::transmit() {
	sendPkt();
	sendTmr_.resched(period_);
}

void UwROVCtrModule::start() {}

void UwROVCtrModule::setPosition(Position p){
	posit = p;
}

Position UwROVCtrModule::getPosition(){
	return posit;
}

void UwROVCtrModule::initPkt(Packet* p) {
	if(this->p == NULL){
		hdr_uwROV_ctr* uwROVh = HDR_UWROV_CTR(p);
		uwROVh -> x() = newX;
		uwROVh->y() = newY;
		uwROVh->z() = newZ;
		uwROVh->speed() = speed;
		uwROVh->sn() = ++sn;
		this->p = p;
	}
	else{
		hdr_uwROV_ctr* uwROVh = HDR_UWROV_CTR(p);
		uwROVh->x() = newX;
		uwROVh->y() = newY;
		uwROVh->z() = newZ;
		uwROVh->speed() = speed;
		uwROVh->sn() = sn;
		if (debug_ > 10){ 
			printf("RITRASMETTO \n");
		}
	}
	UwCbrModule::initPkt(p);
	if (debug_ > 10){
		hdr_uwROV_ctr* uwROVh = HDR_UWROV_CTR(p);
		printf("CTR set new ROV position: X = %f, Y = %f, Z = %f\n", uwROVh->x(), uwROVh->y(), uwROVh->z());
	}
	//buffer.push(p);
}
void UwROVCtrModule::recv(Packet* p, Handler* h) {
	// hdr_cmn* ch = hdr_cmn::access(p);
	recv(p);
}
void UwROVCtrModule::recv(Packet* p) {
	hdr_uwROV_monitoring* monitoring = HDR_UWROV_MONITORING(p);
	x_rov = monitoring->x();
	y_rov = monitoring->y();
	z_rov = monitoring->z();

	if(monitoring->ack()>0){
		sendTmr_.force_cancel();
		this->p = NULL;
		if (debug_ > 10)
			printf("Ack ok \n");
	}
	if((monitoring->ack())<0 && debug_ > 10)
		printf("Errore \n");
	if (debug_ > 10)
		printf("ROV CTR monitoring position: X = %f, Y = %f, Z = %f\n", x_rov,y_rov,z_rov);
	UwCbrModule::recv(p);
}
