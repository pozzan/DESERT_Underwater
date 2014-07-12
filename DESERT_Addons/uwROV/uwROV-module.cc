//
// Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
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
//

/**
 * @file   uwROV-module.cc
 * @author Filippo Campagnaro
 * @version 1.1.0
 * 
 * \brief Provides the <i>UWROV</i> class implementation.
 * 
 * Provides the <i>UWROV</i> class implementation.
 */

#include "uwROV-module.h"

#include <iostream>
#include <rng.h>
#include <stdint.h>

extern packet_t PT_UWCBR;

int hdr_uwROV::offset_;         /**< Offset used to access in <i>hdr_uwROV</i> packets header. */

/**
 * Adds the header for <i>hdr_uwROV</i> packets in ns2.
 */
static class UwROVPktClass : public PacketHeaderClass {
public:

    UwROVPktClass() : PacketHeaderClass("PacketHeader/UWROV", sizeof (hdr_uwROV)) {
        this->bind();
        bind_offset(&hdr_uwROV::offset_);
    }
} class_uwROV_pkt;


/**
 * Adds the module for UwROVModuleClass in ns2.
 */
static class UwROVModuleClass : public TclClass {
public:

    UwROVModuleClass() : TclClass("Module/UW/ROV") {
    }

    TclObject* create(int, const char*const*) {
        return (new UwROVModule());
    }
} class_module_uwROV;



UwROVModule::UwROVModule() : UwCbrModule() {
        //posit = UwGMPosition();
	posit = Position();
}
UwROVModule::~UwROVModule() {
}


int UwROVModule::command(int argc, const char*const* argv) {
	Tcl& tcl = Tcl::instance();
	if(argc == 2){
		if (strcasecmp(argv[1], "getROVheadersize") == 0) {
            tcl.resultf("%d", this->getROVHeaderSize());
            return TCL_OK;
        } 
	}
	return UwCbrModule::command(argc,argv);
}

void UwROVModule::recv(Packet* p, Handler* h) {
//    hdr_cmn* ch = hdr_cmn::access(p);
    recv(p);
}

void UwROVModule::recv(Packet* p) {

    hdr_uwROV* uwROVh = HDR_UWROV(p);
    posit.setX(uwROVh->x());
    posit.setY(uwROVh->y());
    posit.setZ(uwROVh->z());
    //printf("ROV new position: X = %d, Y = %d, Z  = %d\n", uwROVh->x(), uwROVh->y(), uwROVh->z());
    UwCbrModule::recv(p);
}
