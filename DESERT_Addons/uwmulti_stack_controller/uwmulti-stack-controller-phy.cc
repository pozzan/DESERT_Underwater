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
 * @file   uwmulti-stack-controller-phy.cc
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Implementation of UwMultiStackControllerPhy class.
 *
 */

#include "uwmulti-stack-controller-phy.h"
#include "phymac-clmsg.h"

static class UwMultiStackControllerPhyClass : public TclClass {
public:
    UwMultiStackControllerPhyClass() : TclClass("Module/UW/MULTI_STACK_CONTROLLER_PHY") {}
    TclObject* create(int, const char*const*) {
        return (new UwMultiStackControllerPhy);
    }
} class_uwmulti_stack_controller_phy;

map< UwMultiStackControllerPhy::UWPHY_CONTROLLER_STATE, string> UwMultiStackControllerPhy::state_info;

UwMultiStackControllerPhy::UwMultiStackControllerPhy() 
: 
UwMultiStackController(),
receiving_id(0),
current_state(UWPHY_CONTROLLER_STATE_IDLE)
{
    initInfo(); 
}

void UwMultiStackControllerPhy::initInfo() {

    state_info[UWPHY_CONTROLLER_STATE_IDLE] = "STATE_IDLE";
    state_info[UWPHY_CONTROLLER_STATE_BUSY] = "STATE_BUSY";
}

int UwMultiStackControllerPhy::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();
    if (argc == 2) {
		if(strcasecmp(argv[1], "getStatus") == 0){
            tcl.resultf("%d", (int)(current_state));
			return TCL_OK;
		}
	}
    return UwMultiStackController::command(argc, argv);     
} /* UwMultiStackControllerPhy::command */

int UwMultiStackControllerPhy::recvSyncClMsg(ClMessage* m)
{
    if (m->direction() == DOWN)//mac2phy
    {
        m->setDest(lower_id_active_);
        sendSyncClMsgDown(m);
        return 0;
    }
    else if (m->type() == CLMSG_PHY2MAC_STARTRX && current_state == UWPHY_CONTROLLER_STATE_IDLE)
    {
        stateBusy(m->getSource());
        return 0;
    }
    else{
        sendSyncClMsgUp(m);
        return 0;
    }
}

void UwMultiStackControllerPhy::stateIdle(){
    current_state = UWPHY_CONTROLLER_STATE_IDLE;
}

void UwMultiStackControllerPhy::stateBusy(int id){
    current_state = UWPHY_CONTROLLER_STATE_BUSY;
    receiving_id=id;
}

void UwMultiStackControllerPhy::recv(Packet *p, int idSrc){
    hdr_cmn *ch = HDR_CMN(p);
    if(ch->direction() == hdr_cmn::UP && current_state == UWPHY_CONTROLLER_STATE_BUSY && 
        idSrc == receiving_id)
    {
        sendUp(p, min_delay_);
        stateIdle();
    }
    else
    {
        //direction DOWN: packet is coming from upper layers
        recvFromUpperLayers(p);
    }
}