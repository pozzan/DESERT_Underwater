//
// Copyright (c) 2013 Regents of the SIGNET lab, University of Padova.
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
 * @file   uw-csma-aloha-triggered-auv.cc
 * @author Federico Favaro
 * @version 1.0.0
 *
 * \brief Provides the implementation of UwCsmaAloha_Triggered_AUV class
 *
 */



#include "uw-trigger-auv.h"
#include <mac.h>

using namespace std;

const double UwTrigger_AUV::prop_speed = 1500.0;
bool UwTrigger_AUV::initialized = false;

map< UwTrigger_AUV::UW_TRIGGER_AUV_STATUS, string> UwTrigger_AUV::status_info;
map< UwTrigger_AUV::UW_TRIGGER_AUV_REASON_STATUS, string> UwTrigger_AUV::reason_info;

/**
 * Class that represents the binding with the tcl configuration script 
 */
static class UwTriggerModuleClass : public TclClass {
public:

    /**
     * Constructor of the class
     */
    UwTriggerModuleClass() : TclClass("Module/UW/TRIGGER/AUV") {
    }

    /**
     * Creates the TCL object needed for the tcl language interpretation
     * @return Pointer to an TclObject
     */
    TclObject* create(int, const char*const*) {
        return (new UwTrigger_AUV());
    }
} class_module_uw_trigger_auv;

void UwTrigger_AUV::ReceiveTimer::expire(Event* e) {
    timer_status = UW_TRIGGER_AUV_EXPIRED;
    if (module->curr_state != UW_TRIGGER_AUV_STATE_TX_TRIGGER) {
        if (module->debug_) cout << NOW << " UwTrigger_AUV(" << module->addr << ") timer Receiving expire() current state ="
                << module->status_info[module->curr_state] << "; next state " << module->status_info[UW_TRIGGER_AUV_STATE_DISABLE_RX] << endl;
        module->stateDisableRx();

    }
}

UwTrigger_AUV::UwTrigger_AUV()
: print_transitions(false),
curr_state(UW_TRIGGER_AUV_STATE_IDLE),
prev_state(UW_TRIGGER_AUV_STATE_IDLE),
last_reason(UW_TRIGGER_AUV_REASON_NOT_SET),
receive_timer(this),
trigger_pkts_tx(0) {
    bind("debug_", (double*) &debug_);
    bind("TRIGGER_size_", (int*) & TRIGGER_size);
    bind("tx_timer_duration_", (double*) & tx_timer_duration);
}

UwTrigger_AUV::~UwTrigger_AUV() {

}

int UwTrigger_AUV::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();
    if (argc == 2) {
        if (strcasecmp(argv[1], "initialize") == 0) {
            if (initialized == false) initInfo();
            //if (print_transitions) fout.open("/tmp/UW_CSMA_ALOHA_TRIGGERED_AUV_state_transitione.txt", ios_base::app);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "printTransitions") == 0) {
            print_transitions = true;
            return TCL_OK;
        } else if (strcasecmp(argv[1], "sinkRun") == 0) {
            stateIdle();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getTriggerMessagesSent") == 0) {
            tcl.resultf("%d", getTriggerMsgSent());
        }

    }
    return MMac::command(argc, argv);
}

int UwTrigger_AUV::crLayCommand(ClMessage* m) {
    switch (m->type()) {
        default:
            return Module::crLayCommand(m);
    }
}

void UwTrigger_AUV::initInfo() {

    initialized = true;

    if ((print_transitions) && (system(NULL))) {
        if (!system("rm -f /tmp/UW_CSMA_ALOHA_TRIGGERED_AUV_debug_state_transition.txt")) {
            std::cerr << "Cannot rm /tmp/UW_CSMA_ALOHA_TRIGGERED_AUV_debug_state_transition.txt";
        }
        if (!system("touch /tmp/UW_CSMA_ALOHA_TRIGGERED_AUV_debug_state_transition.txt")) {
            std::cerr << "Cannot create /tmp/UW_CSMA_ALOHA_TRIGGERED_AUV_debug_state_transition.txt";
        }


    }
    status_info[UW_TRIGGER_AUV_STATE_IDLE] = "Idle state";
    status_info[UW_TRIGGER_AUV_STATE_NOT_SET] = "State not set";
    status_info[UW_TRIGGER_AUV_STATE_TX_TRIGGER] = "Transmitting TRIGGER to sensors";
    status_info[UW_TRIGGER_AUV_STATE_ENABLE_RX] = "Enabling reception of DATA";
    status_info[UW_TRIGGER_AUV_STATE_DISABLE_RX] = "Disabling reception of DATA";
    status_info[UW_TRIGGER_AUV_STATE_DATA_RX] = "Reception of DATA packet";


    reason_info[UW_TRIGGER_AUV_REASON_DATA_RX] = "DATA received";
    reason_info[UW_TRIGGER_AUV_REASON_START_RX] = "Start rx pkt";
    reason_info[UW_TRIGGER_AUV_REASON_PKT_NOT_FOR_ME] = "Received an erroneous pkt";
    reason_info[UW_TRIGGER_AUV_REASON_PKT_ERROR] = "Erroneous pkt";
    reason_info[UW_TRIGGER_AUV_REASON_NOT_SET] = "Reason NOT SET";
    reason_info[UW_TRIGGER_AUV_REASON_TX_TRIGGER] = "TRIGGER message has been transmitted";
}

void UwTrigger_AUV::recvFromUpperLayers(Packet* p) {
    //no receive data pkt from upper layers in AUV mode
}

void UwTrigger_AUV::stateEnableRx() {
    refreshState(UW_TRIGGER_AUV_STATE_ENABLE_RX);
    if (debug_) cout << NOW << "UwTrigger_AUV(" << addr << ") ----> Enabling Receive() timer_duration = " << tx_timer_duration << endl;
    receive_timer.resched(tx_timer_duration);
    receiving_state_active = true;
}

void UwTrigger_AUV::stateDisableRx() {
    refreshState(UW_TRIGGER_AUV_STATE_DISABLE_RX);
    if (debug_) cout << NOW << "UwTrigger_AUV(" << addr << ")----> Timer fired ----> Disabling Receving Data" << endl;
    receiving_state_active = false;
    stateIdle();
}

void UwTrigger_AUV::initPkt(Packet* p, int dest_addr) {
    hdr_cmn* ch = hdr_cmn::access(p);
    hdr_mac* mach = HDR_MAC(p);
    ch->ptype() = PT_MMAC_TRIGGER;
    ch->size() = TRIGGER_size;
    mach->macSA() = addr;
    mach->macDA() = dest_addr;
}

void UwTrigger_AUV::Mac2PhyStartTx(Packet* p) {
    MMac::Mac2PhyStartTx(p);
}

void UwTrigger_AUV::Phy2MacEndTx(const Packet* p) {
    if (debug_) cout << NOW << "  UwTrigger_AUV(" << addr << ")::Phy2MacEndTx() end tx TRIGGER packet" << endl;
    refreshReason(UW_TRIGGER_AUV_REASON_TX_TRIGGER);
    stateEnableRx();
}

void UwTrigger_AUV::Phy2MacStartRx(const Packet* p) {
    if (receiving_state_active) {
        //RxActive = true;
        refreshReason(UW_TRIGGER_AUV_REASON_START_RX);
        if (debug_) cout << NOW << "  UwTrigger_AUV(" << addr << ")::Phy2MacStartRx() rx Packet " << endl;
    } else {
        //RxActive = false;
        if (debug_) cout << NOW << " UwTrigger_AUV(" << addr << ")::Phy2MacStartRx() Receiving disabled" << endl;
    }

}

void UwTrigger_AUV::Phy2MacEndRx(Packet* p) {
    if (receiving_state_active) {
        hdr_cmn* ch = HDR_CMN(p);
        packet_t rx_pkt_type = ch->ptype();
        hdr_mac* mach = HDR_MAC(p);
        hdr_MPhy* ph = HDR_MPHY(p);

        int dest_mac = mach->macDA();
        /*********************/
        double gen_time = ph->txtime;
        double received_time = ph->rxtime;
        double diff_time = received_time - gen_time;
        double distance = diff_time * prop_speed;
        /********************/


        if (debug_) cout << NOW << "  UwTrigger_AUV(" << addr << ")::Phy2MacEndRx() "
                << status_info[curr_state] << ", received a pkt type = "
                << ch->ptype() << ", src addr = " << mach->macSA()
            << " dest addr = " << mach->macDA()
            << ", estimated distance between nodes = " << distance << " m " << endl;

        if (ch->error()) {

            if (debug_) cout << NOW << "  UwTrigger_AUV(" << addr << ")::Phy2MacEndRx() dropping corrupted pkt " << endl;
            incrErrorPktsRx();

            refreshReason(UW_TRIGGER_AUV_REASON_PKT_ERROR);
            drop(p, 1, UW_TRIGGER_AUV_DROP_REASON_ERROR);
            stateRxPacketNotForMe(NULL);
        } else {
            if (dest_mac == addr || dest_mac == (int) MAC_BROADCAST) {
                if (rx_pkt_type != PT_MMAC_TRIGGER) {
                    refreshReason(UW_TRIGGER_AUV_REASON_DATA_RX);
                    stateRxData(p);
                } else {
                    if (debug_) cout << "  UwTrigger_AUV(" << addr << "):: Received a packet of wrong type " << endl;
                    drop(p, 1, UW_TRIGGER_AUV_DROP_REASON_UNKNOWN_TYPE);
                }
            } else {
                refreshReason(UW_TRIGGER_AUV_REASON_PKT_NOT_FOR_ME);
                stateRxPacketNotForMe(p);
            }
        }
    } else {
        if (debug_) cout << NOW << "  UwTrigger_AUV(" << addr << "):: Phy2MacEndRx ---> Not enabled to receive data " << endl;
        drop(p, 1, UW_TRIGGER_AUV_DROP_REASON_RECEIVING_NOT_ENABLED);

    }
}

void UwTrigger_AUV::stateRxData(Packet* data_pkt) {
    refreshState(UW_TRIGGER_AUV_STATE_DATA_RX);
    if (debug_) cout << NOW << "  UwTrigger_AUV(" << addr << ")::stateRxData() " << endl;
    incrDataPktsRx();
    sendUp(data_pkt);
}

void UwTrigger_AUV::stateRxPacketNotForMe(Packet* p) {
    if (debug_) cout << NOW << "  UwTrigger_AUV(" << addr << ")::stateRxPacketNotForMe() pkt for another address. Dropping pkt" << endl;
    if (p != NULL) {
        drop(p, 1, UW_TRIGGER_AUV_DROP_REASON_WRONG_RECEIVER);
    }
}

void UwTrigger_AUV::stateIdle() {

    receive_timer.force_cancel();

    refreshState(UW_TRIGGER_AUV_STATE_IDLE);

    if (print_transitions) printStateInfo();

    if (debug_) std::cout << NOW << "  UwTrigger_AUV(" << addr << ")::stateIdle() --> TRANSMITTING TRIGGER" << endl;
    stateTxTRIGGER();
}

void UwTrigger_AUV::stateTxTRIGGER() {
    if (debug_) cout << NOW << "UwTrigger_AUV(" << addr << ")--->stateTxTRIGGER()" << endl;
    refreshState(UW_TRIGGER_AUV_STATE_TX_TRIGGER);
    Packet* p = Packet::alloc();
    initPkt(p, MAC_BROADCAST);
    txTRIGGER(p);
}

void UwTrigger_AUV::txTRIGGER(Packet* p) {
    if (!receiving_state_active) {
        refreshState(UW_TRIGGER_AUV_STATE_TX_TRIGGER);
        if (debug_) cout << NOW << "UwTrigger_AUV(" << addr << ")---> Transmitting TRIGGER" << endl;
        incrTRIGGERPacketTx();
        Mac2PhyStartTx(p);
    } else {
        if (debug_) cout << NOW << "UwTrigger_AUV(" << addr << ")----> Not transmitting TRIGGER, receiving active" << endl;
        Packet::free(p);
    }
}

void UwTrigger_AUV::printStateInfo(double delay) {
    if (debug_) cout << NOW << " UwTrigger_AUV(" << addr << ")::printStateInfo() " << "from " << status_info[prev_state]
            << " to " << status_info[curr_state] << ". Reason: " << reason_info[last_reason] << endl;
}

void UwTrigger_AUV::waitForUser() {
    std::string response;
    std::cout << "Press Enter to continue";
    std::getline(std::cin, response);
}

