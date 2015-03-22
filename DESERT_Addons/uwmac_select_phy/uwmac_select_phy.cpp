
#include "uwmac_select_phy.h"

#include <iostream>

#include <stdint.h>

static class UwMacSelectPhyClass : public TclClass {
public:

	/**
	 * Constructor of the class
	*/
	UwMacSelectPhyClass() : TclClass("Module/UW/MAC/SELECT_PHY") {}

	/**
	* Creates the TCL object needed for the tcl language interpretation
	* @return Pointer to an TclObject
	*/
	TclObject* create(int, const char*const*) {
		return (new UwMacSelectPhy());
	}

} class_uwmac_select_phy;

/*void BufferTimer::expire(Event *e) {
    ((UwMacSelectPhy *)module)->stateTxData();
}*/

UwMacSelectPhy::UwMacSelectPhy() 
:
MMac(),
send_physical_id(0),
recv_physical_id(0)
//buffer_timer(this)
{
    bind("send_physical_id", &send_physical_id);
    bind("recv_physical_id", &recv_physical_id);
    bind("debug_", &debug_);
	channel_status = UW_CHANNEL_IDLE;
}

UwMacSelectPhy::~UwMacSelectPhy() {
}

int UwMacSelectPhy::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
	if (argc==3){
		if(strcasecmp(argv[1], "setSendPhysicalId") == 0){
     		send_physical_id=atoi(argv[2]);
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setRecvPhysicalId") == 0){
     		recv_physical_id=atoi(argv[2]);
			return TCL_OK;
		}
	}
	return MMac::command(argc, argv);
}

void UwMacSelectPhy::Mac2PhyStartTx(Packet* p)
{
	channel_status=UW_CHANNEL_BUSY;
	if(!send_physical_id){
		MMac::Mac2PhyStartTx(p);
		//buffer_timer.resched(Mac2PhyTxDuration(p)*1.001);
	}
	else{
		MMac::Mac2PhyStartTx(send_physical_id, p);
		//buffer_timer.resched(Mac2PhyTxDuration(send_physical_id, p)*1.001);
	}
}

void UwMacSelectPhy::stateTxData()
{
	channel_status=UW_CHANNEL_IDLE;
	if(buffer.size()>0){
		Mac2PhyStartTx(buffer.front());
		buffer.pop();
	}
}
void UwMacSelectPhy::recvFromUpperLayers(Packet* p)
{
	buffer.push(p);
	if(channel_status==UW_CHANNEL_IDLE){
 		Mac2PhyStartTx(p);
 		buffer.pop();
 	}
  	MMac::recvFromUpperLayers(p);
}

void UwMacSelectPhy::Phy2MacEndRx(Packet* p){
	hdr_cmn* ch = HDR_CMN(p);
	if (ch->error())
    	drop(p, 1, "ERR");
  	else
		sendUp(p);
}

void UwMacSelectPhy::Phy2MacEndTx(const Packet* p){
	//std::cout <<"jera ora, xio ken!" << std::endl;
	stateTxData();
}
