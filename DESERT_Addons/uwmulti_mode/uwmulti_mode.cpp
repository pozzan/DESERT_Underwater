
#include "uwmulti_mode.h"

#include <iostream>

#include <stdint.h>

static class UwMultiModeClass : public TclClass {
public:

	/**
	 * Constructor of the class
	*/
	UwMultiModeClass() : TclClass("Module/UW/MULTI/MODE") {}

	/**
	* Creates the TCL object needed for the tcl language interpretation
	* @return Pointer to an TclObject
	*/
	TclObject* create(int, const char*const*) {
		return (new UwMultiMode());
	}

} class_uwmulti_mode;

/*void BufferTimer::expire(Event *e) {
    ((UwMultiMode *)module)->stateTxData();
}*/

UwMultiMode::UwMultiMode() 
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

UwMultiMode::~UwMultiMode() {
}

int UwMultiMode::command(int argc, const char*const* argv)
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

void UwMultiMode::Mac2PhyStartTx(Packet* p)
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

void UwMultiMode::stateTxData()
{
	channel_status=UW_CHANNEL_IDLE;
	if(buffer.size()>0){
		Mac2PhyStartTx(buffer.front());
		buffer.pop();
	}
}
void UwMultiMode::recvFromUpperLayers(Packet* p)
{
	buffer.push(p);
	if(channel_status==UW_CHANNEL_IDLE){
 		Mac2PhyStartTx(p);
 		buffer.pop();
 	}
  	MMac::recvFromUpperLayers(p);
}

void UwMultiMode::Phy2MacEndRx(Packet* p){
	sendUp(p);
}

void UwMultiMode::Phy2MacEndTx(const Packet* p){
	//std::cout <<"jera ora, xio ken!" << std::endl;
	stateTxData();
}