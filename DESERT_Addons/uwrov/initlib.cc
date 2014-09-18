#include <tclcl.h>
#include <sap.h>

#include "uwrov-packet.h"
extern EmbeddedTcl UwrovTclCode;

static class UwROVMonPktClass : public PacketHeaderClass {
public:

    UwROVMonPktClass() : PacketHeaderClass("PacketHeader/UWROV", sizeof (hdr_uwROV_monitoring)) {
        this->bind();
        bind_offset(&hdr_uwROV_monitoring::offset_);
    }
} class_uwROV_pkt;
/**
 * Adds the header for <i>hdr_uwROV</i> packets in ns2.
 */
static class UwROVCtrPktClass : public PacketHeaderClass {
public:

    UwROVCtrPktClass() : PacketHeaderClass("PacketHeader/UWROVCtr", sizeof (hdr_uwROV_ctr)) {
        this->bind();
        bind_offset(&hdr_uwROV_ctr::offset_);
    }
} class_uwROVCtr_pkt;


extern "C" int Uwrov_Init() {
	UwrovTclCode.load();
	return 0;
}
