//  -*- mode: c++; c-basic-offset: 4; -*-

#include "uwcbr-multihop.h"

int hdr_uwcbr_mh::offset_;

static class UwCbrMultihopPktClass : public PacketHeaderClass {
public:

    UwCbrMultihopPktClass() : PacketHeaderClass("PacketHeader/UWCBRMH", sizeof (hdr_uwcbr_mh)) {
        this->bind();
        bind_offset(&hdr_uwcbr_mh::offset_);
    }
} class_uwcbr_multihop_pkt;

static class UwCbrMultihopClass : public TclClass {
public:
    UwCbrMultihopClass() : TclClass("Module/UW/CBRMH") {}

    TclObject* create(int, const char*const*) {
        return (new UwCbrMultihop());
    }
} class_module_uwcbr_multihop;

static class UwCbrMultihopSourceClass : public TclClass {
public:
    UwCbrMultihopSourceClass() : TclClass("Module/UW/CBRMHSRC") {}

    TclObject* create(int, const char*const*) {
        return (new UwCbrMultihopSource());
    }
} class_uwcbr_multihop_source;

static class UwCbrMultihopSinkClass : public TclClass {
public:
    UwCbrMultihopSinkClass() : TclClass("Module/UW/CBRMHSNK") {}

    TclObject* create(int, const char*const*) {
        return (new UwCbrMultihopSink());
    }
} class_uwcbr_multihop_sink;
