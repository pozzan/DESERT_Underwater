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
//

/**
 * @file   uwphysicalfromdb.cc
 * @author Giovanni Toso
 * @version 1.0.0
 *
 * \brief Implementation of UnderwaterPhysicalfromdb class.
 *
 */

#include "uwphysicalfromdb.h"

static class UnderwaterPhysicalfromdbClass : public TclClass {
public:
  UnderwaterPhysicalfromdbClass() : TclClass("Module/UW/PHYSICALFROMDB") {}
  TclObject* create(int, const char*const*) {
    return (new UnderwaterPhysicalfromdb);
  }
} class_UnderwaterPhysicalfromdb;

UnderwaterPhysicalfromdb::UnderwaterPhysicalfromdb() {
}

int UnderwaterPhysicalfromdb::command(int argc, const char*const* argv) {
    //if (argc == 3) {
        //if (strcasecmp(argv[1], "addr") == 0) {
            //ipAddr_ = static_cast<uint8_t>(atoi(argv[2]));
            //if (ipAddr_ == 0) {
                //fprintf(stderr, "0 is not a valid IP address");
                //return TCL_ERROR;
            //}
            //return TCL_OK;
        //} else if (strcasecmp(argv[1], "setCountry") == 0) {
            //country = ((char *) argv[2]);
            //return TCL_OK;
        //} else if (strcasecmp(argv[1], "setModulation") == 0) {
            //modulation = ((char *) argv[2]);
            //return TCL_OK;
        //} else if (strcasecmp(argv[1], "addSnr") == 0) {
            //snr.insert(snr.end(), strtod(argv[2], NULL));
            //return TCL_OK;
        //} else if (strcasecmp(argv[1], "addSir") == 0) {
            //sir.insert(sir.end(), strtod(argv[2], NULL));
            //return TCL_OK;
        //} else if (strcasecmp(argv[1], "addOverlap") == 0) {
            //overlap.insert(overlap.end(), strtod(argv[2], NULL));
            //return TCL_OK;
        //} else if (strcasecmp(argv[1], "setPath") == 0) {
            //path_ = (char*) (argv[2]);
            //return TCL_OK;
        //} else if (strcasecmp(argv[1], "setInterference") == 0) {
            //interference_ = dynamic_cast<uwinterference*> (TclObject::lookup(argv[2]));
            //if(!interference_)
                //return TCL_ERROR;
            //return TCL_OK;
    //}
    //return UnderwaterPhysical::command(argc, argv);
} /* UnderwaterPhysicalfromdb::command */

void UnderwaterPhysicalfromdb::recv(Packet* p) {
    //hdr_cmn *ch = HDR_CMN(p);
    //hdr_MPhy *ph = HDR_MPHY(p);

    //if (ch->direction() == hdr_cmn::UP) {
        //ph->dstSpectralMask = getRxSpectralMask(p);
        //ph->dstPosition = getPosition();
        //ph->dstAntenna = getRxAntenna(p);

        //assert(ph->dstSpectralMask);
        //assert(ph->dstPosition);

        //ph->Pr = getRxPower(p);

        //p->txinfo_.RxPr = 0;
        //p->txinfo_.CPThresh = 0;

        //if (ph->Pr > 0) {
            //ph->Pn = getNoisePower(p);

            //if (interference_) {
                //interference_->addToInterference(p);
            //}

            //ph->rxtime = NOW;
            //ph->worth_tracing = true;

            //if (isOn == true) {
                //PacketEvent* pe = new PacketEvent(p);
                //Scheduler::instance().schedule(&rxtimer, pe, ph->duration);

                //startRx(p);
            //} else {
                //Packet::free(p);
            //}
        //} else {
            //Packet::free(p);
        //}
    //} else { // Direction DOWN
        //assert(isOn);

        //ph->Pr = 0;
        //ph->Pn = 0;
        //ph->Pi = 0;
        //ph->txtime = NOW;
        //ph->rxtime = ph->txtime;

        //ph->worth_tracing = false;

        //ph->srcSpectralMask = getTxSpectralMask(p);
        //ph->srcAntenna = getTxAntenna(p);
        //ph->srcPosition = getPosition();
        //ph->dstSpectralMask = 0;
        //ph->dstPosition = 0;
        //ph->dstAntenna = 0;
        //ph->modulationType = getModulationType(p);
        //ph->duration = getTxDuration(p);

        //ph->Pt = getTxPower(p);

        //assert(ph->srcSpectralMask);
        //assert(ph->srcPosition);
        //assert(ph->duration > 0);
        //assert(ph->Pt > 0);

        //ch->prev_hop_ = ipAddr_;  // Must be added to ensure compatibility with uw-al

        //PacketEvent* pe = new PacketEvent(p->copy());
        //Scheduler::instance().schedule(&txtimer, pe, ph->duration);

        //startTx(p);
    //}
}

void UnderwaterPhysicalfromdb::endRx(Packet* p) {
    hdr_cmn* ch = HDR_CMN(p);
    hdr_MPhy* ph = HDR_MPHY(p);

} /* UnderwaterPhysicalfromdb::endRx */

