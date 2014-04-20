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

/**
 * @file   uwphysicalrogersmodel.cc
 * @author Giovanni Toso
 * @version 1.0.0
 *
 * \brief Implementation of UnderwaterPhysicalRogersModelRogersModel class
 *
 */

#include "uwphysicalrogersmodel.h"

static class UwPhysicalRogersModelClass : public TclClass {
public:
  UwPhysicalRogersModelClass() : TclClass("Module/UW/PHYSICALROGERSMODEL") {}
  TclObject* create(int, const char*const*) {
    return (new UnderwaterPhysicalRogersModel);
  }
} class_module_uwphysical;

UnderwaterPhysicalRogersModel::UnderwaterPhysicalRogersModel() :
modulation_name_("BPSK"),
time_ready_to_end_rx_(0),
Tx_Time_(0),
Rx_Time_(0),
Energy_Tx_(0),
Energy_Rx_(0),
tx_power_(3.3),
rx_power_(0.620),
Interference_Model("CHUNK")
{
    bind("rx_power_consumption_", &rx_power_);
    bind("tx_power_consumption_", &tx_power_);
}

int UnderwaterPhysicalRogersModel::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();

    if(argc == 2) {
        if(strcasecmp(argv[1],"getTxTime") == 0) {
            //tcl.resultf("%f",Get_Tx_Time());
            return TCL_OK;
        }
    } else if (argc == 3) {
        if (strcasecmp(argv[1], "modulation") == 0) {
            return TCL_OK;
        }
    }
    return UnderwaterMPhyBpsk::command(argc, argv);
} /* UnderwaterPhysicalRogersModel::command */

double UnderwaterPhysicalRogersModel::getAttenutation(const double& distance, const double& frequency) const {
    const double thetag = getTheta_g_max();
    const double thetal = getTheta_l();

    if (thetag >= thetal) {
        return (15 * log (distance) +
            5 * log (getHeight() * getBeta()) +
            (getBeta() * distance * pow(thetal, 2)) / (4 * getHeight()) -
            7.18 +
            getWaterAttenutation() * distance);
    } else {
        return (10 * log (distance) +
            10 * log (getHeight() / (2 * thetal())) +
            (getBeta() * distance * pow(thetal, 2)) / (4 * getHeight()) +
            getWaterAttenutation() * distance);
    }
} /* UnderwaterPhysicalRogersModel::getAttenutation */

