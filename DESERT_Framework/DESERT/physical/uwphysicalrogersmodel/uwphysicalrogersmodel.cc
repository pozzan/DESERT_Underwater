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
//    derived from this software without specific prior written permission
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

UnderwaterPhysicalRogersModel::UnderwaterPhysicalRogersModel() {
}

int UnderwaterPhysicalRogersModel::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();

    if(argc == 2) {
        if(strcasecmp(argv[1],"getTxTime") == 0) {
            //tcl.resultf("%f",Get_Tx_Time());
            return TCL_OK;
        }
    } else if (argc == 3) {
        if (strcasecmp(argv[1], "set_bottom_depth") == 0) {
            bottom_depth = strtod(argv[2], NULL);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "set_water_attenuation") == 0) {
            water_attenuation = strtod(argv[2], NULL);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "set_sound_speed_surface") == 0) {
            sound_speed_surface = strtod(argv[2], NULL);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "set_sound_speed_bottom") == 0) {
            sound_speed_bottom = strtod(argv[2], NULL);
            return TCL_OK;
        } else if (strcasecmp(argv[1], "set_frequency") == 0) {
            frequency = strtod(argv[2], NULL);
            return TCL_OK;
        }
    }
    return UnderwaterMPropagation::command(argc, argv);
} /* UnderwaterPhysicalRogersModel::command */


double UnderwaterPhysicalRogersModel::getGain(Packet* p) {
    hdr_MPhy *ph = HDR_MPHY(p);

    Position* sp = ph->srcPosition;
    Position* rp = ph->dstPosition;

    MSpectralMask* sm = ph->srcSpectralMask;

    assert(sp);
    assert(rp);
    assert(sm);

    const double freq = ph->srcSpectralMask->getFreq();
    const double dist = sp->getDist(rp);

    const double gain = getAttenuation(sound_speed_bottom, dist/1000.0, freq/1000.0, bottom_depth);

    if (debug_)
        std::cerr << NOW
        << " UnderwaterPhysicalRogersMode::getGain()"
        << " dist=" << dist
        << " freq=" << freq
        << " gain=" << gain
        << std::endl;

    return (gain);

} /* UnderwaterPhysicalRogersModel::getGain */

double UnderwaterPhysicalRogersModel::getAttenuation(const double& _sound_speed_bottom, const double& _distance, const double& _frequency, const double& _bottom_depth) {
    const double thetag_ = getTheta_g_max(_sound_speed_bottom);
    const double thetal_ = getTheta_l(_sound_speed_bottom, _frequency, _bottom_depth);

    if (thetag_ >= thetal_) {
        return (15 * log (_distance) +
            5 * log (_bottom_depth * getBeta()) +
            (getBeta() * _distance * pow(thetal_, 2)) / (4 * _bottom_depth) -
            7.18 +
            getThorp(_frequency) * _distance);
    } else {
        return (10 * log (_distance) +
            10 * log (_bottom_depth / (2 * thetal_)) +
            (getBeta() * _distance * pow(thetal_, 2)) / (4 * _bottom_depth) +
            getThorp(_frequency) * _distance);
    }
} /* UnderwaterPhysicalRogersModel::getAttenutation */

