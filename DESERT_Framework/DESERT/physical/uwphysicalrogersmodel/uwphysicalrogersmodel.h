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
 * @file   uwphysicalrogersmodel.h
 * @author Giovanni Toso
 * @version 1.0.0
 *
 * \brief Definition of UwPhysicalRogersModel class.
 *
 */

#ifndef UWPHYSICALROGERSMODEL_H
#define UWPHYSICALROGERSMODEL_H

#include <undwerwater.h>

#include <cmath>

class UnderwaterPhysicalRogersModel : public Underwater {

public:
    /**
     * Constructor of UnderwaterMPhyBpskDb class.
     */
    UnderwaterPhysicalRogersModel();

    /**
     * Destructor of UnderwaterMPhyBpskDb class.
     */
    virtual ~UnderwaterPhysicalRogersModel() { }

    /**
     * TCL command interpreter. It implements the following OTcl methods:
     *
     * @param argc Number of arguments in <i>argv</i>.
     * @param argv Array of strings which are the command parameters (Note that <i>argv[0]</i> is the name of the object).
     * @return TCL_OK or TCL_ERROR whether the command has been dispatched successfully or not.
     *
     */
    virtual int command(int, const char*const*);

protected:
    /**
     * Attenuation of acoustic signal in underwater channel.
     * The value returned is base on Rogers model for shallow water.
     *
     * @param dist distance in m
     * @param freq freq in kHz
     * @return Attenuation in dB
     *
     */
    virtual double getAttenuation(double dist, double freq);
    virtual double getM0();
    virtual double getN0();
    virtual double getKs();
    virtual double getBeta(const double M0, const double N0, const double Ks);

    inline double get_g () {
        return std::abs (sound_speed_surface - sound_speed_bottom);
    }

    inline double getTheta_g (const double& height, const double& distance) const {
        return std::sqrt ((1.7 * height) / (getBeta() * distance));
    }

    inline double getTheta_g_max () const {
        return std::sqrt ((2 * get_g) / (sound_speed_bottom));
    }

    inline double getTheta_c (const double& frequency, const double& height) const {
        return (sound_speed_bottom / (2 * frequency * height));
    }

    inline double getTheta_l () const {
        return std::max (getTheta_g_max, getTheta_c);
    }

    inline double getWaterAttenutation(const double& frequency) const {
        return 0.001936 * ((0.1 * pow (f, 2)) / (1 + pow (f, 2)) + (40 * pow (f, 2)) / (4100 + pow (f, 2)))
    }

private:
    //Variables
};

#endif /* UWPHYSICALROGERSMODEL_H  */
