//
// Copyright (c) 2015 Regents of the SIGNET lab, University of Padova.
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
 * @file   uwoptical-phy.h
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Definition of UwOptical class.
 *
 */

#ifndef UWOPTICALPHY_OOKRS_H
#define UWOPTICALPHY_OOKRS_H

#include <mphy-sense.h>
#include <rng.h>
#include <tclcl.h>

#include <map>
#include <string>

#define OPTICAL_OOKRS_MOD_TYPE "OPTICAL_OOKRS"
#define NOT_FOUND_VALUE 0

class UwOpticalPhyOOKRS : public MPhyChSense {
public:
  UwOpticalPhyOOKRS();
  virtual ~UwOpticalPhyOOKRS();

  virtual int getModulationType(Packet *p);
  virtual double getTxDuration(Packet *p);
  virtual int command(int, const char*const*);
  virtual double getNoisePower(Packet* p);
  virtual void recv(Packet *p);

  /** Compute the SNR (without interference) in dB of the received packet */
  double getSNRdB(Packet *p);
  /** Compute the SINR in dB of the received packet */
  double getSINRdB(Packet *p);
protected:
  typedef std::map< double, double > DepthMap;
  typedef DepthMap::iterator DMIt;
    
  virtual void startRx(Packet *p);
  virtual void endRx(Packet *p);
  virtual void startTx(Packet *p);
  virtual void endTx(Packet *p);
  /** Inherited from the superclass. Should return the channel noise power? */
  virtual double getNoiseChannel();

  void initializeLUT();
  double lookUpLightNoiseE(double depth);

  /** Computes the bit error rate of the OOK modulation given the linear SNR */
  double bitErrorRateOOK(double snr);
  /** Use the SINR of the received packet to generate a number of wrong bits */
  void setWrongBits(Packet *p);

  /** Setup the modulation type */
  static void initialize();
    
  static int modid; ///< Modulation id given by MPhy
  static bool initialized; ///< True after a call to initialize
private:
  double linearInterpolator( double x, double x1, double y1, double x2, double y2 );
    
  double BitRate_; ///< Raw bitrate
  
  double Id; ///< dark current
  double Il; ///< light current it can be approximated to the short circuit current
  double R; ///< shunt resistance
  double S; ///< sensitivity
  double T; ///< temperature (K)
  double Ar_; ///< receiver area [m^2]
    
  std::string lut_file_name_; //LUT file name
  char lut_token_separator_;
  DepthMap lut_map;
    
  bool use_woss_;
    
  int use_reed_solomon;
  int rs_n;
  int rs_k;

  Packet *PktRx; ///< Packet the receiver is synchronized to for reception
  bool txPending; ///< Flag to indicate whether there is a transmission in progress
};

#endif

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 2
// End:
