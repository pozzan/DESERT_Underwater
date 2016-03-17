//
// Copyright (c) 2015 Regents of the SIGNET lab, University of Padova.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Padova (SIGNET lab) nor the
//  names of its contributors may be used to endorse or promote products
//  derived from this software without specific prior written permission.
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
 * @file   uwhermesphy.h
 * @author Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Definition of UwHermesPhy class.
 *
**/

#ifndef UWHERMESPHY_H
#define UWHERMESPHY_H

#include "uwphysical.h"
#include <math.h> 
#include <iostream>
#include <map>

using namespace std;

typedef ::std::map< double, double > PdrLut;
typedef PdrLut::iterator PdrLutIt;

class UwHermesPhy : public UnderwaterPhysical {

public:

  /**
   * Constructor of UwHermesPhy class.
   */
  UwHermesPhy();
  
  /**
   * Destructor of UwHermesPhy class.
   */
  virtual ~UwHermesPhy();


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
   * Handles the end of a packet reception
   *
   * @param Packet* p Pointer to the packet received
   *
   */
  virtual void endRx(Packet* p);

  /**
   * Returns the packet error rate by using the length of a packet and the information contained in the packet (position
   * of the source and the destiantion.
   *
   * @param snr Calculated by nsmiracle with the Urick model (unused).
   * @param nbits length in bit of the packet.
   * @param p Packet by witch the module gets information about source and destination.
   * @return PER of the packet passed as parameter.
   */
  virtual double getPER(double snr, int nbits, Packet*);

  virtual void initializeLUT();  
  
  
private:

  /**  
  * Return the distance between source and destination.
  *
  * @param p Packet by witch the module gets information about source and destination.
  **/
  virtual double getDistance(Packet*);

  /**  
  * Return the PER via linear interpolation.
  *
  * @param distance: distance between source and destination.
  * @param size: Packet size in bit.
  **/
  virtual double matchPS(double distance, int size);

  /**  
  * Return y via linear interpolation given two points.
  *
  * @param x: input.
  * @param x1, y1: coordinates of the first point.
  * @param x2, y2: coordinates of the second point.
  **/
  virtual double linearInterpolator( double x, double x1, double y1, double x2, double y2 );

  virtual double chunckInterpolator( double p, int size );

  string pdr_file_name_; //LUT file name
  char pdr_token_separator_; //LUT token separator
  PdrLut range2pdr_;
  bool initLUT_;

    /**
   *
   * BCH(n,k,t) : correction of t errors via BCH code
   * new frame length= FRAME_BIT/k*n, example: 9152/11*15
   * by default BCH(15,11,1)
   * 
  **/
  int BCH_N; // by default = 15; BCH(15,11,1)
  int BCH_K; // by default = 11
  int BCH_T; // by default = 1; unused, just for completeness

  int FRAME_BIT; // by default = (9120 info bit +32 CRC bit)

};

#endif /* UWHERMESPHY_H  */
