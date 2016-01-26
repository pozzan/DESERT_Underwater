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
* @file   uwmulti-stack-Control.h
* @author Filippo Campagnaro, Federico Guerra
* @version 1.0.0
*
* \brief Definition of UwMultiTrafficRangeCtr class.
*
*/

#ifndef UWMULTI_TRAFFIC_Control_H
#define UWMULTI_TRAFFIC_Control_H

#include "uwmulti-traffic-control.h"

// DEFINE BEHAVIORS
#define ROBUST 2
#define CHECK_RANGE 3

// DEFINE STATES
#define IDLE 2
#define RANGE_CNF_WAIT 2

/**
 * Class used to represents the UwMultiTrafficRangeCtr layer of a node.
 */
class UwMultiTrafficRangeCtr : public UwMultiTrafficControl {


public:

  /**
   * Constructor of UwMultiPhy class.
   */
  UwMultiTrafficRangeCtr();

  /**
   * Destructor of UwMultiPhy class.
   */
  virtual ~UwMultiTrafficRangeCtr() { }

  /**
   * TCL command interpreter. It implements the following OTcl methods:
   *
   * @param argc Number of arguments in <i>argv</i>.
   * @param argv Array of strings which are the command parameters (Note that <i>argv[0]</i> 
   *             is the name of the object).
   *
   * @return TCL_OK or TCL_ERROR whether the command has been dispatched successfully or not.
   */
  virtual int command(int, const char*const*);

  /**
   * recv method. It is called when a packet is received from the other layers
   *
   * @param Packet* Pointer to the packet that are going to be received
   */
  virtual void recv(Packet *p);

protected:
  int status;
  /** 
   * manage to tx a packet of traffic type
   *
   * @param traffic application traffic id
   *
   */
  virtual void manageBuffer(int traffic);

  /** 
   * return the Best Lower Layer id where to forward the packet of <i>traffic</i> type
   * 
   * @param traffic application traffic id
   *
   * @return the layer id
   */
  virtual int getBestLowerLayer(int traffic);

private:
  //Variables
};

#endif /* UWMULTI_TRAFFIC_Control_H  */
