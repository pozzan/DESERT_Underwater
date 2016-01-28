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

#ifndef UWMULTI_TRAFFIC_CONTROL_H
#define UWMULTI_TRAFFIC_CONTROL_H

#include "uwmulti-traffic-control.h"

// DEFINE BEHAVIORS
#define ROBUST 2
#define CHECK_RANGE 3

// DEFINE STATES
#define IDLE 1
#define RANGE_CNF_WAIT 2

struct stack_status {
  int stack_id;
  int module_id;
  int status;
  int robust_id;
} ;
typedef std::map <int, stack_status> StatusMap; /** traffic, status */

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
  * Cross-Layer messages asynchronous interpreter. 
  * 
  * It has to be properly extended in order to 
  * interpret custom cross-layer messages used by this particular plug-in.
  * This type of communication does not necessarily need a reply.
  *
  * @note Each implementation of this method is responsible for
  * deleting the ClMessage instance referred to by ClMessage* m
  * when the message is received
  *
  * Normally, classes inheriting from other classes should call
  * the recvAsyncClMsg() method of the parent when an unknown
  * ClMsg is detected, in order to allow the parent to handle
  * unknown message types. 
  *
  * A  very importan exception to this rule are classes
  * inheriting directly from either Plugin or Module. These
  * classes should NOT call  neither Plugin::recvAsyncClMsg()
  * nor Module::recvAsyncClMsg() for unknown messages; instead,
  * they should just free the memory associated with ClMessage* m
  * 
  * @param m an instance of <i>ClMessage</i> that represent the message received
  *
  * @return 0 if the method was re-implemented by somebody,
  * RETVAL_NOT_IMPLEMENTED if it is the implementation provided
  * by the parent Plugin class (note that Module does not
  * re-implement it, so also Module::recvAsyncClMsg() returns
  * RETVAL_NOT_IMPLEMENTED)
  * 
  * @see NodeCore, ClMessage, ClSAP, ClTracer
  **/
  int recvAsyncClMsg(ClMessage* m);

  void sendDown(Packet* p) { if(p != NULL ) { Module::sendDown(p); } }

  void sendDown(int moduleId, Packet* p) { if(p != NULL ) { Module::sendDown(moduleId, p); } }

protected:
  StatusMap status;
  double check_to;
  /** 
   * manage to tx a packet of traffic type
   *
   * @param traffic application traffic id
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

  /** 
   * procedure to check if a 
   * 
   * @param traffic application traffic id
   */
  virtual void checkRange(int traffic, int stack_id, int module_id);

  /** 
   * procedure when a CHECKED stack is checked
   * 
   * @param stack_id stack id
   * @param in_range true if the PHY is in range, false otherwise
   */
  virtual void manageCheckedStack(int stack_id, bool in_range);

  /** 
   * default status initialization
   * 
   * @param traffic application traffic id
   */
  virtual void initStatus(int traffic);

  virtual void timerExpired(int traffic);

private:
  //Variables
  class UwCheckRangeTimer : public TimerHandler {
  public:

      UwCheckRangeTimer(UwMultiTrafficRangeCtr *m, int traff) : 
      TimerHandler(), 
      traffic(traff),
      max_increment(10),
      num_expires(0)
      {
          module = m;
      }
      int traffic;
      int num_expires;
      int const max_increment;

  protected:
      virtual void expire(Event *e);
      UwMultiTrafficRangeCtr* module;
  };

  std::map <int, UwCheckRangeTimer> timers; //<stack_id, timer>
};

#endif /* UWMULTI_TRAFFIC_CONTROL_H  */
