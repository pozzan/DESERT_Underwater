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
//

/**
 * @file   uwtdma.h
 * @author Filippo Campagnaro, Roberto Francescon
 * @version 1.0.0
 * 
 * @brief Provides the definition of the class <i>UWTDMA</i>.
 * 
 */

#ifndef UWTDMA_H
#define UWTDMA_H

#include <mmac.h>
#include <queue>

#define UW_TDMA_STATUS_MY_SLOT 1 /**< Status slot active, whether TDMA modality is on >**/
#define UW_TDMA_STATUS_NOT_MY_SLOT 2 /**< Status slot not active, whether TDMA modality is on >**/

#define UW_CHANNEL_IDLE 1  // status channel idle
#define UW_CHANNEL_BUSY 2  // status channel busy


using namespace std;


class UwTDMA;

/**
 * UwTDMATimer class is used to handle the scheduling period of <i>UWTDMA</i> slots.
 */
class UwTDMATimer : public TimerHandler
{

 public:
  /**
   * Costructor of the class UwTDMATimer
   * @param Pointer of a UwTDMA object
   */
    UwTDMATimer(UwTDMA *m) : TimerHandler() {
        module = m;
    } 

 protected:
  /**
   * Method call when the timer expire
   * @param Event*  pointer to an object of type Event
   */
  virtual void expire(Event *e);
  UwTDMA* module;
};


/*class BufferTimer : public TimerHandler {
public:

    BufferTimer(UwTDMA *m) : TimerHandler() {
        module = m;
    }
protected:
    virtual void expire(Event *e);
    UwTDMA* module;
};*/


/**
 * Class that represents a TDMA Node
 */
class UwTDMA: public MMac {

  friend class UwTDMATimer;

 public:

  /**
   * Constructor of the TDMA class
   */
  UwTDMA();
  
  /**
   * Constructor of the TDMA class
   */
  virtual ~UwTDMA();

  /**
   * TCL command interpreter. It implements the following OTcl methods:
   * 
   * @param argc Number of arguments in <i>argv</i>.
   * @param argv Array of strings which are the command parameters 
                            (Note that <i>argv[0]</i> is the name of the object).
   * @return TCL_OK or TCL_ERROR whether the command has been dispatched 
                                                             successfully or not.
   */
  virtual int command(int argc, const char*const* argv);

 protected:

  /**
   * transmit a data packet whether is my slot
   */
  virtual void txData();
  /**
   * state status my slot and and start to txData
   */
  virtual void stateTxData();
  /**
   * Change TDMA status from ACTIVE to NOT-ACTIVE
   */
  virtual void change_tdma_status();
  /**
   * Schedule the beginning of TDMA slots
   */
  virtual void start();
  /**
   * Receive the packet from the upper layer (e.g. IP)
   * @param Packet* pointer to the packet received
   *
   */
  virtual void recvFromUpperLayers(Packet* p);
  /**
   * Method called when the Phy Layer finish to receive a Packet 
   * @param const Packet* Pointer to an Packet object that rapresent the packet in reception
   */
  virtual void Phy2MacEndRx(Packet* p);
  /**
   * Method called when the Phy Layer start to receive a Packet 
   * @param const Packet* Pointer to an Packet object that rapresent the Packet in reception
   */
  virtual void Phy2MacStartRx(const Packet* p);
  /** 
   * Method called when the Mac Layer start to transmit a Packet 
   * @param const Packet* Pointer to an Packet object that rapresent the Packet in transmission
   */
  virtual void Mac2PhyStartTx(Packet* p);
  /** 
   * Method called when the Mac Layer finish to transmit a Packet 
   * @param const Packet* Pointer to an Packet object that rapresent the Packet in transmission
   */
  virtual void Phy2MacEndTx(const Packet* p);


  int slot_status; //active or not
  int channel_status;
  int debug_;
  double num_hosts;
  double host_id;
  double frame_time; // frame duration
  double guard_time; // guard time between slots
  double slot_duration; // slot duration
  UwTDMATimer tdma_timer; // tdma handler
  //BufferTimer buffer_timer; // buffer handler
  std::queue<Packet*> buffer;

};

#endif 
