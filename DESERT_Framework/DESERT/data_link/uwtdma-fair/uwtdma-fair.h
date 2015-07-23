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
//


/**
 * @file   uwtdma-fair.h
 * @author Roberto Francescon
 * @version 0.1.0
 * 
 * @brief A TDMA MAC algorithm with equal length time slots.
 * 
 */

#ifndef UWTDMAFAIR_H
#define UWTDMAFAIR_H

#include <uwtdma.h>

/**
 * Class that represents a TDMAFair Node
 */
class UwTDMAFair : public UwTDMA
{

 public:
  
  /**
   * Constructor of the TDMAFair class
   */
  UwTDMAFair();
  /**
   * Destructor of the TDMAFair class
   */
  ~UwTDMAFair();
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
  
  int start_slot;  /**Slot from which will start the TDMA simulation*/
  int slot_number; /**set the position of the node in the frame 
                                    (starting from 0 to tot_slots-1)*/
  int tot_slots;   /**simply the number of nodes in the simulation*/

};

#endif
