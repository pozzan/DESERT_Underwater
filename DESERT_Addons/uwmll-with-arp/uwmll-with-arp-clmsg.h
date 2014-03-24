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
//
// This module has only slightly modification as respect to the mll module contained  in Miracle, 
// released under the same BSD copyright and implemented by 
// Erik Andersson, Emil Ljungdahl, Lars-Olof Moilanen (Karlstad University)

/**
 * @file   uwmll-clmsg.h
 * @author Saiful Azad
 * @version 1.0.0
 *
 * \brief Provides the declaration of the CrossLayer messages useful for MLL module.
 *
 */


#ifndef UWMLL_CLMSG_H
#define UWMLL_CLMSG_H

#include <clmessage.h>


#define UWMLL_CLMSG_UPD_MAC_VERBOSITY 2  // verbosity of this message


/**
 * Define the the Crosslayer message for udpate the MAC address
 */
extern ClMessage_t UWMLL_CLMSG_UPD_MAC;

/**
 * Class that handle a Packet in ns2
 */
class Packet;

/**
 * Class that defines the CrossLayers message useful for Update the MAC address
 */
class UWMllClMsgUpdMac : public ClMessage {
  
  
  public:
    
   /**
    * Constructor of the class
    * @param Pointer to the Packet used for message
    */ 
  UWMllClMsgUpdMac( Packet* p );
  /**
   * Destructor of the class
   */
  virtual ~UWMllClMsgUpdMac() { }
  /**
   * Copy the message
   * @return ClMessage* Pointer to a copy of the message
   */
  virtual ClMessage* copy();  // copy the message
  
  /**
   * Return the Packet used for the message
   * @return Pointer to the packet
   */
  Packet* getPacket() { return packet; }
  
  /**
   * Set the Packet used for the message
   * @param Packet* pointer to the packet
   */
  void setPacket( Packet* p ) { packet = p; }
   

  protected:
  
  
  Packet* packet; /**< Packet used for handle the message */

  
};



#endif /*  MLL_CLMSG_H */
