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
 * @file   uwmulti-stack-controller.h
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Definition of UwMultiStackController class.
 *
 */

#ifndef UWMULTI_STACK_CONTROLLER_H
#define UWMULTI_STACK_CONTROLLER_H

#include <rng.h>
#include <packet.h>
#include <module.h>
#include <tclcl.h>
#include <map>

#include <iostream>
#include <string.h>
#include <cmath>
#include <limits>
#include <climits>
#include "controller-clmsg.h"

class UwMultiStackController : public Module {


public:
    /**
     * Constructor of UwMultiPhy class.
     */
    UwMultiStackController();
    
    /**
     * Destructor of UwMultiPhy class.
     */
    virtual ~UwMultiStackController() { }
    
    /**
     * TCL command interpreter. It implements the following OTcl methods:
     *
     * @param argc Number of arguments in <i>argv</i>.
     * @param argv Array of strings which are the command parameters (Note that <i>argv[0]</i> is the name of the object).
     *
     * @return TCL_OK or TCL_ERROR whether the command has been dispatched successfully or not.
     */
    virtual int command(int, const char*const*);

    /**
     * Add layer
     * 
     * @param id unique identifier of the module
     * @param layer_name name of the module. The name should be unique
     * @param target target of the module metrics
     * @param hysteresis hysteresis of the module metrics
     */
    
    virtual void addLayer(int id, string layer_name , double target, double hysteresis);
    
    /**
     * recv method. It is called when a packet is received from the other layers
     *
     * @param Packet* Pointer to the packet that are going to be received
     */
    virtual void recv(Packet *p);


    
protected:
    // Variables
    enum Mode
    {
      UW_MANUAL_SWITCH = 0, // state to switch_mode manually
      UW_AUTOMATIC_SWITCH // state to switch_mode automatically
    };

    int debug_;
    int min_delay_;
    Mode switch_mode_;// AUTOMATIC or MANUAL MODE
    int lower_id_active_; // used just in MANUAL MODE

    class Stats
    {
       public:

        string layer_tag_;
        double metrics_target_;
        double hysteresis_size_;

    };

    std::map<int, Stats> layer_map; // layerid, stats

    /** 
     * Handle a packet coming from upper layers
     * 
     * @param p pointer to the packet
     */
    virtual void recvFromUpperLayers(Packet *p);

    /** 
     * return the best layer to forward the packet when the system works in AUTOMATIC_MODE
     * 
     * @param p pointer to the packet
     *
     * @return id of the module representing the best layer
    */
    virtual int  getBestLayer(Packet *p);

    /** 
     * return if the specified layer identified by name is available
     * 
     * @param layer_name 
     *
     * @return if the specified layer is available
     */
    virtual bool isLayerAvailable(int id); //@ TOCHECK: isn't it more convenient to use layer_id than layer_name??

    /** 
     * return the new metrics value obtained from the selected lower layer,
     * in proactive way via ClMessage
     * 
     * @param id to select the lower layer 
     * @param p pointer to the packet 
     *
     * @return the value of the new value of the metrics obtained in proactive way
     */
    virtual double getMetricFromSelectedLowerLayer(int id, Packet* p);

private:
    //Variables
};

#endif /* UWMULTI_STACK_CONTROLLER_H  */
