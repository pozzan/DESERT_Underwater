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
 * @file   uwtdma-fair.cpp
 * @author Roberto Francescon
 * @version 0.0.1
 * 
 * @brief Provides the implementation of UwTDMAFair class
 * 
 */

#include "uwtdma-fair.h"
#include "uwtdma.h"

/**
 * Class that represent the binding of the protocol with tcl
 */
static class TDMAFairModuleClass : public TclClass 
{

  public:

   /**
   * Constructor of the class TDMAFairModuleClass
   */
  TDMAFairModuleClass() : TclClass("Module/UW/TDMAFair") {}
  /**
   * Creates the TCL object needed for the tcl language interpretation
   * @return Pointer to an TclObject
   */
  TclObject* create(int, const char*const*){
    return (new UwTDMAFair());
  }

} class_uwtdmafair;


UwTDMAFair::UwTDMAFair() : UwTDMA()
{
  bind("nodes_number", (int*)& nodes_number);
}

UwTDMAFair::~UwTDMAFair(){}


int UwTDMAFair::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
 	if (argc==2){
    	if(strcasecmp(argv[1], "start") == 0){
	      start();
	      return TCL_OK;
	    }
	    else if(strcasecmp(argv[1], "stop") == 0){
	      tdma_timer.cancel();
	      return TCL_OK;
	    } 
	    else if (strcasecmp(argv[1], "get_buffer_size") == 0){
	    	tcl.resultf("%d", buffer.size());
		  	return TCL_OK;
	    }
	    else if (strcasecmp(argv[1], "get_upper_data_pkts_rx") == 0){
	      tcl.resultf("%d", up_data_pkts_rx);
	      return TCL_OK;
	    }
  	}		
	else if (argc==3){
		if(strcasecmp(argv[1], "setSlotStatus") == 0){
		        slot_status=atoi(argv[2]);
			return TCL_OK;
		}
		else if(strcasecmp(argv[1], "setHostId") == 0){
		        host_id=atoi(argv[2]);
			return TCL_OK;
		}
                else if(strcasecmp(argv[1], "setSlotDuration") == 0){
			slot_duration=atof(argv[2]);
			return TCL_OK;
                }
                else if(strcasecmp(argv[1], "setFrameTime") == 0){
			frame_time=atof(argv[2]);
			return TCL_OK;
		}
	}
	return MMac::command(argc, argv);
}
