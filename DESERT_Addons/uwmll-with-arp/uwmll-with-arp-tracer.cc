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
// This module has only slightly modification as respect to the mll module contained  in Miracle, 
// released under the same BSD copyright and implemented by 
// Erik Andersson, Emil Ljungdahl, Lars-Olof Moilanen (Karlstad University)

/**
 * @file   uwmll-tracer.cc
 * @author Saiful Azad
 * @version 1.0.0
 *
 * \brief Provides the implementation of MLLTracer class that represents the tracer for MLL module.
 *
 */

#include "uwmll-with-arp-tracer.h"

MLLTracer::MLLTracer() : Tracer(1) {}

void MLLTracer::format(Packet *p, SAP *sap)
{
	hdr_cmn* ch = HDR_CMN(p);
	packet_t  t = ch->ptype();

	std::stringstream descrstream;

	if(t == PT_ARP)
	{
		hdr_ll	*lh = HDR_LL(p);
		hdr_arp	*ah = HDR_ARP(p);
		hdr_mac *mh = HDR_MAC(p);

		if(ah->arp_op  == ARPOP_REQUEST)
		{
			descrstream << " ARP Who has "
				    << ah->arp_tpa
				    << " tell "
				    << ah->arp_sha;
		}
		else if(ah->arp_op == ARPOP_REPLY)
		{
			descrstream << " ARP "
				    << ah->arp_spa
				    << " is at "
				    << ah->arp_sha;
		}
	}
	else
	  {
	    hdr_mac *mh = HDR_MAC(p);
	    descrstream << " " << mh->macSA()  << " -> " << mh->macDA() << " ";
	  }

		

	if(descrstream.str().length() > 0)
		writeTrace(sap, "%s", descrstream.str().c_str());
}

extern "C" int Uwmlltracer_Init()
{
	SAP::addTracer(new MLLTracer);
	return 0;
}
extern "C" int  Cyguwmlltracer_Init()
{
	Uwmlltracer_Init();
}
