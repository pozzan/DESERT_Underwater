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
 * @file   uwmll-module.cc
 * @author Saiful Azad
 * @version 1.0.0
 *
 * \brief Provides the implementation of UWModuleClass that represents the implementation of MLL module.
 *
 */

#include "uwmll-with-arp-module.h"
#include <mmac-clmsg.h>
#include "uwmll-with-arp-clmsg.h"
#include <uwip-clmsg.h>
#include <iostream>


extern ClMessage_t UWIP_CLMSG_SEND_ADDR;
/**
 * Class that represents the binding with the tcl configuration script 
 */ 
static class UWMllModuleClass : public TclClass {
	public:
		/**
		 * Constructor of the class
		 */
		UWMllModuleClass() : TclClass("Module/UW/MLL-WITH-ARP") {}
		/**
	 	 * Creates the TCL object needed for the tcl language interpretation
	 	 * @return Pointer to an TclObject
	 	 */
		TclObject* create(int, const char*const*) {
			return (new UWMllModule());
		}
} class_uwll;

UWMllModule::UWMllModule() : seqno_(0), ackno_(0)
{
	arptable_ = new UWARPTable();
}

UWMllModule::~UWMllModule()
{
}

int UWMllModule::crLayCommand(ClMessage* m)
{
	if(m->type() == UWIP_CLMSG_SEND_ADDR)
	{
		nsaddr_t tmp = ((UWIPClMsgSendAddr *)m)->getAddr();
		netAddr.push_back(tmp);
		delete m;
		return 1;
	}
	return Module::crLayCommand(m);
}


int UWMllModule::recvAsyncClMsg(ClMessage* m)
{
	if(m->type() == UWIP_CLMSG_SEND_ADDR)
	{
		nsaddr_t tmp = ((UWIPClMsgSendAddr *)m)->getAddr();
		netAddr.push_back(tmp);
		delete m;
		return 1;
	}
	return Module::crLayCommand(m);
}


int UWMllModule::recvSyncClMsg(ClMessage* m) {
  if(m->type() == UWMLL_CLMSG_UPD_MAC)
  {
    Packet* p = ((UWMllClMsgUpdMac *)m)->getPacket();

    hdr_cmn* ch = HDR_CMN(p);
    hdr_uwip *ih = HDR_UWIP(p);
    hdr_mac* mach = HDR_MAC(p);

    nsaddr_t dst = ih->daddr();

    switch(ch->addr_type()) {

      case NS_AF_ILINK:
        /* Uhh!? nsaddr_t to int!? */
        mach->macDA() = ch->next_hop();
        break;

      case NS_AF_INET:
        dst = ch->next_hop();
        /* FALL THROUGH */

      case NS_AF_NONE:
      
        if (IP_BROADCAST == (u_int32_t) dst)
        {
          mach->macDA() = MAC_BROADCAST;
          break;
        }

        /* Resolv ARP */
        arpResolve(dst, p);
        break;

      default:
        mach->macDA() = MAC_BROADCAST;
        break;
    }    
    
    if (debug_) {
      ::std::cout << "UWMllModule::recvSyncClMsg() UWMLL_CLMSG_UPD_MAC received, mac da = " << mach->macDA() << ::std::endl;
    }
    
    return 0;
  }
  
  return Module::recvSyncClMsg(m);
}



int UWMllModule::command(int argc, const char*const* argv)
{
  if (argc == 2)
    {
      if (strcasecmp(argv[1], "reset") == 0) {
	arptable_->clear();
	//FALL-THROUGH to give parents a chance to reset
      }
    }
  if (argc == 4)
    {
      if (strcasecmp(argv[1], "addentry") == 0)
	{
	  nsaddr_t ipaddr = atoi(argv[2]);
	  UWARPEntry* e = new UWARPEntry(ipaddr);
	  e->macaddr_ = atoi(argv[3]);
	  e->up_ = 1;
	  arptable_->addEntry(e);
	  return TCL_OK;
	}
    }
  return Module::command(argc, argv);
}

void UWMllModule::recv(Packet* p)
{
	recv(p, -1);
}

void UWMllModule::recv(Packet *p, int idSrc)
{
	hdr_cmn *ch = HDR_CMN(p);
	if(ch->direction() == hdr_cmn::UP)
	{
		if(ch->ptype_ == PT_ARP)
			processARP(p, idSrc);
		else
			sendUp(p);
	}
	else
	{
		ch->direction() = hdr_cmn::DOWN;
		sendDown(p);
	}
}

void UWMllModule::sendDown(Packet* p)
{
	hdr_cmn *ch = HDR_CMN(p);
	hdr_uwip *ih = HDR_UWIP(p);
	nsaddr_t dst = ih->daddr();
	hdr_ll *llh = HDR_LL(p);
	hdr_mac *mh = HDR_MAC(p);

	llh->seqno_ = ++seqno_;
	llh->lltype() = LL_DATA;

	mh->macSA() = getDownAddr();
	mh->hdr_type() = ETHERTYPE_IP;
	int tx = 0;

	switch(ch->addr_type()) {

		case NS_AF_ILINK:
			/* Uhh!? nsaddr_t to int!? */
			mh->macDA() = ch->next_hop();
			break;

		case NS_AF_INET:
			if (ch->next_hop() == UWIP_BROADCAST) dst = ch->next_hop();
			else if (ch->next_hop() <= 0) dst = ih->daddr();
			else dst = ch->next_hop();

		case NS_AF_NONE:
		
			if (UWIP_BROADCAST == (u_int32_t) dst)
			{
				mh->macDA() = MAC_BROADCAST;
				break;
			}

			/* Resolv ARP */
			tx = arpResolve(dst, p);
			break;

		default:
			mh->macDA() = MAC_BROADCAST;
			break;
	}

	if (tx == 0)
	{
		Module::sendDown(p);
	}
}

void UWMllModule::sendUp(Packet* p)
{
	Module::sendUp(p);
}

void UWMllModule::processARP(Packet* p, int idSrc)
{
	hdr_arp *ah = HDR_ARP(p);
	UWARPEntry *llinfo;

	/* If we don't have requesting node already, add it */
	if((llinfo = arptable_->lookup(ah->arp_spa)) == 0) {

		/* Create a new ARP entry */
		llinfo = new UWARPEntry(ah->arp_spa);
		arptable_->addEntry(llinfo);
	}
	llinfo->macaddr_ = ah->arp_sha;
	llinfo->up_ = 1;

	/* If packet was stored, can we send whatever's being held? */
	if(llinfo->hold_)
	{
		hdr_cmn *ch = HDR_CMN(llinfo->hold_);
		hdr_mac *mh = HDR_MAC(llinfo->hold_);
		hdr_uwip *ih = HDR_UWIP(llinfo->hold_);
		nsaddr_t dst = ih->daddr();

		if((ch->addr_type() == NS_AF_NONE &&
				  dst == ah->arp_spa) ||
				  (NS_AF_INET == ch->addr_type() &&
				  ch->next_hop() == ah->arp_spa))
		{
			if(debug_ > 5)
				printf("\tsending HELD packet.\n");

			mh->macDA() = ah->arp_sha;

			Module::sendDown(llinfo->hold_);
			llinfo->hold_ = 0;
		}
		else
		{
			fprintf(stderr, "\tfatal ARP error...\n");
			 exit(1);
		}
	}

	if(ah->arp_op == ARPOP_REQUEST && netAddrPresent(ah->arp_tpa))
	{
		hdr_cmn *ch = HDR_CMN(p);
		hdr_mac *mh = HDR_MAC(p);
		hdr_ll  *lh = HDR_LL(p);

		ch->size() = ARP_HDR_LEN;
		ch->error() = 0;
		ch->direction() = hdr_cmn::DOWN; // send this pkt down

		int downAddr = getDownAddr(idSrc);

		mh->macDA() = ah->arp_sha;
		mh->macSA() = downAddr;
		mh->hdr_type() = ETHERTYPE_ARP;

		lh->seqno() = 0;
		lh->lltype() = LL_DATA;

		ah->arp_op  = ARPOP_REPLY;
		ah->arp_tha = ah->arp_sha;
		ah->arp_sha = downAddr;

		nsaddr_t t = ah->arp_spa;
		ah->arp_spa = ah->arp_tpa;
		ah->arp_tpa = t;

		Module::sendDown(p);
		return;
	}
	Packet::free(p);
}

bool UWMllModule::netAddrPresent(nsaddr_t addr)
{
	fillNetAddrTable();
	vector<nsaddr_t>::iterator p = find(netAddr.begin(), netAddr.end(), addr);
	return (p != netAddr.end());
}

void UWMllModule::fillNetAddrTable()
{
	netAddr.clear();
	UWIPClMsgReqAddr *c = new UWIPClMsgReqAddr(getId());
	c->setDest(CLBROADCASTADDR);
	sendSyncClMsgUp(c);
	delete c;
}

int UWMllModule::getDownAddr(int downId)
{
	MacClMsgGetAddr *c;
	if(downId != -1)
	{
		c = new MacClMsgGetAddr(UNICAST, downId);
		sendSyncClMsgDown(c);
	}
	else
	{
		c = new MacClMsgGetAddr(BROADCAST, CLBROADCASTADDR);
		sendSyncClMsgDown(c);
	}
	int val = c->getAddr();
	delete c;
	return val;
}

int UWMllModule::arpResolve(nsaddr_t dst, Packet* p)
{
	hdr_mac *mh = HDR_MAC(p);
	UWARPEntry *llinfo;
	llinfo = arptable_->lookup(dst);

	// Found entry, set dest and return
	if(llinfo && llinfo->up_)
	{
		mh->macDA() = llinfo->macaddr_;
		return 0;
	}

	if(llinfo == 0) {
		/* Create a new ARP entry */
		llinfo = new UWARPEntry(dst);
		arptable_->addEntry(llinfo);
	}

	if(llinfo->count_ >= ARP_MAX_REQUEST_COUNT) {
                /*
		* Because there is not necessarily a scheduled event between
		* this callback and the point where the callback can return
		* to this point in the code, the order of operations is very
		* important here so that we don't get into an infinite loop.
		*                                      - josh
		*/
		Packet *t = llinfo->hold_;

		llinfo->count_ = 0;
		llinfo->hold_ = 0;
		hdr_cmn* ch;
		
		if(t) {
			ch = HDR_CMN(t);

			if (ch->xmit_failure_) {
				ch->xmit_reason_ = 0;
				ch->xmit_failure_(t, ch->xmit_failure_data_);
			}
			else {
				drop(t, 1, DROP_IFQ_ARP_FULL);
			}
		}

		ch = HDR_CMN(p);

		if (ch->xmit_failure_) {
			ch->xmit_reason_ = 0;
			ch->xmit_failure_(p, ch->xmit_failure_data_);
		}
		else {
			drop(p, 1, DROP_IFQ_ARP_FULL);
		}

		return EADDRNOTAVAIL;
	}

	llinfo->count_++;
	if(llinfo->hold_)
		drop(llinfo->hold_, 1, DROP_IFQ_ARP_FULL);
	llinfo->hold_ = p;

	/*
	*  We don't have a MAC address for this node.  Send an ARP Request.
	*
	*  XXX: Do I need to worry about the case where I keep ARPing
	*	 for the SAME destination.
	*/
	fillNetAddrTable();
	if(netAddr.size() > 0)
		arpRequest(netAddr[0], dst);
	else
		arpRequest(0, dst);
	return EADDRNOTAVAIL;
}

void UWMllModule::arpRequest(nsaddr_t src, nsaddr_t dst)
{
	Packet *p = Packet::alloc();

	hdr_cmn *ch = HDR_CMN(p);
	hdr_mac *mh = HDR_MAC(p);
	hdr_ll	*lh = HDR_LL(p);
	hdr_arp	*ah = HDR_ARP(p);

	ch->uid() = 0;
	ch->ptype() = PT_ARP;
	ch->size() = ARP_HDR_LEN;
	ch->iface() = -2;
	ch->error() = 0;

	mh->macDA() = MAC_BROADCAST;
	mh->macSA() = getDownAddr();
	mh->hdr_type() = ETHERTYPE_ARP;

	lh->seqno() = 0;
	lh->lltype() = LL_DATA;

	ch->direction() = hdr_cmn::DOWN; // send this pkt down
	ah->arp_hrd = ARPHRD_ETHER;
	ah->arp_pro = ETHERTYPE_IP;
	ah->arp_hln = ETHER_ADDR_LEN;
	ah->arp_pln = sizeof(nsaddr_t);
	ah->arp_op  = ARPOP_REQUEST;
	ah->arp_sha = mh->macSA();
	ah->arp_spa = src;
	ah->arp_tha = 0;		// what were're looking for
	ah->arp_tpa = dst;

	Module::sendDown(p);
}
