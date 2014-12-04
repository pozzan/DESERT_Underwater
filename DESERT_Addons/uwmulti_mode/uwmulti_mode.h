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
 * @file   uwmulti_mode.h
 * @author Filippo Campagnaro
 * @version 1.0.0
 * 
 * \brief Provides the definition of the class <i>UwMultiMode</i>.
 * 
 */

#ifndef UWMULTI_MODE_H
#define UWMULTI_MODE_H

#include <mmac.h>
#include <queue>
#include <map>
#include <string>

#define UW_MANUAL_SWITCH 0 // state to switch_mode manually
#define UW_AUTOMATIC_SWITCH 1 // state to switch_mode automatically

using namespace std;

class UwMultiMode;

class PhyMultiRecvSet{
public:
	inline bool contains(int id){ return (recv_physical_.find(id) != recv_physical_.end()); }
	inline bool isEmpty(){ return recv_physical_.empty(); }
	inline int find(int id){ return recv_physical_.find(id)->second; }
	virtual void add(int id);
	virtual void remove(int id);
private:
	map<int,int> recv_physical_;//id, num
};

class UwMultiMode: public MMac {
public:
	UwMultiMode();

	virtual ~UwMultiMode();
	virtual void initInfo();

    virtual int command(int argc, const char*const* argv);
	virtual void stateTxData();
	virtual void stateRxTx();

protected:

  	enum UWMULTI_MODE_STATE {
    	UWMULTI_MODE_STATE_IDLE = 1, UWMULTI_MODE_STATE_TX, UWMULTI_MODE_STATE_RX, 
    	UWMULTI_MODE_STATE_RX_TX
  	};

	int send_physical_id;
	PhyMultiRecvSet recv_physical_ids;
	/*PhyMultiRecvSet gioco;*/
	UWMULTI_MODE_STATE current_state;
	bool initialized;
	//bool sending_channel_idle;
	int switch_mode;
	std::queue<Packet*> buffer;
	map< double, int > send_physical_map;
	static map< UWMULTI_MODE_STATE , string > state_info;

	virtual void Mac2PhyStartTx(Packet* p);
	virtual void Phy2MacEndRx(Packet* p, int idSrc);
	virtual void Phy2MacEndTx(const Packet* p);
	virtual void Phy2MacStartRx(const Packet* p, int idSrc);
	virtual void recvFromUpperLayers(Packet* p);
	virtual void recv(Packet *p, int idSrc);
	virtual int recvSyncClMsg(ClMessage* m);

	virtual void addPhysical(double distance, int phyId);
  
};

#endif 
