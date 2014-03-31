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

/* 
 * File:   uwApplication_module.cc
 * Author: Loris Brolo
 *
 * Created on 15 dicembre 2013, 14.54
 */

#include <sstream>
#include <time.h>
#include "uwApplication_cmn_header.h"
#include "uwApplication_module.h"

static class uwApplicationModuleClass : public TclClass {
public:

    /**
     * Constructor of uwApplicationModuleClass class 
     */
    uwApplicationModuleClass() : TclClass("Module/UW/APPLICATION") {
    }

    /**
     *  Creates the TCL object needed for the TCL language interpretation
     * 
     * @return Pointer to an TclObject
     */
    TclObject* create(int, const char*const*) {
        return (new uwApplicationModule());
    }
} class_module_uwapplicationmodule;

uwApplicationModule::uwApplicationModule()
: chkTimerPeriod(this),
uidcnt(0),
txsn(1),
rftt(0),
pkts_lost(0),
pkts_recv(0),
pkts_ooseq(0),
pkts_invalid(0),
pkts_last_reset(0),
lrtime(0),
sumrtt(0),
sumrtt2(0),
rttsamples(0),
sumftt(0),
sumftt2(0),
fttsamples(0),
esn(0),
sumbytes(0),
sumdt(0),
hrsn(0),
servSockDescr(0),
servPort(0),
socket_active(0),
tcp_udp(-1)
{
    bind("debug_", (int*) &debug_);
    bind("period_", (int*) &PERIOD);
    bind("PoissonTraffic_", (int*) &POISSON_TRAFFIC);
    bind("Payload_size_", (int*) &PAYLOADSIZE);
    bind("destAddr_", (int*) &DST_ADDR);
    bind("destPort_", (int*) &PORT_NUM);
    bind("Socket_Port_", (int*) &servPort);
    bind("drop_out_of_order_", (int*) &DROP_OUT_OF_ORDER);
    bind("pattern_sequence_", (int*) &PATTERN_SEQUENCE);

    sn_check = new bool[USHRT_MAX];
    for (int i = 0; i < USHRT_MAX; i++) {
        sn_check[i] = false;
    }
    //servPort = PORT_NUM;
} //end uwApplicationModule() Method

uwApplicationModule::~uwApplicationModule() {

}

int uwApplicationModule::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();

    if (argc == 2) {
        if (strcasecmp(argv[1], "start") == 0) {
            if (withoutSocket()) {
                //Generate DATA packets without the use of sockets
                start_generation();
            } else {
                //The communication take place with the use of sockets 
                if (useTCP()) {
                    //Generate DATA packets using TCP connection
                    uwApplicationModule::openConnectionTCP();
                } else {
                    //Generate DATA packets using UDP connection
                    uwApplicationModule::openConnectionUDP();
                }
            }
            return TCL_OK;
        } else if (strcasecmp(argv[1], "stop") == 0) {
            stop();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getsentpkts") == 0) {
            tcl.resultf("%d", getPktSent());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "lostpkts") == 0) {
            tcl.resultf("%f", getPktLost());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrecvpkts") == 0) {
            tcl.resultf("%d", getPktRecv());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "outofsequencepkts") == 0) {
            tcl.resultf("%f", getPktsOOSequence());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "notknownpktrx") == 0) {
            tcl.resultf("%f", getPktsInvalidRx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrecvpktsqueue") == 0) {
            tcl.resultf("%d", getPktsPushQueue());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrtt") == 0) {
            tcl.resultf("%f", GetRTT());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getrttstd") == 0) {
            tcl.resultf("%f", GetRTTstd());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getftt") == 0) {
            tcl.resultf("%f", GetFTT());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getfttstd") == 0) {
            tcl.resultf("%f", GetFTTstd());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getper") == 0) {
            tcl.resultf("%f", GetPER());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getthr") == 0) {
            tcl.resultf("%f", GetTHR());
            return TCL_OK;
        } 
    } else if (argc == 3) {
        if (strcasecmp(argv[1],"SetSocketProtocol") == 0) {
            string protocol = argv[2];
            cout << "Setting socket !!" << endl;
            if (strcasecmp(protocol.c_str(),"UDP") == 0)
            {
                socket_active = true;
                tcp_udp = 0;
            } else if (strcasecmp(protocol.c_str(),"TCP") == 0) {
                socket_active = true;
                tcp_udp = 1;
            } else {
                socket_active = false;
                tcp_udp = -1;
            }
            return TCL_OK;
        }
    }
    return Module::command(argc, argv);
}//end command() Method

int uwApplicationModule::crLayCommand(ClMessage* m) {
    switch (m->type()) {
        default:
            return Module::crLayCommand(m);
    }
}//end crLayCommand() Method

void uwApplicationModule::recv(Packet* p) {
    
    if (withoutSocket()) {
        //if (debug_ >= 1) std::cout << "Time: " << NOW << " uwApplicationModule::recv() ---> Using CBR without socket communication." << std::endl;
        if (debug_ >=1 ) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::RECV_PACKET_WITHOUT_SOCKET_MODE" << endl;
        //uwApplicationModule::statistics(p);
        statistics(p);
    } else {
        //Communication take place with sockets 
        if (useTCP()) {
            if (debug_ >=1 ) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::RECV_PACKET_USING_TCP" << endl;
            statistics(p);
        } else {
            if (debug_ >=1 ) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::RECV_PACKET_USING_UDP" << endl;
            statistics(p);
        }
    }
}; //end recv() Method

void uwApplicationModule::statistics(Packet* p) {
    hdr_cmn* ch = hdr_cmn::access(p);
    hdr_DATA_APPLICATION* uwApph = HDR_DATA_APPLICATION(p);

    if (ch->ptype_ != PT_DATA_APPLICATION) {
        if (debug_ >=0 ) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::DROP_PACKET_NOT_APPLICATION_TYPE" << endl;
        drop(p, 1, UWAPPLICATION_DROP_REASON_UNKNOWN_TYPE);
        incrPktInvalid(); //Increment the number of packet received invalid
        return;
    }
    esn = hrsn + 1; // Increase the expected sequence number

    //Verify if the data packet is already processed. 
    if (!useDropOutOfOrder()) {
        if (sn_check[uwApph->sn_ & 0x00ffffff]) { // Packet already processed: drop it
            if (debug_ >= 0 ) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::DROP_PACKET_PACKET_ALREADY_PROCESSED_ID_" << (int)uwApph->sn_ << endl;
            incrPktInvalid();
            drop(p, 1, UWAPPLICATION_DROP_REASON_DUPLICATED_PACKET);
            return;
        }
    }
    //The data packet with this particular SN is not already processed
    //Set true sn_check. In this way we assume that these sn are already processed by the node
    sn_check[uwApph->sn_ & 0x00ffffff] = true;

    //The data packet received is out of order
    if (useDropOutOfOrder()) {
        if (uwApph->sn_ < esn) { // packet is out of sequence and is to be discarded
            incrPktOoseq(); //Increase the number of data packets receive out of sequence.
            //if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::recv() ---> Packet received with SN " << uwApph->sn_ << " is out of sequence."
            //        << " The highest sequence number received is " << hrsn << ". DROP IT!" << std::endl;
            if (debug_ >= 0 ) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::DROP_PACKET_PACKET_OOS_ID_" << (int)uwApph->sn_ << "_LAST_SN_" << hrsn << endl;
            drop(p, 1, UWAPPLICATION_DROP_REASON_OUT_OF_SEQUENCE);
            return;
        }
    }

    //Compute the Forward Trip time 
    rftt = Scheduler::instance().clock() - ch->timestamp();
    if ( (uwApph->rftt_valid_)/10000 ) {
        double rtt = rftt + uwApph->rftt();
        //if (debug_ >= 1 ) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::DROP_PACKET_PACKET_OOS_ID_" << (int)uwApph->sn_ << "_LAST_SN_" << hrsn << endl;
        updateRTT(rtt);
    }

    updateFTT(rftt); //Update the forward trip time

    hrsn = uwApph->sn_; //Update the highest sequence number received
   
    //Verify if a packet is lost
    if (useDropOutOfOrder()) {
        if (uwApph->sn_ > esn) {
            if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::recv() ---> Packet lost. Packet received have SN " << uwApph->sn_ << ","
                    << " the expected SN is " << esn << std::endl;
            if (debug_ >= 0) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::PACKET_LOST_ID_RECEIVED" << (int)uwApph->sn_ << "_ID_EXPECTED_" << esn << endl;
            incrPktLost(uwApph->sn_ - (esn));
        }
    }

    double dt = Scheduler::instance().clock() - lrtime;
    updateThroughput(ch->size(), dt); //Update Throughput

    incrPktRecv(); //Increase the number of data packets received

    lrtime = Scheduler::instance().clock(); //Update the time in which the last packet is received.
    cout << "Payload pacchetto " << endl;
    for(int i=0;i<MAX_LENGTH_PAYLOAD;i++)
    {
        cout<<uwApph->payload_msg[i];
    }
    //cout<<endl;

    Packet::free(p);

/*    if (useDropOutOfOrder()) {
        if (pkts_lost + pkts_recv + pkts_last_reset != hrsn) {
            if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::recv() ---> Number of packets lost " << pkts_lost << ","
                    << " number of packets received " << pkts_recv << ","
                    << " highest sequence number received " << hrsn << std::endl;
        }
    }*/
}//end statistics method

void uwApplicationModule::start_generation() {
    //if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::start_generation() ---> Start the process to generate DATA packets." << std::endl;
    if(debug_ >= 1) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::START_GENERATION_DATA" << endl;
    chkTimerPeriod.resched(getTimeBeforeNextPkt());
} //end start_generation() method

void uwApplicationModule::init_Packet() {
    Packet* p = Packet::alloc();

    double delay = 0;

    hdr_cmn* ch = hdr_cmn::access(p);
    hdr_uwudp* uwudp = hdr_uwudp::access(p);
    hdr_uwip* uwiph = hdr_uwip::access(p);
    hdr_DATA_APPLICATION* uwApph = HDR_DATA_APPLICATION(p);

    //Common header fields
    ch->uid() = uidcnt++; //Increase the id of data packet
    ch->ptype_ = PT_DATA_APPLICATION; //Assign the type of packet that is being created
    ch->size() = PAYLOADSIZE; //Assign the size of data payload 
    ch->direction() = hdr_cmn::DOWN; //The packet must be forward at the level above of him

    //Transport header fields
    uwudp->dport() = PORT_NUM; //Set the destination port

    //IP header fields
    uwiph->daddr() = DST_ADDR; //Set the IP destination address

    //Application header fields
    incrPktSent();
    uwApph->sn_ = txsn; //Sequence number to the data packet

    if (rftt >= 0) {
        uwApph->rftt_ = (int) (rftt * 10000); //Forward Trip Time
        uwApph->rftt_valid_ = true;
    } else {
        uwApph->rftt_valid_ = false;
    }
    uwApph->priority_ = 0; //Priority of the message

    //Create the payload message
    if (getPayLoadSize() < MAX_LENGTH_PAYLOAD) {
        for (int i = 0; i < getPayLoadSize(); i++) {
            (*uwApph).payload_msg[i] = rand() % 26 + 'a';
        }
        for (int i = getPayLoadSize(); i < MAX_LENGTH_PAYLOAD; i++) {
            (*uwApph).payload_msg[i] = '0';
        }
    } else {
        for (int i = 0; i < MAX_LENGTH_PAYLOAD; i++) {
            (*uwApph).payload_msg[i] = rand() % 26 + 'a';
        }
    }

    //Show the DATA payload generated
    ch->timestamp() = Scheduler::instance().clock();

    if (debug_) std::cout << "" << std::endl;

    //Show some DATA packet information 
    if (debug_ >= 2) std::cout << "[" << getEpoch() << "]::" << NOW << "UWAPPLICATION::INIT_PKT_UID_" << ch->uid() << std::endl;
    if (debug_ >= 2) std::cout << "[" << getEpoch() << "]::" << NOW <<  "UWAPPLICATION::SERIAL_NUMBER_" << uwApph->sn() << std::endl;
    if (debug_ >= 2) std::cout << "[" << getEpoch() << "]::" << NOW <<  "UWAPPLICATION::DADDR_" << (uint)uwiph->daddr() << std::endl;
    if (debug_ >= 2) std::cout << "[" << getEpoch() << "]::" << NOW <<  "UWAPPLICATION::DPORT_" << (uint)uwudp->dport() << std::endl;
    if (debug_ >= 2) std::cout << "[" << getEpoch() << "]::" << NOW <<  "UWAPPLICATION::PAYLOAD_SIZE_" << sizeof (uwApph->payload_msg) << std::endl;

    sendDown(p, delay);
    chkTimerPeriod.resched(getTimeBeforeNextPkt()); // schedule next transmission
} //end init_Packet() method

void uwApplicationModule::stop() {
    if (withoutSocket()) {
        chkTimerPeriod.force_cancel();
    } else {
        //Close the connection
        if (useTCP()) {
            chkTimerPeriod.force_cancel();
            close(servSockDescr);
        }
    }
} //end stop() method

double uwApplicationModule::getTimeBeforeNextPkt() {
    if (getPeriod() < 0) {
        if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::getTimeBeforeNextPkt() ---> Error PERIOD < 0. Exit from the program." << std::endl;
        exit(1);
    }
    if (usePoissonTraffic()) {
        //Data packets are generated with a Poisson process
        double u = RNG::defaultrng()->uniform_double();
        double lambda = 1.0 / PERIOD;
        if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::getTimeBeforeNextPkt() ---> DATA packets are generated with a Poisson process."
                << " Generation rate is: " << lambda << std::endl;
        return (-log(u) / lambda);
    } else {
        //Data packets are generated wit a costant bit rate
        if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::getTimeBeforeNextPkt() ---> DATA packets are generated with constant bit rate."
                << " Generation rate is: " << PERIOD << "[s]" << std::endl;
        return PERIOD;
    }
} //end getTimeBeforeNextPkt() Method

double uwApplicationModule::GetRTT() const {
    return (rttsamples > 0) ? sumrtt / rttsamples : 0;
} //end GetRTT() method

double uwApplicationModule::GetRTTstd() const {
    if (rttsamples > 1) {
        double var = (sumrtt2 - (sumrtt * sumrtt / rttsamples)) / (rttsamples - 1);
        return (sqrt(var));
    } else
        return 0;
} //end GetRTTstd() method

void uwApplicationModule::updateRTT(const double& rtt) {
    if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::updateRTT() ---> Update RTT." << std::endl;
    sumrtt += rtt;
    sumrtt2 += rtt*rtt;
    rttsamples++;
} //end updateRTT() method

double uwApplicationModule::GetFTT() const {
    return (fttsamples > 0) ? sumftt / fttsamples : 0;
} //end getFTT() method

double uwApplicationModule::GetFTTstd() const {
    if (fttsamples > 1) {
        double var = 0;
        var = (sumftt2 - (sumftt * sumftt / fttsamples)) / (fttsamples - 1);
        if (var > 0)
            return (sqrt(var));
        else return 0;
    } else {
        return 0;
    }
} //end getFTT() method

double uwApplicationModule::GetPER() const {
    if (DROP_OUT_OF_ORDER) {
        if ((pkts_recv + pkts_lost) > 0) {
            return ((double) pkts_lost / (double) (pkts_recv + pkts_lost));
        } else {
            return 0;
        }
    } else {
        if (esn > 1)
            return (1 - (double) pkts_recv / (double) (esn - 1));
        else
            return 0;
    }
} //end getPER() method

double uwApplicationModule::GetTHR() const {
    return ((sumdt != 0) ? sumbytes * 8 / sumdt : 0);
}//end GetTHR() method

void uwApplicationModule::updateFTT(const double& ftt) {
    sumftt += ftt;
    sumftt2 += ftt*ftt;
    fttsamples++;
}//end updateFTT() method

void uwApplicationModule::updateThroughput(const int& bytes, const double& dt) {
    sumbytes += bytes;
    sumdt += dt;
    if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::updateThroughput ---> bytes= " << bytes << " dt=" << dt << std::endl;
}//end updateThroughput() method




void uwApplicationModule::uwSendTimerAppl::expire(Event* e) {
    if (m_->debug_) std::cout << "Time: " << NOW << " uwSendTimer::expire() -->Period timeout expired." << std::endl;
    if (m_->withoutSocket()) {
        //Communication take placing without socket 
        if (m_->usePatternSequence()) {
            //Using a pattern sequence for data payload
            // m_-> initialize_DATA_Pck_Pattern();
        } else {
            //Using a random sequence for data payload
            m_->init_Packet();
        }
    } else {
        //Communication take placing with sockets 
        if (m_->useTCP()) {
            //The protocol that the system is using is TCP 
            m_->init_Packet_TCP();
            m_->chkTimerPeriod.resched(m_->PERIOD);
        } else {
            //The protocol that the system is using is UDP 
            m_->init_Packet_UDP();
            m_->chkTimerPeriod.resched(m_->PERIOD);
        }
    }
} //end expire();
