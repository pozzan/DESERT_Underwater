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
#include <error.h>
#include <errno.h>

int uwApplicationModule::openConnectionTCP() {
    int sockoptval = 1;
    if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP():: Socket listening on port " << servPort << endl;
    
    //Create socket for incoming connections
    if((servSockDescr=socket(AF_INET,SOCK_STREAM,0)) < 0){
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP() ---> Socket creation failed." << std::endl;
        exit(1);
    }
    if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP() ---> Socket created." << endl;
    //setsockopt(servSockDescr, SOL_SOCKET, SO_REUSEADDR, &sockoptval, sizeof (int));
    
    //Fill the members of sockaddr_in structure
    memset(&servAddr,0,sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(servPort);
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    //Bind to the local address       
    if(::bind(servSockDescr, (struct sockaddr *) &servAddr, sizeof (servAddr)) < 0) {
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP() ---> Binding failed." << std::endl;
        printf("Errore: %s\n", strerror(errno));
        exit(1);
    }
    
    //Listen for incoming connections    
    if(listen(servSockDescr,1)) {
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP() ---> Listen failed." << std::endl;
        exit(1);
    }
    if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP() ---> Server ready."<< std::endl;
    
    chkTimerPeriod.resched(getPeriod());   
    pthread_t pth;
    if (pthread_create(&pth, NULL, read_process_TCP, (void*) this) != 0)
    {
        cerr << "Cannot create a parralel thread" << endl;
        exit(1);
    }
    
    return servSockDescr;
}//end openConnectionTCP() method

void *read_process_TCP(void* arg){
    uwApplicationModule* obj = (uwApplicationModule*) arg; 
    int debug_=1;
    //struct sockaddr_in clnAddr;
    
    socklen_t clnLen = sizeof(sockaddr_in);
    //int clnSockDescr;

    clnLen = sizeof(obj->clnAddr);


    while(true) {
        if( (obj->clnSockDescr=accept(obj->servSockDescr, (struct sockaddr *)&(obj->clnAddr), (socklen_t*) &clnLen)) < 0 ) {
            if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP() ---> Connection not accepted by the server."<< std::endl;
        }
        //if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionTCP() ---> NEW Client connected with IP address "<< inet_ntoa(obj->clnAddr.sin_addr)<<std::endl; 
        cout << "READ PROCESS ---> HANDLE TCP CLIENT!!! " << endl;
        obj->handleTCPclient(obj->clnSockDescr);
    }
    
}//end read_process_TCP() method

void uwApplicationModule::handleTCPclient(int clnSock)
{   
    while (true)
    {
        int recvMsgSize = 0;
        cout << "HANDLE TCP CLIENT!! " << endl;
        char buffer_msg[MAX_LENGTH_PAYLOAD];
        Packet* p = Packet::alloc();
        hdr_DATA_APPLICATION *hdr_Appl = HDR_DATA_APPLICATION(p);
        for(int i = 0; i < MAX_LENGTH_PAYLOAD; i++) {
            buffer_msg[i] = 0;
        }
        if ((recvMsgSize = read(clnSock, buffer_msg, MAX_LENGTH_PAYLOAD)) < 0) {
            if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::handleTCPclient() ---> Reception from the client is failed." << std::endl;
            break;
        }
        if (recvMsgSize == 0) { //client disconnected
            break;
        } else {
            cout << "BUFFER MESSAGE CONTENT " << endl;
            for(int i = 0; i < MAX_LENGTH_PAYLOAD; i++)
            {
                cout << buffer_msg[i];
            }
            for (int i = 0; i < MAX_LENGTH_PAYLOAD; i++) {
                hdr_Appl->payload_msg[i] = buffer_msg[i];
            }
            queuePckReadTCP.push(p);
            incrPktsPushQueue();
        }
    }
}


void uwApplicationModule::init_Packet_TCP(){
    if( queuePckReadTCP.empty() ) {
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::init_Packet_TCP() ---> Ther is no DATA packet to pass the below levels."<< std::endl;
    } else {
        //if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::init_Packet_TCP() ---> Start to initialize the fields of DATA packet."<< std::endl;
        
        //Packet *ptmp = Packet::alloc();
        cout << " Number of DATA packets saved: " << queuePckReadTCP.size() << std::endl;
        Packet* ptmp = queuePckReadTCP.front();
        queuePckReadTCP.pop();
        hdr_cmn *ch = HDR_CMN(ptmp);
        hdr_uwudp *uwudph = hdr_uwudp::access(ptmp);
        hdr_uwip *uwiph = hdr_uwip::access(ptmp);
        hdr_DATA_APPLICATION* uwApph = HDR_DATA_APPLICATION(ptmp);
        for (int i = 0; i < MAX_LENGTH_PAYLOAD; i++) {
            cout << uwApph->payload_msg[i];
        }

        //hdr_DATA_APPLICATION* pck_rcv_TCP = queuePckReadTCP.front();
        cout << endl;
        //Common header fields
        ch->uid_ = uidcnt++;
        ch->ptype_ = PT_DATA_APPLICATION;
        ch->size_ = 10;
        ch->direction_ = hdr_cmn::DOWN; 
        ch->timestamp() = Scheduler::instance().clock();
        
        //Transport header fields
        uwudph->dport() = PORT_NUM; 
        
        //IP header fields
        uwiph->daddr() = DST_ADDR; 
        
        //uwApplication packet header fields
        uwApph->sn_ = txsn++; //Sequence number to the data packet
        if (rftt >= 0) {
                uwApph->rftt_ = (int) ( rftt * 10000); //Forward Trip Time
                uwApph->rftt_valid_ = true;
        } else {
                uwApph->rftt_valid_ = false;
        }
        uwApph->priority_ = 0; //Priority of the message

        sendDown(ptmp);
    }
}//end init_Packet_TCP() method
