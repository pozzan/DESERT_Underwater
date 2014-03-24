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

int uwApplicationModule::openConnectionUDP() {
    int sockoptval = 1;
    if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionUDP() ---> Enable the server port number "<< servPort << "."
            <<"With this port Server offer a service for any client."<< std::endl;
    
    //Create socket for incoming connections
    if((servSockDescr=socket(AF_INET,SOCK_DGRAM,0)) < 0){
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionUDP() ---> Socket creation failed." << std::endl;
        exit(1);
    }
    if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionUDP() ---> Socket created."
            <<" Socket descriptor is "<< servSockDescr << "."<< std::endl;

    memset(&servAddr,0,sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(servPort);
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if(::bind(servSockDescr, (struct sockaddr *) &servAddr, sizeof (servAddr)) < 0) {
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionUDP() ---> Binding failed." << std::endl;
        printf("Errore: %s\n", strerror(errno));
        exit(1);
    }
    if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::openConnectionUDP() ---> Binding executed."<< std::endl;
    
    pthread_t pth;
    pthread_create(&pth, NULL, read_process_UDP, (void*) this);
    
    chkTimerPeriod.resched(getPeriod());   
    
    return servSockDescr;
}; //end openConnectionUDP() method

void *read_process_UDP(void* arg)
{
    int debug_=1;
    char buffer_msg[MAX_LENGTH_PAYLOAD]; //Buffer in which the DATA packets payload are stored
    int recvMsgSize; //Size of the DATA payload received;
    uwApplicationModule* obj = (uwApplicationModule*) arg; 
    
    socklen_t clnLen = sizeof(sockaddr_in);
    
    while(true) {
        clnLen = sizeof(obj->clnAddr);
        //Put the message to 0
        for (int i = 0; i < MAX_LENGTH_PAYLOAD; i++) {
                buffer_msg[i] = 0;
        }
        //Block until receive message from a client
        if( (recvMsgSize=recvfrom(obj->servSockDescr,buffer_msg,MAX_LENGTH_PAYLOAD,0,(struct sockaddr *)&(obj->clnAddr),&clnLen)) < 0 ) {
            if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::read_process_UDP() ---> Reception of DATA packet failed."<< std::endl;
        }

        //if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::read_process_UDP() ---> Handling client with IP address "<< inet_ntoa((obj->clnAddr).sin_addr) << std::endl;        
        
        if (recvMsgSize > 0)
        {
            Packet* p = Packet::alloc();
            hdr_DATA_APPLICATION* hdr_Appl = HDR_DATA_APPLICATION(p);
            for (int i = 0; i < MAX_LENGTH_PAYLOAD; i++) {
                hdr_Appl->payload_msg[i] = buffer_msg[i];
            }
            obj->queuePckReadUDP.push(p);
            obj->incrPktsPushQueue();
            if (debug_) std::cout << "Time: " << NOW << " uwApplicationModule::read_process_UDP() ---> Message saved in the queue."
                << " Number of DATA packets saved: " << obj->queuePckReadUDP.size() << std::endl;

        }
    }
    
}//end read_process_UDP() method

void uwApplicationModule::initialize_DATA_pck_wth_UDP(){
    if( queuePckReadUDP.empty() ) {
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::initialize_DATA_pck_wth_UDP() ---> There is no DATA packet to pass the below levels."<< std::endl;
    } else {
        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::initialize_DATA_pck_wth_UDP() ---> Start to initialize the fields of DATA packet."<< std::endl;
        
        Packet *ptmp = queuePckReadUDP.front();
        queuePckReadUDP.pop();
        hdr_cmn *ch = HDR_CMN(ptmp);
        hdr_uwudp *uwudph = hdr_uwudp::access(ptmp);
        hdr_uwip *uwiph = hdr_uwip::access(ptmp);
        hdr_DATA_APPLICATION* uwApph = HDR_DATA_APPLICATION(ptmp);
        
        ch->uid_ = uidcnt++;
        ch->ptype_ = PT_DATA_APPLICATION;
        ch->size_ = sizeof (ptmp);
        ch->direction_ = hdr_cmn::DOWN; 
        ch->timestamp() = Scheduler::instance().clock();
        
        uwudph->dport() = PORT_NUM; 
        
        uwiph->daddr() = DST_ADDR; 
        
        uwApph->sn_ = txsn++; //Sequence number to the data packet
        if (rftt >= 0) {
                uwApph->rftt_ = (int) (rftt * 10000); //Forward Trip Time
                uwApph->rftt_valid_ = true;
        } else {
                uwApph->rftt_valid_ = false;
        }
        uwApph->priority_ = 0; //Priority of the message

        if(debug_) std::cout << "Time: " << NOW << " uwApplicationModule::initialize_DATA_pck_wth_UDP() ---> DATA packet initialized. Identifier " << ch->uid() << ", Sequence Number " << uwApph->sn() << ","
         << " Destination address IP " << uwiph->daddr() << ", Destination port: " << uwudph->dport() << std::endl;
     
        sendDown(ptmp);
    }
}//end initialize_DATA_pck_wth_UDP() method