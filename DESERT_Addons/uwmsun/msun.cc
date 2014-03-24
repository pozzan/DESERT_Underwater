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

/**
 * @file   msun.cc
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Implements a MSun.
 * 
 */

#include "msun.h"

int msun::uid_ = 0;

/**
 * Adds the module for MSun in ns2.
 */
static class MSunModuleClass : public TclClass {
public:
    MSunModuleClass() : TclClass("Module/UW/MSUN") {
    }

    TclObject* create(int, const char*const*) {
        return (new MSun());
    }
} class_module_msun;

// Constructor for MSun class
MSun::MSun()
:
// Variables
ipAddr_(0),
metric_(msun::HOPCOUNT),
delay_status_(0.1),
delay_data_(1),
printDebug_(0),
timer_search_path_enabled_(true),
timer_error_packets_enabled_(true),
disable_path_error_(0),
disable_route_error_(0),
min_sinr_path_request_(0),
// Acks
num_retx_(3),
// Broadcast
restricted_broadcast_(0),
ttl_(10),
maximum_cache_time_(60),
// Sink
enable_sink_(0),
// Buffer
buffer_max_size_(10),
adaptive_timer_buffer_(0),
alpha_data_(1),
wait_path_answer_(0),
enable_tweak_retx_(0),
prevent_from_drop_(0),
max_buffer_tmr_(60),
min_buffer_tmr_(1),
// Timers
timer_route_validity_(600),
timer_buffer_(3),
timer_search_path_(60),
timer_answer_path_(20),
timer_error_packets_(20),
bufferTmr_(this),
searchPathTmr_(this),
broadcastDuplicatePacketsTimer_(this),
errorPacketsTimer_(this),
// Trace
trace_(false),
trace_path_(false),
trace_file_name_(NULL),
trace_file_path_name_(NULL),
trace_separator_('\t'),
// Statistics
num_pathest_search_tx_(0),
num_pathest_search_fw_(0),
num_pathest_search_rx_(0),
num_pathest_search_me_(0),
num_pathest_answer_tx_(0),
num_pathest_answer_fw_(0),
num_pathest_answer_rx_(0),
num_pathest_answer_me_(0),
num_data_tx_(0),
num_data_fw_(0),
num_data_rx_(0),
num_data_me_(0),
num_data_acked_(0),
num_data_stored_(0),
num_ack_tx_(0),
num_ack_rx_(0),
num_ack_me_(0),
num_error_tx_(0),
num_error_fw_(0),
num_error_rx_(0),
num_error_me_(0),
num_drop_buff_full_(0),
num_drop_maxretx_(0)
{
    if (msun::STACK_TRACE)
        std::cout << "> MSun()" << std::endl;

    bind("metric_", &metric_);
    bind("delay_status_", &delay_status_);
    bind("delay_data_", &delay_data_);
    bind("printDebug_", &printDebug_);
    bind("disable_path_error_", &disable_path_error_);
    bind("disable_route_error_", &disable_route_error_);
    bind("min_sinr_path_request_", &min_sinr_path_request_);
    bind("num_retx_", &num_retx_);
    bind("restricted_broadcast_", &restricted_broadcast_);
    bind("ttl_", &ttl_);
    bind("maximum_cache_time_", &maximum_cache_time_);
    bind("buffer_max_size_", &buffer_max_size_);
    bind("adaptive_timer_buffer_", &adaptive_timer_buffer_);
    bind("alpha_data_", &alpha_data_);
    bind("wait_path_answer_", &wait_path_answer_);
    bind("enable_tweak_retx_", &enable_tweak_retx_);
    bind("prevent_from_drop_", &prevent_from_drop_);
    bind("max_buffer_tmr_", &max_buffer_tmr_);
    bind("min_buffer_tmr_", &min_buffer_tmr_);
    bind("timer_route_validity_", &timer_route_validity_);
    bind("timer_buffer_", &timer_buffer_);
    bind("timer_search_path_", &timer_search_path_);
    bind("timer_answer_path_", &timer_answer_path_);
    bind("timer_error_packets_", &timer_error_packets_);
//    srand((unsigned) time(0));
    buffer_data.reserve(buffer_max_size_);
    buffer_data_forward.reserve(buffer_max_size_);
    bufferTmr_.resched(timer_buffer_);
} /* MSun::MSun */

// Destructor for MSun class
MSun::~MSun() {
    if (msun::STACK_TRACE)
        std::cout << "> ~MSun()" << std::endl;
    
    delete[] trace_file_name_;
    delete[] trace_file_path_name_;
    if (buffer_data.size() > 0) {
        for (std::vector<buffer_element>::iterator it = buffer_data.begin(); it != buffer_data.end(); ++it) {
            Packet::free(it->p_);
        }
    }
    if (buffer_data_forward.size() > 0) {
        for (std::vector<buffer_element>::iterator it = buffer_data_forward.begin(); it != buffer_data_forward.end(); ++it) {
            Packet::free(it->p_);
        }
    }
} /* MSun::~MSun */


int MSun::recvSyncClMsg(ClMessage* m) {
    if (msun::STACK_TRACE)
        std::cout << "> recvSyncClMsg()" << std::endl;

    return Module::recvSyncClMsg(m);
} /* MSun::recvSyncClMsg */

int MSun::recvAsyncClMsg(ClMessage* m) {
    if (msun::STACK_TRACE)
        std::cout << "> recvAsyncClMsg()" << std::endl;

    if (m->type() == UWIP_CLMSG_SEND_ADDR) {
        UWIPClMsgSendAddr* m_ = reinterpret_cast<UWIPClMsgSendAddr*>(m);
        ipAddr_ = m_->getAddr();
    }
    return Module::recvAsyncClMsg(m);
} /* MSun::recvAsyncClMsg */

int MSun::command(int argc, const char*const* argv) {
    if (msun::STACK_TRACE)
        std::cout << "> command()" << std::endl;

    Tcl& tcl = Tcl::instance();

    if (argc == 2) {
        if (strcasecmp(argv[1], "initialize") == 0) {
            this->initialize();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "enable_sink") == 0) {
            enable_sink_ = 1;
            return TCL_OK;
        } else if (strcasecmp(argv[1], "disable_sink") == 0) {
            enable_sink_ = 0;
            return TCL_OK;
        } else if (strcasecmp(argv[1], "clearroutingtable") == 0) {
            this->clearRoutingTable();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "clearbuffer") == 0) {
            this->clearBuffer();
            this->clearBufferForwarded();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "printlowesthopcount") == 0) {
            this->printLowestHopCount();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "printroutingtable") == 0) {
            this->printRoutingTable();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getackheadersize") == 0) {
            tcl.resultf("%d", this->getAckHeaderSize());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getdatapktheadersize") == 0) {
            tcl.resultf("%d", this->getDataPktHeaderSize());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getpathestheadersize") == 0) {
            tcl.resultf("%d", this->getPathEstHeaderSize());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathsearchtx") == 0) {
            tcl.resultf("%u", this->getNumPathestSearchTx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathsearchfw") == 0) {
            tcl.resultf("%u", this->getNumPathestSearchFw());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathsearchrx") == 0) {
            tcl.resultf("%u", this->getNumPathestSearchRx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathsearchme") == 0) {
            tcl.resultf("%u", this->getNumPathestSearchMe());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathanswertx") == 0) {
            tcl.resultf("%u", this->getNumPathestAnswerTx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathanswerfw") == 0) {
            tcl.resultf("%u", this->getNumPathestAnswerFw());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathanswerrx") == 0) {
            tcl.resultf("%u", this->getNumPathestAnswerRx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumpathanswerme") == 0) {
            tcl.resultf("%u", this->getNumPathestAnswerMe());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdatatx") == 0) {
            tcl.resultf("%u", this->getNumDataTx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdatafw") == 0) {
            tcl.resultf("%u", this->getNumDataFw());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdatarx") == 0) {
            tcl.resultf("%u", this->getNumDataRx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdatame") == 0) {
            tcl.resultf("%u", this->getNumDataMe());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdataacked") == 0) {
            tcl.resultf("%u", this->getNumDataAcked());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdatastored") == 0) {
            tcl.resultf("%u", this->getNumDataStored());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumacktx") == 0) {
            tcl.resultf("%u", this->getNumAckTx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumackrx") == 0) {
            tcl.resultf("%u", this->getNumAckRx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumackme") == 0) {
            tcl.resultf("%u", this->getNumAckMe());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumerrortx") == 0) {
            tcl.resultf("%u", this->getNumErrorTx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumerrorfw") == 0) {
            tcl.resultf("%u", this->getNumErrorFw());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumerrorrx") == 0) {
            tcl.resultf("%u", this->getNumErrorRx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumerrorme") == 0) {
            tcl.resultf("%u", this->getNumErrorMe());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdropbufffull") == 0) {
            tcl.resultf("%u", this->getNumDropBuffFull());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getnumdropmaxretx") == 0) {
            tcl.resultf("%u", this->getNumDropMaxRetx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getbufferssize") == 0) {
            tcl.resultf("%u", this->getBuffersSize());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getmeanretx") == 0) {
            tcl.resultf("%f", this->getMeanRetx());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "printidspkts") == 0) {
            this->printIdsPkts();
            return TCL_OK;
        }
    } else if (argc == 3) {
        if (strcasecmp(argv[1], "getstatstx") == 0) {
            uint8_t addr_sink = atoi(argv[2]);
//            nsaddr_t addr_sink = this->str2addr((char*) argv[2]);
            tcl.resultf("%s", this->getStatsTx(addr_sink).c_str());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "getstatsrx") == 0) {
            uint8_t addr_sink = atoi(argv[2]); // this->str2addr((char*) argv[2]);
//            nsaddr_t addr_sink = this->str2addr((char*) argv[2]);
            tcl.resultf("%s", this->getStatsRx(addr_sink).c_str());
            return TCL_OK;
        } else if (strcasecmp(argv[1], "addr") == 0) {
            ipAddr_ = static_cast<uint8_t>(atoi(argv[2]));
            if (ipAddr_ == 0) {
                fprintf(stderr, "0 is not a valid IP address");
                return TCL_ERROR;
            }
            return TCL_OK;
        } else if (strcasecmp(argv[1], "trace") == 0) {
            if (argv[2] == NULL) {
                std::fprintf(stderr, "Empty string for the trace file name.");
                return TCL_ERROR;
            }
            string tmp_(argv[2]);
            trace_file_name_ = new char[tmp_.length() + 1];
            strcpy(trace_file_name_, tmp_.c_str());
            trace_ = true;
            // These two lines are used to create the file. It may be empty at the end of the simulation.
            trace_file_.open(trace_file_name_);
            trace_file_.close();
            return TCL_OK;
        } else if (strcasecmp(argv[1], "tracepaths") == 0) {
            if (argv[2] == NULL) {
                fprintf(stderr, "Empty string for the trace file name");
                return TCL_ERROR;
            }
            string tmp_(argv[2]);
            trace_file_path_name_ = new char[tmp_.length() + 1];
            strcpy(trace_file_path_name_, tmp_.c_str());
            trace_path_ = true;
            // These two lines are used to create the file. It may be empty at the end of the simulation.
            trace_file_path_.open(trace_file_path_name_);
            trace_file_path_.close();
            return TCL_OK;
        }
    }
    return Module::command(argc, argv);
} /* MSun::command */

void MSun::recv(Packet* p) {
    if (msun::STACK_TRACE)
        std::cout << "> recv()" << std::endl;
    hdr_cmn* ch                         = HDR_CMN(p);
    hdr_uwip* iph                       = HDR_UWIP(p);
    hdr_msun_data* hdata                = HDR_MSUN_DATA(p);
    hdr_msun_broadcastdata* hbcastdata  = HDR_MSUN_BROADCASTDATA(p);
    hdr_msun_path_est* hpest            = HDR_MSUN_PATH_EST(p);
    hdr_uwcbr* uwcbrh                   = HDR_UWCBR(p);
    
    if (!ch->error()) { // Process the packet only if it is free from errors.
        double delay_tx_ = this->getDelay(delay_data_);
        if (trace_)
            this->tracePacket(p, "RECV_PKT");
        if (ch->direction() == hdr_cmn::UP) {
            // Drop a packet received from the lower layers if the source is the node itself. This check is redundant, it should be done by the IP layer.
            if (iph->saddr() == ipAddr_) {
                if (trace_)
                    this->tracePacket(p, "DROP_OBM");
                drop(p, 1, ORIGINATED_BY_ME);
                return;
            }
            if (\
                (ch->next_hop() & 0x000000ff) == (ipAddr_ & 0x000000ff) || \
                (ch->next_hop() & 0x000000ff) == (UWIP_BROADCAST & 0x000000ff) || \
                (iph->daddr() & 0x000000ff) == (ipAddr_ & 0x000000ff) || \
                (iph->daddr() & 0x000000ff) == (UWIP_BROADCAST & 0x000000ff)) { // Right destination.
            //if ((ch->next_hop() == UWIP_BROADCAST) || (ch->next_hop() == ipAddr_) || (iph->daddr() == UWIP_BROADCAST) || (iph->daddr() == ipAddr_)) { // Redundant check already made in UWIP (just in case).
                if (ch->ptype() == PT_MSUN_PATH_EST) { // Path Establishment Packet.
                    if (hpest->ptype() == msun::PATH_SEARCH) { // Path Search.
                        num_pathest_search_rx_++;
                        if (iph->saddr() == ipAddr_) { // Garbage: the node received a request from itself.
                            Packet::free(p);
                            return;
                        } else {
                            if (trace_)
                                this->tracePacket(p, "RECV_SRC");
                            this->forwardPathEstSearch(p->copy());
                            
                            /* If the node participates in a Path Request resched,
                             * and the destination of it is the same of the first
                             * packet in the buffer resched the possibility to send
                             * a new Path Request to the same destination.
                             */
                            std::vector<buffer_element>* buffer;
                            buffer = &buffer_data;
                            if (!buffer->empty()) {
                                buffer_element _tmp = buffer->front();
                                hdr_uwip* iph_tmp       = HDR_UWIP(_tmp.p_);
                                if ((iph_tmp->daddr() & 0x000000ff) == (iph->daddr() & 0x000000ff)) {
                                    timer_search_path_enabled_ = false;
                                    searchPathTmr_.resched(timer_search_path_);
                                }
                            }
                            buffer = &buffer_data_forward;
                            if (!buffer->empty()) {
                                buffer_element _tmp = buffer->front();
                                hdr_uwip* iph_tmp       = HDR_UWIP(_tmp.p_);
                                if ((iph_tmp->daddr() & 0x000000ff) == (iph->daddr() & 0x000000ff)) {
                                    timer_search_path_enabled_ = false;
                                    searchPathTmr_.resched(timer_search_path_);
                                }
                            }
                            Packet::free(p);
                            return;
                        }
                    } else if (hpest->ptype() == msun::PATH_ANSWER) { // Path Answer.
                        num_pathest_answer_rx_++;
                        if (iph->saddr() == ipAddr_) { // Garbage: the node received an answer from itself.
                            Packet::free(p);
                            return;
                        } else {
                            if (iph->daddr() == ipAddr_) { // The packet is arrived to the destination.
                                num_pathest_answer_me_++;
                                if (trace_)
                                    this->tracePacket(p, "RECV_PTH");
                                this->evaluatePath(p);
                                Packet::free(p);
                                return;
                            } else {
                                this->forwardPathEstAnswer(p->copy());
                                
                                /* If the node participates in a Path Request resched,
                                 * and the destination of it is the same of the first
                                 * packet in the buffer resched the possibility to send
                                 * a new Path Request to the same destination.
                                 */
                               std::vector<buffer_element>* buffer;
                               buffer = &buffer_data;
                               if (!buffer->empty()) {
                                   buffer_element _tmp = buffer->front();
                                   hdr_uwip* iph_tmp       = HDR_UWIP(_tmp.p_);
                                   if ((iph_tmp->daddr() & 0x000000ff) == (iph->daddr() & 0x000000ff)) {
                                       timer_search_path_enabled_ = false;
                                       searchPathTmr_.resched(timer_search_path_);
                                   }
                               }
                               buffer = &buffer_data_forward;
                               if (!buffer->empty()) {
                                   buffer_element _tmp = buffer->front();
                                   hdr_uwip* iph_tmp       = HDR_UWIP(_tmp.p_);
                                   if ((iph_tmp->daddr() & 0x000000ff) == (iph->daddr() & 0x000000ff)) {
                                       timer_search_path_enabled_ = false;
                                       searchPathTmr_.resched(timer_search_path_);
                                   }
                               }
                               Packet::free(p);
                               return;
                            }
                        }
                    } else if (hpest->ptype() == msun::PATH_ERROR) {
                        num_error_rx_++;
                        if (iph->saddr() == ipAddr_) { // Garbage: the node received an error from itself.
                            Packet::free(p);
                            return;
                        } else { // Path error: reset the route information and send back the route error packet.
                            if (trace_)
                                this->tracePacket(p, "RECV_ERR");
                            // Update the routing table of the node.
                            this->updateRoutingTableAfterError(p);
                            
                            // If the current node is not the destination forward the packet.
                            if (iph->daddr() != ipAddr_) {
                                this->sendRouteErrorBack(p->copy());
                            } else {
                                num_error_me_++;
                            }
                            Packet::free(p);
                            return;
                        }
                    } else {
                        Packet::free(p);
                        return;
                    }
                } else if (ch->ptype() == PT_MSUN_ACK) { // Ack Packet.
                    num_ack_rx_++;
                    if (iph->saddr() == ipAddr_) { // Garbage: the node received an ack from itself.
                        Packet::free(p);
                        return;
                    } else if (iph->daddr() == ipAddr_) {
                        num_ack_me_++;
                        if (trace_) {
                            this->tracePacket(p, "RECV_ACK");
                        }
                        // Process the ack packet: update the buffer.
                        this->processAck(p);

                        Packet::free(p);
                        return;
                    } else {
                        Packet::free(p);
                        return;
                    }

                } else { //ch->ptype_ == Data packet.
                    num_data_rx_++;
                    if (trace_)
                        this->tracePacket(p, "RECV_DTA");
                    if (iph->daddr() == UWIP_BROADCAST) { // Broadcast packet: sendUp if the current node is enabled as sink and forward it.
                        // SendUp procedures
                        if (enable_sink_) {
                            std::map<uint8_t, std::vector<int> >::iterator it = data_rx_.find(iph->saddr());
                            if (it != data_rx_.end()) {
                                it->second[hdata->list_of_hops_length()]++;
                            } else {
                                std::vector<int> tmp_(msun::MAX_HOP_NUMBER + 1, 0);
                                tmp_[hdata->list_of_hops_length()] = 1;
                                data_rx_.insert(pair<uint8_t, std::vector<int> >(iph->saddr(), tmp_));
                            }
                            ch->size() -= sizeof(hdr_msun_broadcastdata);
                            num_data_me_++;
                            if (trace_)
                                this->tracePacket(p, "SDUP_DTA");
                            if(trace_path_)
                                this->writePathInTrace(p);
                            sendUp(p->copy());
                            ch->size() += sizeof(hdr_msun_broadcastdata);
                        }

                        // SendDown procedures
                        hbcastdata->ttl()--;
                        ch->prev_hop_ = ipAddr_;
                        if (hbcastdata->ttl() <= 0) { // Check the ttl
                            if (trace_)
                                this->tracePacket(p, "DROP_TEZ");
                            drop(p, 1, TTL_EQUALS_TO_ZERO);
                            return;
                        }

                        map_forwarded_packets::iterator it2 = my_forwarded_packets_.find(iph->saddr());
                        if (it2 != my_forwarded_packets_.end()) {
                            map_packets::iterator it3 = it2->second.find(ch->uid());
                            
                            if (it3 == it2->second.end()) { // Known source and new packet -> add it to the map and forward.
                                it2->second.insert(std::pair<uint16_t, double>(ch->uid(), ch->timestamp()));
                                num_data_tx_++;
                                num_data_fw_++;
                                if (trace_)
                                    this->tracePacket(p, "FRWD_DTA");
                                sendDown(p, delay_tx_);
                                return;
                            } else if (Scheduler::instance().clock() - it3->second > maximum_cache_time_) { // Packet already processed by not valid maximum cache timer -> update the cache time and forward.
                                it3->second = Scheduler::instance().clock();
                                num_data_tx_++;
                                num_data_fw_++;
                                if (trace_)
                                    this->tracePacket(p, "FRWD_DTA");
                                sendDown(p, delay_tx_);
                                return;
                            } else {
                                if (trace_)
                                    this->tracePacket(p, "DROP_FRW");
                                Packet::free(p);
                                return;
                            }
                        } else { // Unknown source.
                            std::map<uint16_t, double> tmp_map;
                            tmp_map.insert(std::pair<uint16_t, double>(ch->uid(), Scheduler::instance().clock()));
                            my_forwarded_packets_.insert(std::pair<uint8_t, map_packets>(iph->saddr(), tmp_map));
                            num_data_tx_++;
                            num_data_fw_++;
                            if (trace_)
                                this->tracePacket(p, "FRWD_DTA");
                            sendDown(p, delay_tx_);
                            return;
                        }
                    } else if (iph->daddr() == ipAddr_) { // The packet is arrived at destination: send it up.
                        // Send back an ack
                        this->sendBackAck(p);
                        
                        ch->size() -= sizeof(hdr_msun_data);
                        num_data_me_++;
                        std::map<uint8_t, std::vector<int> >::iterator it = data_rx_.find(iph->saddr());
                        if (it != data_rx_.end()) {
                            it->second[hdata->list_of_hops_length()]++;
                        } else {
                            std::vector<int> tmp_(msun::MAX_HOP_NUMBER + 1, 0);
                            tmp_[hdata->list_of_hops_length()] = 1;
                            data_rx_.insert(pair<uint8_t, std::vector<int> >(iph->saddr(), tmp_));
                        }
                        if (trace_)
                            this->tracePacket(p, "SDUP_DTA");
                        if(trace_path_)
                            this->writePathInTrace(p);
                        sendUp(p->copy());
                        Packet::free(p);
                        return;
                    } else if ((ch->next_hop() == ipAddr_) && (iph->daddr() != ipAddr_)) { // The node is a relay.
                        // Send back an ack
                        this->sendBackAck(p); // Send back an ack.
                        
                        /*
                         * Buffer the packet only if it is not duplicated. Useful to avoid high overhead, but reduces the reliability.
                         */
                        bool packet_to_insert_ = true;
                        for (std::vector<buffer_element>::iterator it = buffer_data_forward.begin(); it != buffer_data_forward.end(); ++it) {
                            Packet* packet_to_check_      = it->p_;
                            hdr_cmn* ch_to_check          = HDR_CMN(packet_to_check_);
                            hdr_uwip* iph_to_check        = HDR_UWIP(packet_to_check_);
                            if ((iph->daddr() & 0x000000ff) == (iph_to_check->daddr()) && (ch->uid() == ch_to_check->uid())) {
                                packet_to_insert_ = false;
                            }
                        }
                        
                        if ((buffer_data_forward.size() < buffer_max_size_) && packet_to_insert_) {
                            num_data_stored_++;
                            /*
                             * Set the number of retx of the new packet.
                             */
                            int tmp_num_retx_ = 0;
                            if (enable_tweak_retx_ != 0) {
                                tmp_num_retx_ = - this->getPlusNumRetx(p); // Negative number.
                            }

                            /*
                             *
                             * Check for the priority.
                             * If priority == 0 -> push the packet back.
                             * If priority == 1 -> push the packet after the last packet with priority 1.
                             */
                            if (uwcbrh->priority() == 1) {
                                /*
                                 * If the buffer is empty or
                                 * the buffer is not empyt and the first element has priority == 0
                                 * -> insert the packet at the front of the buffer.
                                 */
                                if (buffer_data_forward.empty() || (!buffer_data_forward.empty() && HDR_UWCBR(buffer_data_forward[0].p_)->priority_ == 0)) {
                                    buffer_data_forward.insert(buffer_data_forward.begin(), buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, tmp_num_retx_, msun::MSUN_UNICAST, iph->saddr(), true));
                                }
                                /*
                                 * Search for the last element with priority 1. Insert here the new packet. At the first occurrence of an element with priority 0 break.
                                 */
                                else { 
                                    for (std::vector<buffer_element>::iterator itv(buffer_data_forward.begin()); itv != buffer_data_forward.end(); ++itv) {
                                        if (HDR_UWCBR(itv->p_)->priority_ == 0) { // Insert the element at this position and break.
                                            buffer_data_forward.insert(itv, buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, tmp_num_retx_, msun::MSUN_UNICAST, iph->saddr(), true));
                                            break;
                                        }
                                    }
                                }
                            } else {
                                buffer_data_forward.push_back(buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, tmp_num_retx_, msun::MSUN_UNICAST, iph->saddr(), true));
                            }
                            Packet::free(p);
                        } else if ((buffer_data_forward.size() < buffer_max_size_) && packet_to_insert_ == false) {
                            if (trace_)
                                this->tracePacket(p, "DROP_DDP");
                            drop(p, 1, DROP_DUPLICATED_DATA_PKT);
                        } else {
                            num_drop_buff_full_++;
                            if (trace_)
                                this->tracePacket(p, "DROP_BIF");
                            drop(p, 1, DROP_BUFFER_IS_FULL);
                        }
                        return;
                    } else { // Some error with the destination, this check should be done by the IP module.
                        Packet::free(p);
                        return;
                    }
                }
            } else { // Wrong destination.
                if (trace_)
                    this->tracePacket(p, "DROP_PNM");
                drop(p, 1, DROP_PACKET_NOT_FOR_ME);
                return;
            }
        } else if (ch->direction() == hdr_cmn::DOWN) { // The packet is arrived from the upper layers.
            iph->saddr() = ipAddr_;
            if (trace_)
                this->tracePacket(p, "DATA_APP");
            if (iph->daddr() == UWIP_BROADCAST) { // Packet to BROADCAST
                if (restricted_broadcast_) {
                    uint8_t ip_sink_ = 0;
                    if (metric_ == msun::HOPCOUNT) {
                        ip_sink_ = this->ipLowestHopCountPath();
                    } else if (metric_ == msun::SNR) {
                        ip_sink_ = this->ipMaxMinSNRPath();
                    }

                    if (ip_sink_ == 0) { // No path to any sink -> send a request.
                        if (timer_search_path_enabled_) {
                            this->searchPath(iph->daddr());
                            timer_search_path_enabled_ = false;
                            searchPathTmr_.resched(timer_search_path_ + delay_status_);
                        }
                        bcast_queue_.push(p->copy());
                        broadcastDuplicatePacketsTimer_.resched(timer_search_path_);
                        Packet::free(p);
                        return;
                    } else {
                        for (std::map<uint8_t, route_container>::const_iterator it(routing_table.begin()); it != routing_table.end(); ++it) { // Create one copy of the packet for each destination.
                            if (this->hcToIp(it->first) > 0) {
                                // Alloc a new packet, set the destination and store it.
                                iph->daddr() = it->first;
                                ch->size()   += sizeof(hdr_msun_data); // Add the size before to push the data packet in the buffer.
                                if (buffer_data.size() < buffer_max_size_) { // There is space to store the packet.
                                    num_data_stored_++;
                                    if (uwcbrh->priority() == 1) {
                                        /*
                                         * If the buffer is empty or
                                         * the buffer is not empyt and the first element has priority == 0
                                         * -> insert the packet at the front of the buffer.
                                         */
                                        if (buffer_data.empty() || (!buffer_data.empty() && HDR_UWCBR(buffer_data[0].p_)->priority_ == 0)) {
                                            buffer_data.insert(buffer_data.begin(), buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_UNICAST, iph->saddr(), true));
                                        }
                                        /*
                                         * Search for the last element with priority 1. Insert here the new packet. At the first occurrence of an element with priority 0 break.
                                         */
                                        else { 
                                            for (std::vector<buffer_element>::iterator itv(buffer_data.begin()); itv != buffer_data.end(); ++itv) {
                                                if (HDR_UWCBR(itv->p_)->priority_ == 0) { // Insert the element at this position and break.
                                                    buffer_data.insert(itv, buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_UNICAST, iph->saddr(), true));
                                                    break;
                                                }
                                            }
                                        }
                                    } else {
                                        buffer_data.push_back(buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_UNICAST, iph->saddr(), true));
                                    }
                                } else {
                                    num_drop_buff_full_++;
                                    if (trace_)
                                        this->tracePacket(p, "DROP_BIF");
                                    drop(p, 1, DROP_BUFFER_IS_FULL); // If the buffer is full, drop the original packet and stop with the process of generating packets.
                                    return;
                                }
                            }
                        }
                    }
                    Packet::free(p);
                    return;
                } else {
                    ch->prev_hop_           = ipAddr_;
                    ch->next_hop()          = UWIP_BROADCAST;
                    ch->size()              += sizeof(hdr_msun_broadcastdata);
                    hbcastdata->ttl()       = ttl_;
                    num_data_tx_++;
                    if (trace_)
                        this->tracePacket(p, "SEND_DTA");
                    sendDown(p, delay_tx_);
                    return;
                }
            } else if (iph->daddr() == 0) { // Packet to the best: ANYCAST.
                if (buffer_data.size() < buffer_max_size_) { // There is space to buffer the packet.
                    num_data_stored_++;
                    ch->size() += sizeof(hdr_msun_data); // Add the size before to push the data packet in the buffer.
                    if (uwcbrh->priority() == 1) {
                        /*
                         * If the buffer is empty or
                         * the buffer is not empyt and the first element has priority == 0
                         * -> insert the packet at the front of the buffer.
                         */
                        if (buffer_data.empty() || (!buffer_data.empty() && HDR_UWCBR(buffer_data[0].p_)->priority_ == 0)) {
                            buffer_data.insert(buffer_data.begin(), buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_ANYCAST, iph->saddr(), true));
                        }
                        /*
                         * Search for the last element with priority 1. Insert here the new packet. At the first occurrence of an element with priority 0 break.
                         */
                        else { 
                            for (std::vector<buffer_element>::iterator itv(buffer_data.begin()); itv != buffer_data.end(); ++itv) {
                                if (HDR_UWCBR(itv->p_)->priority_ == 0) { // Insert the element at this position and break.
                                    buffer_data.insert(itv, buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_ANYCAST, iph->saddr(), true));
                                    break;
                                }
                            }
                        }
                    } else {
                        buffer_data.push_back(buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_ANYCAST, iph->saddr(), true));
                    }
                    Packet::free(p);
                } else {
                    num_drop_buff_full_++;
                    if (trace_)
                        this->tracePacket(p, "DROP_BIF");
                    drop(p, 1, DROP_BUFFER_IS_FULL);
                }
                return;
            } else { // Packet to a specific destination: UNICAST.
                if (buffer_data.size() < buffer_max_size_) { // There is space to buffer the packet.
                    num_data_stored_++;
                    ch->size() += sizeof(hdr_msun_data); // Add the size before to push the data packet in the buffer.
                    if (uwcbrh->priority() == 1) {
                        /*
                         * If the buffer is empty or
                         * the buffer is not empyt and the first element has priority == 0
                         * -> insert the packet at the front of the buffer.
                         */
                        if (buffer_data.empty() || (!buffer_data.empty() && HDR_UWCBR(buffer_data[0].p_)->priority_ == 0)) {
                            buffer_data.insert(buffer_data.begin(), buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_UNICAST, iph->saddr(), true));
                        }
                        /*
                         * Search for the last element with priority 1. Insert here the new packet. At the first occurrence of an element with priority 0 break.
                         */
                        else {
                            for (std::vector<buffer_element>::iterator itv(buffer_data.begin()); itv != buffer_data.end(); ++itv) {
                                if (HDR_UWCBR(itv->p_)->priority_ == 0) { // Insert the element at this position and break.
                                    buffer_data.insert(itv, buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_UNICAST, iph->saddr(), true));
                                    break;
                                }
                            }
                        }
                    } else {
                        buffer_data.push_back(buffer_element(p->copy(), ch->uid(), Scheduler::instance().clock(), 0, 0, msun::MSUN_UNICAST, iph->saddr(), true));
                    }
                    Packet::free(p);
                } else {
                    num_drop_buff_full_++;
                    if (trace_)
                        this->tracePacket(p, "DROP_BIF");
                    drop(p, 1, DROP_BUFFER_IS_FULL);
                }
                return;
            }
        } else {
            if (trace_)
                this->tracePacket(p, "DROP_PWD");
            drop(p, 1, DROP_PACKET_WITHOUT_DIRECTION);
            return;
        }
    }
} /* MSun::recv */
