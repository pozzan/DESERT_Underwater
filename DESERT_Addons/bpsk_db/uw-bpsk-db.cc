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
 * @file   snr_db_retriever.cc
 * @author Giovanni Toso
 * @version 1.0.0
 * 
 * \brief Implementation of SnrDatabaseRetriever class.
 * 
 */

#include "uw-bpsk-db.h"
#include "uwip-module.h"

using namespace std;

static class UWMPhyBpskDbClass : public TclClass {
public:
  UWMPhyBpskDbClass() : TclClass("Module/UW/BPSKDB") {}
  TclObject* create(int, const char*const*) {
    return (new UnderwaterMPhyBpskDb);
  }
} class_UWMPhyBpskDb;

UnderwaterMPhyBpskDb::UnderwaterMPhyBpskDb() :
time_roughness_(1),
depth_roughness_(1),
distance_roughness_(1),
total_time_(1)
{
    bind("time_roughness_", &time_roughness_);
    bind("depth_roughness_", &depth_roughness_);
    bind("distance_roughness_", &distance_roughness_);
    bind("total_time_", &total_time_);
    bind_error("token_separator_", &token_separator_);
    token_separator_ = '\t';
    path_ = "";
}

int UnderwaterMPhyBpskDb::command(int argc, const char*const* argv) {
    if (argc == 3) {
        if (strcasecmp(argv[1], "path") == 0) {
            string tmp_ = ((char *) argv[2]);
            path_ = new char[tmp_.length() + 1];
            strcpy(path_, tmp_.c_str());
            if (path_ == NULL) {
                fprintf(stderr, "Empty string for the trace file name");
                return TCL_ERROR;
            }
            return TCL_OK;
        } 
    }
    return UnderwaterMPhyBpsk::command(argc, argv);
} /* UnderwaterMPhyBpskDb::command */

void UnderwaterMPhyBpskDb::endRx(Packet* p) {
    static int mac_addr = -1;

    if (debug_) {
        ClMsgPhy2MacAddr msg;
        sendSyncClMsg(&msg);
        mac_addr = msg.getAddr();
    }

    hdr_cmn* ch = HDR_CMN(p);
    hdr_MPhy* ph = HDR_MPHY(p);

    if (debug_) cout << NOW << "  MPhy_Bpsk(" << mac_addr << ")::endRx() start rx time = " << ph->rxtime << endl;

    if (PktRx != 0) {

        if (PktRx == p) {
            // We had synchronized onto this packet so we now try to see if
            // it has been received correctly
            if (debug_) cout << NOW << "  MPhy_Bpsk(" << mac_addr << ")::endRx() receiving sync pkt" << " dest " << HDR_CMN(p)->next_hop() << endl;

            double per_ni; // packet error rate due to noise and/or interference
            double per_n; // packet error rate due to noise only 

            int nbits = ch->size()*8;
            double x = RNG::defaultrng()->uniform_double();
            per_n = getPER(ph->Pr / ph->Pn, nbits, p);
            bool error_n = x <= per_n;
            bool error_ni = 0;

            if (!error_n) {
                if (interference_) {
                    const PowerChunkList& power_chunk_list = interference_->getInterferencePowerChunkList(p);

                    for (PowerChunkList::const_iterator itInterf = power_chunk_list.begin(); itInterf != power_chunk_list.end(); itInterf++) {
                        int nbits2 = itInterf->second * BitRate_;
                        per_ni = getPER(ph->Pr / (ph->Pn + itInterf->first), nbits2, p);
                        x = RNG::defaultrng()->uniform_double();
                        error_ni = x <= per_ni;
                        if (error_ni) {
                            break;
                        }
                    }
                } else {
                    per_ni = getPER(ph->Pr / (ph->Pn + ph->Pi), nbits, p);
                    error_ni = x <= per_ni;
                }
            }

            ch->error() = error_ni || error_n;

            if (error_n) {
                incrErrorPktsNoise();
            } else if (error_ni) {
                incrErrorPktsInterf();
            }
            sendUp(p);

            PktRx = 0; // We can now sync onto another packet
        } else {

            dropPacket(p);
        }
    } else {

        if (debug_) cout << NOW << "  MPhy_Bpsk(" << mac_addr << ")::endRx() not synced on any pkt, dropping rx pkt" << " dest " << HDR_CMN(p)->next_hop() << endl;
        dropPacket(p);
    }
}

double UnderwaterMPhyBpskDb::getPER(double _snr, int _nbits, Packet* p) {
    hdr_MPhy* ph = HDR_MPHY(p);
    
    static const double FROM_12_TO_25_KHZ = 1.000961057782496;
    double x_ = (ph->srcPosition)->getX();
    double y_ = (ph->srcPosition)->getY();
    double z_ = (ph->srcPosition)->getZ();
    double x_dst_ = (ph->dstPosition)->getX();
    double y_dst_ = (ph->dstPosition)->getY();
    double z_dst_ = (ph->dstPosition)->getZ();
    double depth_src_ = z_;
    double depth_dst_ = z_dst_;
    double dist_dst_ = sqrt((x_ - x_dst_)*(x_ - x_dst_) + (y_ - y_dst_)*(y_ - y_dst_));

    double gain_ = pow(10, (this->getGain(NOW, depth_src_, depth_dst_, dist_dst_) / 10));
    gain_ = gain_ * pow(FROM_12_TO_25_KHZ, - dist_dst_);
    //std::cout << ":GIN:" << 10*log10(gain_);
    
    double snr_;
    if ((ph->Pn + ph->Pi) != 0) {
        snr_ = (ph->Pt * gain_) / (ph->Pn + ph->Pi);
//        std::cout << ":DST:" << dist_dst_;
//        std::cout << ":SNR:" << snr_;
    } else {
        snr_ = - INT_MAX;
        cerr << "ph->Pn + ph->Pi = 0!" << endl;
    }
    
//    if (isZero(snr_)) {
//        cout << "distance:" << dist_dst_ << ":snr:" << "-INF" << endl;
//    } else {
//        cout << "distance:" << dist_dst_ << ":snr:" << 10*log10(snr_) << endl;
//    }
//    cout << "_snr: " << _snr << endl;
//    cout << "snr_: " << 10log10(snr_) << endl;
//    cout << "erfc:" << erfc(sqrt(snr_)) << endl;
//    cout << "ber:" << (0.5 * erfc(sqrt(snr_))) << endl;
//    double ber_fsk_ = 0.5 * exp(- snr_ / 2);
//    std::cout << ":BER:" << ber_fsk_;
//    std::cout << ":NIS:" << ph->Pn << std::endl;
//    std::cout << std::endl;
//    double per_fsk_ = 1 - pow(1 - ber_fsk_, _nbits);
//    std::cout << scientific << Scheduler::instance().clock() << '\t' << ch->uid() << '\t' << (ch->prev_hop_ & 0x000000ff) << '\t' << dist_dst_ << '\t'  << ber_fsk_  << '\t' << per_fsk_ << '\t' << snr_ << '\t' << gain_ << '\t' << ph->Pt << '\t' << ph->Pn << '\t' << ph->Pi << std::endl;
    return UnderwaterMPhyBpsk::getPER(snr_, _nbits);
}

void UnderwaterMPhyBpskDb::setTimeRoughness(const int& _time) {
    assert(_time >= 0);
    time_roughness_ = _time;
    return;
} /* UnderwaterMPhyBpskDb::setTimeRoughness */

void UnderwaterMPhyBpskDb::setDepthRoughness(const int& _depth) {
    assert(_depth >= 0);
    depth_roughness_ = _depth;
    return;
} /* UnderwaterMPhyBpskDb::setDepthRoughness */

void UnderwaterMPhyBpskDb::setDistanceRoughness(const int& _distance) {
    assert(_distance >= 0);
    distance_roughness_ = _distance;
    return;
} /* UnderwaterMPhyBpskDb::setDistanceRoughness */

void UnderwaterMPhyBpskDb::setTotalTime(const int& _total_time) {
    assert(_total_time > 0);
    total_time_ = _total_time;
    return;
} /* UnderwaterMPhyBpskDb::setTotalTime */

double UnderwaterMPhyBpskDb::getGain(const double& _time, const double& _source_depth, const double& _destination_depth, const double& _destination_distance) {
    assert(_time >= 0);
    assert(_source_depth <= 0);
    assert(_destination_depth <= 0);
    assert(_destination_distance >= 0);
    
    int time_filename_ = (int) ((int) ceil(_time) / ((int) time_roughness_) * (int) time_roughness_ % (int) total_time_);
    int source_depth_filename_ = (int) ((int) ceil(_source_depth * -1) / (int) depth_roughness_ * (int) depth_roughness_);
    if (source_depth_filename_ == 0)
        source_depth_filename_ = depth_roughness_;
    string file_name = this->createNameFile(time_filename_, source_depth_filename_);

    int line_index_ = (int) ((int) ceil(_destination_depth * -1) / (int) depth_roughness_);
    int column_index_ = (int) ((int) ceil(_destination_distance) / (int) distance_roughness_);
    double gain_ = this->retriveGainFromFile(file_name, line_index_, column_index_);

    return gain_;
} /* UnderwaterMPhyBpskDb::getSnr */

double UnderwaterMPhyBpskDb::retriveGainFromFile(const string& _file_name, const int& _row_index, const int& _column_index) const {
    int row_iterator_ = 0;
    int column_iterator_ = 0;
    ifstream input_file_;
    istringstream stm;
    string line_;
    string token_;
    string value_;
    double return_value_;

    char* tmp_ = new char[_file_name.length() + 1];
    strcpy(tmp_, _file_name.c_str());
    if (tmp_ == NULL) {
        fprintf(stderr, "Empty string for the file name");
    }

    input_file_.open(tmp_);
    if (input_file_.is_open()) {
        while (std::getline(input_file_, line_)) {
            row_iterator_++;
            if (row_iterator_ == _row_index) {
                break;
            }
        }
    } else {
        cerr << "Impossible to open file " << _file_name << endl;
    }

    istringstream iss(line_);
    while (getline(iss, token_, token_separator_)) {
        column_iterator_++;
        if (column_iterator_ == _column_index) {
            value_ = token_;
        }
    }

    stm.str(value_);
    stm >> return_value_;
//    cout << "file:" << _file_name << ":column:" << _column_index << ":row:" << _row_index << ":gain:" << return_value_ << endl;
    
    delete[] tmp_;
    if (this->isZero(return_value_)) {
        return (- INT_MAX);
    } else {
        return return_value_ ;
    }
} /* UnderwaterMPhyBpskDb::retriveSnrFromFile */

string UnderwaterMPhyBpskDb::createNameFile(const int& _time, const int& _source_depth) {
    osstream_.clear();
    osstream_.str("");
    osstream_ << path_ << "/" << _time << "_" << _source_depth << ".txt";
    return osstream_.str();
} /* UnderwaterMPhyBpskDb::createNameFile */
