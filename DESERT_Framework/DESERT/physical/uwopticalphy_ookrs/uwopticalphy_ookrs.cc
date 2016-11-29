//
// Copyright (c) 2015 Regents of the SIGNET lab, University of Padova.
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
 * @file   uwoptical.cc
 * @author Federico Favaro, Federico Guerra, Filippo Campagnaro
 * @version 1.0.0
 *
 * \brief Implementation of UwOptical class.
 *
 */

#include "uwopticalphy_ookrs.h"
#include <node-core.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

const double K = 1.38064852e-23; ///< Boltzmann constant
const double q = 1.60217662e-19; ///< Electron charge

int binomial_rv(int n, double p) {
  if (p > 0.5) return n - binomial_rv(n, 1-p);
  double u = RNG::defaultrng()->uniform();
  double c = p / (1-p);
  double pr = pow(1-p, n);
  if (pr == 0) throw runtime_error("Would loop endlessly");
  double F = pr;
  int i = 0;
  while (u >= F) {
    pr *= c * (n-i) / (i+1);
    F += pr;
    i++;
  }
  return i;
}

static class UwOpticalPhyOOKRSClass : public TclClass 
{
public:
  UwOpticalPhyOOKRSClass() : TclClass("Module/UW/OPTICAL/PHY_OOKRS") {
  }
    
  TclObject* create(int, const char*const*) {
    return (new UwOpticalPhyOOKRS);
  }
} class_module_optical_ookrs;

bool UwOpticalPhyOOKRS::initialized = false;
int UwOpticalPhyOOKRS::modid = -1;

void UwOpticalPhyOOKRS::initialize() {
  if (!initialized)
    {
      modid = registerModulationType(OPTICAL_OOKRS_MOD_TYPE);
      initialized = true;
    }
}

UwOpticalPhyOOKRS::UwOpticalPhyOOKRS() :
  lut_file_name_(""),
  lut_token_separator_('\t'),
  use_woss_(false)
{
  initialize();

  bind("BitRate_", &BitRate_);

  bind("Id_",&Id);
  bind("Il_",&Il);
  bind("R_",&R);
  bind("S_",&S);
  bind("T_",&T);
  bind("Ar_",&Ar_);

  bind("use_reed_solomon", &use_reed_solomon);
  bind("rs_n", &rs_n);
  bind("rs_k", &rs_k);
}

UwOpticalPhyOOKRS::~UwOpticalPhyOOKRS() {
}

int UwOpticalPhyOOKRS::getModulationType(Packet*)
{
  return modid;
}

double UwOpticalPhyOOKRS::getTxDuration(Packet *p) {
  hdr_cmn* ch = HDR_CMN(p);
  int totsize = ch->size() + ch->fecsize();
  double txduration = totsize * 8.0 / BitRate_;
  assert(txduration > 0);
  return txduration;
}

int UwOpticalPhyOOKRS::command(int argc, const char*const* argv) {
  if (argc == 2) {
      if (strcasecmp(argv[1], "useLUT") == 0) {
        use_woss_ = false;
        initializeLUT();        
        return TCL_OK;
      }
      else if (strcasecmp(argv[1], "useWOSS") == 0) {
        use_woss_ = true;
        return TCL_OK;
      }
  }
  else if (argc == 3) {
    if (strcasecmp(argv[1], "setLUTFileName") == 0) 
      {
        string tmp_(argv[2]);
        if (tmp_.size() == 0) 
          {
            fprintf(stderr, "Empty string for the file name");
            return TCL_ERROR;
          }
        lut_file_name_ = tmp_;
        return TCL_OK;
      } 
    else if (strcasecmp(argv[1], "setLUTSeparator") == 0) 
      {
        string tmp_(argv[2]);
        if (tmp_.size() == 0) {
          fprintf(stderr, "Empty char for the file name");
          return TCL_ERROR;
        }
        lut_token_separator_ = tmp_.at(0);
        return TCL_OK;
      }
  }
  return MPhyChSense::command(argc, argv);
}

void UwOpticalPhyOOKRS::startTx(Packet* p)
{
  // we abort any ongoing rx activity
  PktRx = 0;
  txPending = true;
  sendDown(p);
}

void UwOpticalPhyOOKRS::endTx(Packet* p)
{
  txPending = false;
  // Notify the MAC
  Phy2MacEndTx(p);
}

void UwOpticalPhyOOKRS::startRx(Packet* p)
{
  if (PktRx != 0) {
    if (debug_) cout << "UwOpticalPhyOOKRS::Drop Packet::Synced onto another packet PktRx = " << PktRx << ", pending = " << txPending << endl;
    return;
  }

  if (txPending) {
    if (debug_) cout << "UwOpticalPhyOOKRS::Drop Packet::Tx Pending PktRx = " << PktRx << ", pending = " << txPending << endl;
    return;
  }

  hdr_MPhy* ph = HDR_MPHY(p);
  double snr_dB = getSNRdB(p);

  if (debug_) {
    cout << "UwOpticalPhyOOKRS::startRx snr_db = " << snr_dB << endl;
    cout << "UwOpticalPhyOOKRS::startRx Pn = " << ph->Pn << endl;
  }
	
  if (snr_dB <= getAcquisitionThresholdDb()) {
    if (debug_) cout << "UwOpticalPhyOOKRS::Drop Packet::Below Threshold : snrdb = " << snr_dB << ", threshold = " << getAcquisitionThresholdDb() << endl;
    return;
  }

  if (ph->modulationType == modid) {
    PktRx = p;
    Phy2MacStartRx(p);
  }
  else {
    if (debug_) cout << "UwOpticalPhyOOKRS::Drop Packet::Wrong modulation" << endl;
  }
}

double UwOpticalPhyOOKRS::getSNRdB(Packet* p)
{
  hdr_MPhy* ph = HDR_MPHY(p);
  return getSINRdB(ph->Pr, 0, ph->dstSpectralMask->getBandwidth(), ph->Pn);
}

double UwOpticalPhyOOKRS::getSINRdB(Packet *p) {
  hdr_MPhy* ph = HDR_MPHY(p);
  return getSINRdB(ph->Pr, ph->Pi, ph->dstSpectralMask->getBandwidth(), ph->Pn);
}

double UwOpticalPhyOOKRS::getSINRdB(double rx_power, double rx_interf_power,
                                    double bandwidth, double noise_power) {
  double rx_curr_2 = pow(rx_power * S, 2);
  double rx_interf_current_2 = pow(rx_interf_power * S, 2);
  double amb_noise_curr_2 = pow(noise_power * S, 2);
  double rec_noise_curr_2 = 2*q*(Id+Il)*bandwidth + 4*K*T*bandwidth/R;
  
  double snr_linear = rx_curr_2 / (amb_noise_curr_2 + rec_noise_curr_2 +
                                         rx_interf_current_2);
  assert(snr_linear >= 0);
  return snr_linear ? 10*log10(snr_linear) : -numeric_limits<double>::infinity();
}

double UwOpticalPhyOOKRS::getNoisePower(Packet* p)
{
  assert(!use_woss_);
  hdr_MPhy *ph = HDR_MPHY(p);
  Position* dest = ph->dstPosition;
  assert(dest);
  double depth = use_woss_ ? abs(dest->getAltitude()) : abs(dest->getZ());
  double lut_value = lut_map.empty() ? 0 : lookUpLightNoiseE(depth);
  return lut_value * Ar_;//right now returns 0, due to not bias the snr calculation with unexpected values
}
    
void UwOpticalPhyOOKRS::endRx(Packet* p)
{ 
  if (PktRx == 0) {
    Packet::free(p);
    return;
  }

  if (PktRx != p) {
    Packet::free(p);
    return;
  }

  setWrongBits(p);
  
  hdr_cmn* ch = HDR_CMN(p);
  hdr_MPhy *ph = HDR_MPHY(p);

  PktRx = 0;
  if (ch->error()) {
    Packet::free(p);
  }
  else {
    sendUp(p);
  }
}

void UwOpticalPhyOOKRS::recv(Packet *p) {
  hdr_cmn *ch = HDR_CMN(p);
  if (use_reed_solomon) {
    int blocks = ceil(ch->size() / (double) rs_k);
    int fecsize = blocks * rs_n;
    
    if(ch->direction() == hdr_cmn::UP) {
      assert(ch->fecsize() == fecsize);
    }
    else { // (ch->direction() == hdr_cmn::DOWN)
      assert(ch->fecsize() == 0); // Otherwise it must be counted in the blocks
      ch->fecsize() = fecsize;
    }
  }
  else {
    assert(ch->fecsize() == 0);
  }
  MPhyChSense::recv(p);
}

double UwOpticalPhyOOKRS::lookUpLightNoiseE(double depth)
{
  //TODO: search noise Energy in the lookup table
  double return_value_ = -1;
  DepthMap::iterator it = lut_map.lower_bound(depth);
  if (it != lut_map.end() && it->first == depth)
    {
      if (debug_) std::cout <<depth<< " "<<it->first << " " << it->second << std::endl; 
      return it->second;  
    }
  if (it == lut_map.end() || it == lut_map.begin())
    {
      if (debug_) std::cout <<depth<< " Nothing returned depth = " << depth << std::endl; 
    
      return NOT_FOUND_VALUE;
    }
  DepthMap::iterator u_it = it;
  it--;
  if (debug_)
    std::cout <<depth<< " "<<it->first << " " << it->second << " " 
              <<u_it->first << " " << u_it->second <<std::endl;
  return linearInterpolator(depth, it->first, it->second, u_it->first, u_it->second);
}

double UwOpticalPhyOOKRS::linearInterpolator( double x, double x1, double y1, double x2, double y2 )
{ 
  assert (x1 != x2);  
  double m = (y1-y2)/(x1-x2);
  double q = y1 - m * x1;
  return m * x + q;
}

void UwOpticalPhyOOKRS::initializeLUT()
{
  ifstream input_file_;
  string line_;
  char* tmp_ = new char[lut_file_name_.length() + 1];
  strcpy(tmp_, lut_file_name_.c_str());
  input_file_.open(tmp_);
  if (input_file_.is_open()) {
    // skip first 2 lines
    for (int i = 0; i < 2; ++i)
      {
        std::getline(input_file_, line_);
      }
    while (std::getline(input_file_, line_)) {
      //TODO: retrive the right row and break the loop
      ::std::stringstream line_stream(line_);
      double d;
      double n;
      line_stream >> d;
      line_stream >> n;
      lut_map[d] = n;
    }
  } else {
    cerr << "Impossible to open file " << lut_file_name_ << endl;
  }
}

double UwOpticalPhyOOKRS::bitErrorRateOOK(double snr) {
  return 0.5 * erfc(sqrt(snr) / (2*sqrt(2)));
}

void UwOpticalPhyOOKRS::setWrongBits(Packet *p) {
  double snr = getSNRdB(p);
  double sinr = getSINRdB(p);

  hdr_cmn *ch = HDR_CMN(p);
  hdr_MPhy *ph = HDR_MPHY(p);

  if (sinr == numeric_limits<double>::infinity()) {
    ch->error() = 0;
    ch->errbitcnt() = 0;
    return;
  }

  double sinr_lin = pow(10, sinr/10);
  double ber = bitErrorRateOOK(sinr_lin);

  if (debug_) cerr << "setWrongBits: snrdb=" << snr <<
                " sinrdb=" << sinr << endl;
  if (debug_) cerr << "setWrongBits: sinrlin=" << sinr_lin << endl;
  if (debug_) cerr << "setWrongBits: ber="<<ber<<endl;
    
  if (!use_reed_solomon) {
    ch->errbitcnt() = binomial_rv(ch->size(), ber);
    ch->error() = (ch->errbitcnt() > 0);
  }
  else {  
    int t = (rs_n - rs_k)/2;
    int nblocks = ceil(ch->size() / (double)rs_k);
    ch->errbitcnt() = 0;
    ch->error() = 0;
    for (int i = 0; i < nblocks; i++) {
      int blockerr = binomial_rv(rs_n, ber);
      ch->errbitcnt() += blockerr;
      if (blockerr > t) ch->error() = 1;
    }
  }
  if (debug_) cerr << "setWrongBits: errbitcnt="<<ch->errbitcnt()<<endl;
}

double UwOpticalPhyOOKRS::getNoiseChannel() {
  throw runtime_error("getNoiseChannel is not implemented!");
  
  assert(!use_woss_); // Does not look like it is implemented
  Position *dest = getPosition();
  assert(dest);
  double depth = use_woss_ ? abs(dest->getAltitude()) : abs(dest->getZ());
  double lut_value = lut_map.empty() ? 0 : lookUpLightNoiseE(depth);
  double bw = spectralmask_->getBandwidth();
  double receiver_noise = sqrt(2*q*(Id+Il)*bw + 4*K*T*bw/R) / S;
  return lut_value * Ar_ * S + receiver_noise;
}

// Local Variables:
// mode: c++
// indent-tabs-mode: nil
// c-basic-offset: 2
// End:
