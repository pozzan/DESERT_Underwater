/* -*-	Mode:C++ -*- */

/*
 * Copyright (c) 2015 Regents of the SIGNET lab, University of Padova.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University of Padova (SIGNET lab) nor the 
 *    names of its contributors may be used to endorse or promote products 
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   uwoptical-propagation.cc
 * @author Filippo Campagnaro, Federico Favaro, Federico Guerra
 * @version 1.0.0
 *
 * \brief Definition of UwOpticas class.
 *
 */

#include<node-core.h>
#include<iostream>
#include"uwoptical-mpropagation.h"
#include <math.h>

// constants initialization

//const string UwOpticalMPropagation::file_name_ = "optical_LUT.txt";

static class UwOpticalMPropagationClass : public TclClass {
public:
  UwOpticalMPropagationClass() : TclClass("Module/UW/OPTICAL/Propagation") {}
  TclObject* create(int, const char*const*) {
    return (new UwOpticalMPropagation);
  }
} class_UwOpticalMPropagation;


UwOpticalMPropagation::UwOpticalMPropagation()
:
MPropagation(),
/*file_name_("optical_LUT.txt"),
token_separator_('\t')*/
Ar_(1),
At_(1),
c_(0),
theta_(0),
omnidirectional_(false)
{
  /*bind_error("token_separator_", &token_separator_);*/
  bind("Ar_", &Ar_);
  bind("At_", &At_);
  bind("c_", &c_);
  bind("theta_", &theta_);
}

/*int UwOpticalMPropagation::command(int argc, const char*const* argv){
  if (argc == 3) {
    if (strcasecmp(argv[1], "setFileName") == 0) {
      string tmp_ = ((char *) argv[2]);
      if (tmp_.size() == 0) {
          fprintf(stderr, "Empty string for the file name");
          return TCL_ERROR;
      }
      file_name_ = tmp_;
      return TCL_OK;
    } 
    else if (strcasecmp(argv[1], "setSeparator") == 0) {
      string tmp_ = ((char *) argv[2]);
      if (tmp_.size() == 0) {
          fprintf(stderr, "Empty char for the file name");
          return TCL_ERROR;
      }
      token_separator_ = tmp_.at(0);
      return TCL_OK;
    }
  }
  return MPropagation::command(argc, argv);
}*/
int UwOpticalMPropagation::command(int argc, const char*const* argv){
  if (argc == 2) {
    if (strcasecmp(argv[1], "setOmnidirectional") == 0) {
      omnidirectional_ = true;
      return TCL_OK;
    } 
    else if (strcasecmp(argv[1], "setDirectional") == 0) {
      omnidirectional_ = false;
      return TCL_OK;
    } 
  }
  else if (argc == 3) {
    if (strcasecmp(argv[1], "setAr") == 0) {
      Ar_ = strtod(argv[2], NULL);
      return TCL_OK;
    } 
    else if (strcasecmp(argv[1], "setAt") == 0) {
      At_ = strtod(argv[2], NULL);
      return TCL_OK;
    }
    else if (strcasecmp(argv[1], "setC") == 0) {
      c_ = strtod(argv[2], NULL);
      return TCL_OK;
    }
    else if (strcasecmp(argv[1], "setTheta") == 0) {
      theta_ = strtod(argv[2], NULL);
      return TCL_OK;
    }
  }
  return MPropagation::command(argc, argv);
}

double UwOpticalMPropagation::getGain(Packet* p)
{//TODO: lookup table

   hdr_MPhy *ph = HDR_MPHY(p);

   Position* sp = ph->srcPosition;
   Position* rp = ph->dstPosition;
   assert(sp);
   assert(rp);
   double dist = sp->getDist(rp);
   double beta = sp->getRelZenith(rp);//mmm is it the elevation?? 
   double PCgain=getLambertBeerGain(dist,beta);
   if (debug_)
    std::cout << NOW << " UwOpticalMPropagation::getGain()" << " dist="
              << dist << " gain=" << PCgain << std::endl;

   return PCgain;
}

double UwOpticalMPropagation::getLambertBeerGain(double d, double beta){
  double cosBeta = omnidirectional_ ? 1 : cos(beta);
  double L = d / cosBeta; // TODO: verify this 
  return 2 * Ar_ * cosBeta / (M_PI * pow(L, 2.0) * (1 - cos(theta_)) + 2 * At_) 
         * exp(-c_*d);
}

/*double UwOpticalMPropagation::lookUpGain(double d, double angle){
	//TODO: search gain in the lookup table
  ifstream input_file_;
  istringstream stm;
  string line_;
  string token_;
  double return_value_;

  char* tmp_ = new char[file_name_.length() + 1];
  strcpy(tmp_, file_name_.c_str());
  input_file_.open(tmp_);
  if (input_file_.is_open()) {
    while (std::getline(input_file_, line_)) {
      //TODO: retrive the right row and break the loop
    }
  } else {
    cerr << "Impossible to open file " << file_name_ << endl;
  }
  istringstream iss(line_);
  while (getline(iss, token_, token_separator_)) {
    //TODO: find the right column
  }
  stm.str(token_);
  stm >> return_value_;
  //return return_value_;
	return 1.0;
}*/