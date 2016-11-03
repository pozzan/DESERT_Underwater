#
# Copyright (c) 2015 Regents of the SIGNET lab, University of Padova.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the University of Padova (SIGNET lab) nor the 
#    names of its contributors may be used to endorse or promote products 
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# @file   uwmultiphy-default.tcl
# @author Filippo Campagnaro
# @version 1.0.0

#Module/UW/OPTICAL/PHY_OOKRS set TxPower_  				0.01
#Module/UW/OPTICAL/PHY_OOKRS set AcquisitionThreshold_dB_              1
Module/UW/OPTICAL/PHY_OOKRS set Id_                                   [expr 1.0e-9]
Module/UW/OPTICAL/PHY_OOKRS set Il_                                   [expr 1.0e-6]
Module/UW/OPTICAL/PHY_OOKRS set R_                                    [expr 1.49e9]
Module/UW/OPTICAL/PHY_OOKRS set S_                                    0.26
Module/UW/OPTICAL/PHY_OOKRS set T_                                    293.15
Module/UW/OPTICAL/PHY_OOKRS set Ar_                                   0.0000011
#Module/UW/OPTICAL/PHY_OOKRS set debug_                                0

Module/UW/OPTICAL/PHY_OOKRS set interference_threshold_               1
Module/UW/OPTICAL/PHY_OOKRS set use_reed_solomon                      0
Module/UW/OPTICAL/PHY_OOKRS set rs_n                                  1
Module/UW/OPTICAL/PHY_OOKRS set rs_k                                  1
