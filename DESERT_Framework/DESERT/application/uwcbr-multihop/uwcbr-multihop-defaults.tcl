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
# @file   uwcbr-defaults.tcl
# @author Giovanni Toso
# @version 1.1.0

load libuwcbr.so

PacketHeaderManager set tab_(PacketHeader/UWCBRMH) 1

Module/UW/CBRMH_SRC set PoissonTraffic_     1
Module/UW/CBRMH_SRC set debug_              0
Module/UW/CBRMH_SRC set destAddr_           0
Module/UW/CBRMH_SRC set destPort_           0
Module/UW/CBRMH_SRC set drop_out_of_order_  1
Module/UW/CBRMH_SRC set dupack_thresh       2
Module/UW/CBRMH_SRC set packetSize_         500
Module/UW/CBRMH_SRC set period_             60
Module/UW/CBRMH_SRC set rx_window           1
Module/UW/CBRMH_SRC set timeout_            60
Module/UW/CBRMH_SRC set traffic_type_       0
Module/UW/CBRMH_SRC set tx_window           1
Module/UW/CBRMH_SRC set use_arq             0
Module/UW/CBRMH_SRC set use_rtt_timeout     1

Module/UW/CBRMH_SINK set PoissonTraffic_     1
Module/UW/CBRMH_SINK set debug_              0
Module/UW/CBRMH_SINK set destAddr_           0
Module/UW/CBRMH_SINK set destPort_           0
Module/UW/CBRMH_SINK set drop_out_of_order_  1
Module/UW/CBRMH_SINK set dupack_thresh       2
Module/UW/CBRMH_SINK set packetSize_         500
Module/UW/CBRMH_SINK set period_             60
Module/UW/CBRMH_SINK set rx_window           1
Module/UW/CBRMH_SINK set timeout_            60
Module/UW/CBRMH_SINK set traffic_type_       0
Module/UW/CBRMH_SINK set tx_window           1
Module/UW/CBRMH_SINK set use_arq             0
Module/UW/CBRMH_SINK set use_rtt_timeout     1

Module/UW/CBRMH_RELAY set debug_           0
Module/UW/CBRMH_RELAY set dupack_thresh    2

# Module/UW/CBRMHSRC instproc init {args} {
#    $self next $args
#    $self settag "UW/CBRMH"
# }

# Module/UW/CBRMHSNK instproc init {args} {
#    $self next $args
#    $self settag "UW/CBRMH"
# }
