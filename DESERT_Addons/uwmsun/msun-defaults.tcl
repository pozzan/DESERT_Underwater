#
# Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
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
# @file   msun-defaults.tcl
# @author Giovanni Toso
# @version 1.2.0

PacketHeaderManager set tab_(PacketHeader/MSUN_ACK)           1
PacketHeaderManager set tab_(PacketHeader/MSUN_DATA)          1
PacketHeaderManager set tab_(PacketHeader/MSUN_BROADCASTDATA) 1
PacketHeaderManager set tab_(PacketHeader/MSUN_PEST)          1

Module/UW/MSUN set metric_                  1
Module/UW/MSUN set delay_status_            0.1
Module/UW/MSUN set delay_data_              2
Module/UW/MSUN set printDebug_              0
Module/UW/MSUN set disable_path_error_      0
Module/UW/MSUN set disable_route_error_     0
Module/UW/MSUN set min_sinr_path_request_   0

Module/UW/MSUN set num_retx_                3

Module/UW/MSUN set restricted_broadcast_    0
Module/UW/MSUN set ttl_                     10
Module/UW/MSUN set maximum_cache_time_      60

Module/UW/MSUN set buffer_max_size_         100
Module/UW/MSUN set adaptive_timer_buffer_   0
Module/UW/MSUN set alpha_data_              0.8
Module/UW/MSUN set wait_path_answer_        0
Module/UW/MSUN set enable_tweak_retx_       0
Module/UW/MSUN set prevent_from_drop_       0
Module/UW/MSUN set max_buffer_tmr_          60
Module/UW/MSUN set min_buffer_tmr_          1


Module/UW/MSUN set timer_route_validity_    600
Module/UW/MSUN set timer_buffer_            3
Module/UW/MSUN set timer_search_path_       60
Module/UW/MSUN set timer_answer_path_       20
Module/UW/MSUN set timer_error_packets_     20

Module/UW/MSUN set debug_                   0

