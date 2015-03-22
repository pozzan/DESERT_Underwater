#
# Copyright (c) 2013 Regents of the SIGNET lab, University of Padova.
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
#
# In this script the new Interference Model can be tested. 
# This model permits to choose in wich manner you want take in account of 
# the interfering power in the reception of a packet. 
#
# 1. CHUNK mode will divide the packet into a number of chunks and the module will 
# compute the interfering power for each chunk. The packet will be considered correct only if
# each chunk is correct.
#
# 2. MEANPOWER instead, compute the average of the interfering power over the whole packet duration
# 
# The Physical layer is improved and implement brand new statistics, also on the packets lost
# and permits to detect the reason for packet dropping
#
#
# Author: Filippo Campagnaro
# Version: 1.0.0
#
# NOTE: tcl sample tested on Ubuntu 11.10, 64 bits OS
#
# Stack of the nodes
#   +-------------------------+
#   |  7. UW/ROV              |
#   +-------------------------+
#   |  6. UW/UDP              |
#   +-------------------------+
#   |  5. UW/STATICROUTING    |
#   +-------------------------+
#   |  4. UW/IP               |
#   +-------------------------+
#   |  3. UW/MLL              | it is an ARP resolve
#   +-------------------------+
#   |  2. UW/CSMA_ALOHA       |
#   +-------------------------+        +-------------------+
#   |  1. UW/PHYSICAL         | <----- |   UW/INTERFERENCE |
#   +-------------------------+        +-------------------+
#           |         |                          ^
#   +-------------------------+                  |
#   |    UnderwaterChannel    |-------------------
#   +-------------------------+

######################################
# Flags to enable or disable options #
######################################
set opt(verbose) 			1
set opt(trace_files)		1
set opt(bash_parameters) 	0
set opt(ACK_Active)         0

#####################
# Library Loading   #
#####################
load libMiracle.so
load libMiracleBasicMovement.so
load libmphy.so
load libmmac.so
load libuwip.so
load libuwstaticrouting.so
load libuwmll.so
load libuwudp.so
load libuwcbr.so
load libuwaloha.so
load libuwcsmaaloha.so
load libuwmac_select_phy.so
load libUwmStd.so
load libUwmStdPhyBpskTracer.so
load libuwinterference.so
#load libuwphysical.so

# NS-Miracle initialization #
#############################
# You always need the following two lines to use the NS-Miracle simulator
set ns [new Simulator]
$ns use-Miracle

##################
# Tcl variables  #
##################
set opt(start_clock) [clock seconds]

set opt(nn)                 2.0 ;# Number of Nodes

set opt(starttime)          1
set opt(stoptime)           3000
set opt(starttime2)          3010
set opt(stoptime2)           6000
set opt(txduration)         [expr $opt(stoptime) - $opt(starttime)] ;# Duration of the simulation

set opt(txpower)           185.8;#158.263 ;#Power transmitted in dB re uPa 185.8 is the maximum
set opt(propagation_speed) 1500;# m/s

set opt(maxinterval_)       200
set opt(freq)               375000.0 ;#Frequency used in Hz
set opt(bw)                  76000.0	;#Bandwidth used in Hz
set opt(bitrate)            87768.0 ;#150000;#bitrate in bps
set opt(freq2)               75000.0 ;#Frequency used in Hz
set opt(bw2)                 30000.0 ;#Bandwidth used in Hz
set opt(bitrate2)           4300.0 ;#150000;#bitrate in bps
set opt(pktsize)            525  ;# Pkt size in byte

set opt(cbr_period) 0.05
set opt(cbr_period2) 1

set rng [new RNG]
set rng_position [new RNG]
set opt(seedcbr)    1


set rnd_gen [new RandomVariable/Uniform]
$rnd_gen use-rng $rng
if {$opt(trace_files)} {
	set opt(tracefilename) "./test_uwrovmovement.tr"
	set opt(tracefile) [open $opt(tracefilename) w]
	set opt(cltracefilename) "./test_uwrovmovement.cltr"
	set opt(cltracefile) [open $opt(tracefilename) w]
} else {
	set opt(tracefilename) "/dev/null"
	set opt(tracefile) [open $opt(tracefilename) w]
	set opt(cltracefilename) "/dev/null"
	set opt(cltracefile) [open $opt(cltracefilename) w]
}

MPropagation/Underwater set practicalSpreading_ 2
MPropagation/Underwater set debug_              0
MPropagation/Underwater set windspeed_          10
MPropagation/Underwater set shipping_          1


set channel [new Module/UnderwaterChannel]
set propagation [new MPropagation/Underwater]

set data_mask [new MSpectralMask/Rect]
$data_mask setFreq       $opt(freq)
$data_mask setBandwidth  $opt(bw)
$data_mask setPropagationSpeed  $opt(propagation_speed)

set data_mask2 [new MSpectralMask/Rect]
$data_mask2 setFreq       $opt(freq2)
$data_mask2 setBandwidth  $opt(bw2)
$data_mask2 setPropagationSpeed  $opt(propagation_speed)

#########################
# Module Configuration  #
#########################
Module/UW/CBR set packetSize_          $opt(pktsize)
Module/UW/CBR set debug_               0

Module/MPhy/BPSK  set AcquisitionThreshold_dB_    15.0 
Module/MPhy/BPSK  set RxSnrPenalty_dB_            0
Module/MPhy/BPSK  set TxSPLMargin_dB_             0
Module/MPhy/BPSK  set TxPower_               $opt(txpower)
Module/MPhy/BPSK  set debug_                      0 


################################
# Procedure(s) to create nodes #
################################

proc createNode { id } {

    global channel ns cbr position node udp portnum ipr ipif
    global opt mll mac propagation data_mask data_mask2 interf_data
    
    set node($id) [$ns create-M_Node $opt(tracefile) $opt(cltracefile)] 
    if {$id == 0} {
        Module/UW/CBR set period_              $opt(cbr_period)
    } else {
        Module/UW/CBR set period_              $opt(cbr_period2)
    }
    for {set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
        set cbr($id,$cnt)  [new Module/UW/CBR] 
    }
    set udp($id)  [new Module/UW/UDP]
    set ipr($id)  [new Module/UW/StaticRouting]
    set ipif($id) [new Module/UW/IP]
    set mll($id)  [new Module/UW/MLL] 
    set mac($id)  [new Module/UW/MAC/SELECT_PHY] 
    #set mac($id)  [new Module/UW/CSMA_ALOHA] 

    Module/MPhy/BPSK/Underwater  set BitRate_    $opt(bitrate)
    set phy($id) [new Module/MPhy/BPSK/Underwater]

    Module/MPhy/BPSK/Underwater  set BitRate_    $opt(bitrate2)
    set phy2($id) [new Module/MPhy/BPSK/Underwater]

    for {set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
        $node($id) addModule 7 $cbr($id,$cnt)   1  "CBR"
    }
    $node($id) addModule 7 $udp($id)   1  "UDP"
    $node($id) addModule 6 $ipr($id)   1  "IPR"
    $node($id) addModule 5 $ipif($id)  1  "IPF"   
    $node($id) addModule 4 $mll($id)   1  "MLL"
    $node($id) addModule 3 $mac($id)   1  "MAC"
    $node($id) addModule 2 $phy($id)   0  "PHY"    
    $node($id) addModule 1 $phy2($id)   0  "PHY2"    

    for {set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
        $node($id) setConnection $cbr($id,$cnt)   $udp($id)   0
        set portnum($id,$cnt) [$udp($id) assignPort $cbr($id,$cnt) ]
    }
    $node($id) setConnection $udp($id)   $ipr($id)   1
    $node($id) setConnection $ipr($id)   $ipif($id)  1
    $node($id) setConnection $ipif($id)  $mll($id)   1
    $node($id) setConnection $mll($id)   $mac($id)   1
    $node($id) setConnection $mac($id)   $phy($id)   1
    $node($id) setConnection $mac($id)   $phy2($id)  1
    $node($id) addToChannel  $channel    $phy($id)   1
    $node($id) addToChannel  $channel    $phy2($id)  1

    #Set the IP address of the node
    $ipif($id) addr [expr $id + 1]
    
    set position($id) [new "Position/BM"]
    $node($id) addPosition $position($id)
    
    #Setup positions
    if { $id == 0 } {
        $position($id) setX_  0
        $position($id) setY_  0
        $position($id) setZ_ -100
    } else {
        $position($id) setX_  20
        $position($id) setY_  20
        $position($id) setZ_ -100
    }

    #Interference model
    set interf_data($id)  [new "Module/UW/INTERFERENCE"]
    $interf_data($id) set maxinterval_ $opt(maxinterval_)
    $interf_data($id) set debug_       0
    set interf_data2($id)  [new "Module/UW/INTERFERENCE"]
    $interf_data2($id) set maxinterval_ $opt(maxinterval_)
    $interf_data2($id) set debug_       0

    #Propagation model
    $phy($id) setPropagation $propagation
    $phy2($id) setPropagation $propagation
       
    $phy($id) setInterference $interf_data($id)
    $phy2($id) setInterference $interf_data2($id)
    $phy($id) setSpectralMask $data_mask
    $phy2($id) setSpectralMask $data_mask2

    set phy_id [$phy($id) Id_]
    set phy_id2 [$phy2($id) Id_]
    puts "id = $phy_id, id2 = $phy_id2 "
     if {$id == 0 } {
         $mac($id) setSendPhysicalId $phy_id
         $mac($id) setRecvPhysicalId $phy_id2
     } else {
         $mac($id) setSendPhysicalId $phy_id2
         $mac($id) setRecvPhysicalId $phy_id
     }
    #$mac($id) initialize
	
    #$mac($id) "setNoAckMode"
}

#################
# Node Creation #
#################
# Create here all the nodes you want to network together
for {set id 0} {$id < $opt(nn)} {incr id}  {
    createNode $id
}

################################
# Inter-node module connection #
################################
proc connectNodes {id1 des1} {
    global ipif ipr portnum cbr cbr_sink ipif_sink portnum_sink ipr_sink opt 
    $cbr($id1,$des1) set destAddr_ [$ipif($des1) addr]
    $cbr($id1,$des1) set destPort_ $portnum($des1,$id1)
}

##################
# Setup flows    #
##################
connectNodes 0 1
connectNodes 1 0

###################
# Fill ARP tables #
# ###################
 $mll(0) addentry [$ipif(0) addr] [$mac(0) addr]
 $mll(0) addentry [$ipif(1) addr] [$mac(1) addr]
 $mll(1) addentry [$ipif(1) addr] [$mac(1) addr]
 $mll(1) addentry [$ipif(0) addr] [$mac(0) addr]
# ########################
# # Setup routing tables #
# ########################
# $ipr(0) addRoute [$ipif(1) addr] [$ipif(1) addr]
# $ipr(1) addRoute [$ipif(0) addr] [$ipif(0) addr]



##################
# Routing tables #
##################
for {set id1 0} {$id1 < $opt(nn)} {incr id1}  {
	for {set id2 0} {$id2 < $opt(nn)} {incr id2}  {
			set ip_value [expr $id2 + 1]
            $ipr($id1) addRoute ${ip_value} ${ip_value}
	}
}


#####################
# Start/Stop Timers #
#####################
# Set here the timers to start and/or stop modules (optional)
# e.g., 
$ns at $opt(starttime)    "$cbr(0,1) start"
$ns at $opt(stoptime)     "$cbr(0,1) stop"

$ns at [expr $opt(starttime) + 1.1]    "$cbr(1,0) start"
$ns at $opt(stoptime)     "$cbr(1,0) stop"

###################
# Final Procedure #
###################
# Define here the procedure to call at the end of the simulation
proc finish {} {
    global ns opt outfile
    global mac propagation cbr_sink mac_sink phy_data phy_data_sink channel db_manager propagation
    global node_coordinates
    global ipr_sink ipr ipif udp cbr phy phy_data_sink
    global node_stats tmp_node_stats sink_stats tmp_sink_stats
    if ($opt(verbose)) {
        puts "---------------------------------------------------------------------"
        puts "Simulation summary"
        puts "number of nodes  : $opt(nn)"
        puts "packet size      : $opt(pktsize) byte"
        puts "cbr period       : $opt(cbr_period) s"
        puts "number of nodes  : $opt(nn)"
        puts "---------------------------------------------------------------------"
    }
    set sum_cbr_throughput     0
    set sum_cbr_sent_pkts      0.0
    set sum_cbr_rcv_pkts       0.0    
    set throughput(0) 0.0
    set throughput(1) 0.0
    set cbr_rcv_pkts_div(0) 0.0
    set cbr_rcv_pkts_div(1) 0.0
    set cbr_sent_pkts_div(0) 0.0
    set cbr_sent_pkts_div(1) 0.0

    for {set i 0} {$i < $opt(nn)} {incr i}  {
        set throughput($i) 0.0
        set cbr_rcv_pkts_div($i) 0.0
        set cbr_sent_pkts_div($i) 0.0
        for {set j 0} {$j < $opt(nn)} {incr j} {
            if {$i != $j} {
                set cbr_throughput           [$cbr($i,$j) getthr]
                set cbr_sent_pkts        [$cbr($i,$j) getsentpkts]
                set cbr_rcv_pkts           [$cbr($i,$j) getrecvpkts]
            } else {
                set cbr_throughput           0
                set cbr_sent_pkts           0
                set cbr_rcv_pkts           0
            }
            set cbr_rcv_pkts_div($i) [expr $cbr_rcv_pkts_div($i) + $cbr_rcv_pkts]
            set cbr_sent_pkts_div($i) [expr $cbr_sent_pkts_div($i) + $cbr_sent_pkts]
            set throughput($i) [expr $throughput($i) + $cbr_throughput]
        }
        
        set sum_cbr_throughput [expr $sum_cbr_throughput + $cbr_throughput]
        set sum_cbr_sent_pkts [expr $sum_cbr_sent_pkts + $cbr_sent_pkts]
        set sum_cbr_rcv_pkts  [expr $sum_cbr_rcv_pkts + $cbr_rcv_pkts]
    }
    set per_cbr1 [$cbr(1,0) getper]
    set per_cbr2 [$cbr(0,1) getper]
    
    if ($opt(verbose)) {
        puts "---------------------------------------------------------------------"
        puts "First node performance"
        puts "CBR1 Throughput          : $throughput(0)"
        puts "CBR1 sent packets          : $cbr_sent_pkts_div(0)"
        puts "CBR1 recv packets          : $cbr_rcv_pkts_div(0)"
        puts "---------------------------------------------------------------------"
        puts "Second node performance"
        puts "CBR2 Throughput          : $throughput(1)"
        puts "CBR2 sent packets          : $cbr_sent_pkts_div(1)"
        puts "CBR2 recv packets          : $cbr_rcv_pkts_div(1)"
        puts "---------------------------------------------------------------------"
        puts "PER CBR1                 : $per_cbr1 "
        puts "PER CBR2                 : $per_cbr2 "
    }
    
    $ns flush-trace
    close $opt(tracefile)
}


###################
# start simulation
###################
if ($opt(verbose)) {
    puts "\nStarting Simulation\n"
}


$ns at [expr $opt(stoptime) + 1500.0]  "finish; $ns halt" 

$ns run
