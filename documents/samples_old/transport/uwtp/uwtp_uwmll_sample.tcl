##-------------------------------------------------------------------------
# Author: Saiful Azad	
# Date: May 18, 2011
# Department of Information Engineering (DEI), University of Padova
#-------------------------------------------------------------------------
#
# Copyright (c) 2010 Regents of the SIGNET lab, University of Padova.
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

# BPSK + ALOHA sample


#	+-------------------------+	+-------------------------+
#	| 	7. CBR             |	| 	7. CBR             |
#	+-------------------------+	+-------------------------+
#	| 	6. Port            |	| 	6. Port            |
#	+-------------------------+	+-------------------------+
#	| 	5. IP Routing      |	| 	5. IP Routing      |
#	+-------------------------+	+-------------------------+
#	| 	4. IP Interface    |	| 	4.  IP Interface   |
#	+-------------------------+	+-------------------------+
#	| 	3. MLL Link Layer  |	| 	3. MLL Link Layer  |
#	+-------------------------+	+-------------------------+
#	| 	2. MMAC/ALOHA/ADV  |	| 	2. MMAC/ALOHA/ADV  |
#	+-------------------------+	+-------------------------+
#	| 1. MPHY/BPSK/Underwater |	| 1. MPHY/BPSK/Underwater |
#	+-------------------------+	+-------------------------+
#                    |                          |
#	+-----------------------------------------------------+
#      |                  UnderwaterChannel                  |
#      +-----------------------------------------------------+


# Module libraries

load libMiracle.so
load libmphy.so
load libMiracleBasicMovement.so

load libuwip.so
load libuwudp.so
load libuwtp.so
load libuwcbr.so
load libuwmll.so

load libuwmlltracer.so
load libroutingtracer.so
load libcbrtracer.so

load libUwmStd.so
load libWOSS.so
load libWOSSPhy.so

load libcsmaaloha.so


set ns [new Simulator]
$ns use-Miracle

######################################
# global allocations & misc options
######################################

set opt(start_clock) [clock seconds]

set opt(start_lat)  	 	43.0625
set opt(start_long)    	 	9.3095
set opt(month)		 	7
set opt(nn)		 	[lindex $argv 2]
set opt(max_dist)		1500.0
set opt(dmax)		 	1496.663
set opt(dist_x)		 	1600
set opt(dist_y)		 	1600
set opt(pktsize)	 	125
set opt(cbr_period)	 	[lindex $argv 1]
set opt(stoptime)        	50000
set opt(ack_mode)	 	"setAckMode"
set opt(usr_arq_mode)		"sr_train"
set opt(usr_tx_mode)		"RTT"
set opt(mac_ack_mode)		"setNoAckMode"
set opt(port_ack_mode)		"setAckMode"
set opt(ack_tx_mode) 		"setNoCumAckMode"

set rng [new RNG]

if {$argc != 3} {
  puts "The aloha.tcl script requires three numbers to be inputed. one for cbr period and another for seed and one more for nodes"
  puts "For example, ns aloha.tcl 4 100 1"
  puts "Please try again."
} else { 
$rng seed 			[lindex $argv 0]
set opt(rep_num)	 	[lindex $argv 0]
}

set opt(cbrpr) [expr int($opt(cbr_period))]
set opt(rnpr)  [expr int($opt(rep_num))]
set opt(apr)   "rtt"

if {$opt(usr_tx_mode) == "PD"} {
   set opt(apr)  "pd" 
}

set opt(starttime)       	0.1
set opt(txduration)     	[expr $opt(stoptime) - $opt(starttime)]

set opt(txpower)	 	200.0
set opt(per_tgt)	 	0.1
set opt(rx_snr_penalty_db)	-10.0
set opt(tx_margin_db)		10.0

set opt(node_min_angle)		-90.0
set opt(node_max_angle)		90.0
set opt(node_bathy_offset)	-2.0

set opt(maxinterval_)    	10.0
set opt(freq) 			25000.0
set opt(bw)              	9000.0
set opt(bitrate)	 	4800.0
set opt(geom_freq) [expr sqrt( ( $opt(freq) - $opt(bw) / 2.0 ) * ( $opt(freq) + $opt(bw) / 2.0 ) ) ]

set opt(t_time)	 	 	0.2222

# set opt(tracefilename) "/tmp/${argv0}.tr"
set opt(tracefilename) "/dev/null"
set opt(tracefile) [open $opt(tracefilename) w]

# set opt(cltracefilename) "/tmp/${argv0}.cltr"
set opt(cltracefilename) "/dev/null"
set opt(cltracefile) [open $opt(cltracefilename) w]

set outfile [open "./node_${opt(nn)}_${opt(apr)}_usr_aimd.out" a+]

set opt(path) "./${opt(apr)}${opt(cbrpr)}${opt(month)}${opt(nn)}${opt(rep_num)}"
set opt(db_path) "/home/azad/Desktop/simulation/ocean_database/nc_files"
set opt(db_res_path) "./month_${opt(month)}"

exec mkdir -p $opt(path)
exec mkdir -p $opt(db_res_path)

###################################
#Random Number Generators
####################################

global def_rng
set def_rng [new RNG]
$def_rng default

set opt(xmin)		0.0
set opt(ymin)		0.0
set opt(zmin)		1.0
set opt(zmax)		80.0

set positionrng [new RNG]

set rvposx [new RandomVariable/Uniform]
$rvposx set min_ $opt(xmin)
$rvposx set max_ $opt(dist_x)
$rvposx use-rng $positionrng

set rvposy [new RandomVariable/Uniform]
$rvposy set min_ $opt(ymin)
$rvposy set max_ $opt(dist_y)
$rvposy use-rng $positionrng

set rvposz [new RandomVariable/Uniform]
$rvposz set min_ $opt(zmin)
$rvposz set max_ $opt(zmax)
$rvposz use-rng $positionrng

for {set k 0} {$k < $opt(rep_num)} {incr k} {
     $def_rng next-substream
}


########################################
# global setup
########################################

WOSS/Definitions/RandomGenerator/NS2 set rep_number_ $opt(rep_num)
WOSS/Definitions/RandomGenerator/C   set seed_       $opt(rep_num)

set ssp_creator         [new "WOSS/Definitions/SSP"]
set sediment_creator    [new "WOSS/Definitions/Sediment"]
set pressure_creator    [new "WOSS/Definitions/Pressure"]
set time_arr_creator    [new "WOSS/Definitions/TimeArr"]
set time_reference      [new "WOSS/Definitions/TimeReference/NS2"]
set transducer_creator  [new "WOSS/Definitions/Transducer"]
set rand_generator      [new "WOSS/Definitions/RandomGenerator/NS2"]
# set rand_generator      [new "WOSS/Definitions/RandomGenerator/C"]
$rand_generator initialize

set def_handler [new "WOSS/Definitions/Handler"]
$def_handler setSSPCreator         $ssp_creator
$def_handler setSedimentCreator    $sediment_creator
$def_handler setPressureCreator    $pressure_creator
$def_handler setTimeArrCreator     $time_arr_creator
$def_handler setTransducerCreator  $transducer_creator
$def_handler setTimeReference      $time_reference
$def_handler setRandomGenerator    $rand_generator


WOSS/Utilities set debug 0
set woss_utilities [new WOSS/Utilities]


WOSS/Creator/Database/Binary/Results/TimeArr set debug           0
WOSS/Creator/Database/Binary/Results/TimeArr set woss_db_debug   0
WOSS/Creator/Database/Binary/Results/TimeArr set space_sampling  0

set db_res_arr [new WOSS/Creator/Database/Binary/Results/TimeArr]
$db_res_arr setDbPathName "${opt(db_res_path)}/arr_nn${opt(nn)}_distx${opt(dist_x)}_disty${opt(dist_y)}_freq${opt(geom_freq)}_rng${opt(rep_num)}.dat"


WOSS/Creator/Database/NetCDF/Sediment/DECK41 set debug         0
WOSS/Creator/Database/NetCDF/Sediment/DECK41 set woss_db_debug 0

set db_sedim [new WOSS/Creator/Database/NetCDF/Sediment/DECK41]
$db_sedim setUpDeck41CoordinatesDb  "${opt(db_path)}/sea_floor/DECK41_coordinates.nc"
$db_sedim setUpDeck41MarsdenDb      "${opt(db_path)}/sea_floor/DECK41_mardsen_square.nc"
$db_sedim setUpDeck41MarsdenOneDb   "${opt(db_path)}/sea_floor/DECK41_mardsen_one_degree.nc"


WOSS/Creator/Database/NetCDF/SSP/WOA2005/MonthlyAverage set debug          0
WOSS/Creator/Database/NetCDF/SSP/WOA2005/MonthlyAverage set woss_db_debug  0

set db_ssp [new WOSS/Creator/Database/NetCDF/SSP/WOA2005/MonthlyAverage]
$db_ssp setDbPathName "${opt(db_path)}/ssp/standard_depth/2WOA2005_SSP_[$woss_utilities convertMonth $opt(month)].nc"


WOSS/Creator/Database/NetCDF/Bathymetry/GEBCO set debug           0
WOSS/Creator/Database/NetCDF/Bathymetry/GEBCO set woss_db_debug   0

set db_bathy [new WOSS/Creator/Database/NetCDF/Bathymetry/GEBCO]
$db_bathy setDbPathName      "${opt(db_path)}/bathymetry/gebco_08.nc"
$db_bathy useThirtySecondsPrecision


WOSS/Database/Manager/V2 set debug 0
set db_manager [new WOSS/Database/Manager/V2]


WOSS/Creator/Bellhop set debug                        0.0
WOSS/Creator/Bellhop set woss_debug                   0.0
WOSS/Creator/Bellhop set max_time_values              20
WOSS/Creator/Bellhop set total_runs                   1
WOSS/Creator/Bellhop set frequency_step               0.0
WOSS/Creator/Bellhop set total_range_steps            20.0
WOSS/Creator/Bellhop set tx_min_depth_offset          0.0
WOSS/Creator/Bellhop set tx_max_depth_offset          0.0
WOSS/Creator/Bellhop set total_transmitters           1
WOSS/Creator/Bellhop set total_rx_depths              1
WOSS/Creator/Bellhop set rx_min_depth_offset          -1.0
WOSS/Creator/Bellhop set rx_max_depth_offset          1.0
WOSS/Creator/Bellhop set total_rx_ranges              2
WOSS/Creator/Bellhop set rx_min_range_offset          -1.0
WOSS/Creator/Bellhop set rx_max_range_offset          1.0
WOSS/Creator/Bellhop set total_rays                   0.0
WOSS/Creator/Bellhop set min_angle                    -45.0
WOSS/Creator/Bellhop set max_angle                    45.0
WOSS/Creator/Bellhop set ssp_depth_precision          1.0e-8
WOSS/Creator/Bellhop set normalized_ssp_depth_steps   100000

set woss_creator [new "WOSS/Creator/Bellhop"]
$woss_creator setWorkDirPath        "${opt(path)}/woss_channel_"
$woss_creator setBellhopPath        ""
$woss_creator setBellhopMode        0 0 "A"
$woss_creator setBeamOptions        0 0 "B"
$woss_creator setBathymetryType     0 0 "L"
$woss_creator setSimulationTimes    0 0 1 12 2009 0 0 1 1 12 2009 0 0 1


WOSS/Manager/Simple set debug 0
set woss_manager [new "WOSS/Manager/Simple"]


# WOSS/Manager/Simple/MultiThread set debug 0
# WOSS/Manager/Simple/MultiThread set concurrent_threads 3
# WOSS/Manager/Simple/MultiThread set space_sampling 0
# set woss_manager [new "WOSS/Manager/Simple/MultiThread"]

WOSS/Definitions/TransducerHandler set debug 0
set transducer_handler [new "WOSS/Definitions/TransducerHandler"]


WOSS/Controller set debug 0
set woss_controller [new WOSS/Controller]
$woss_controller setBathymetryDbCreator      $db_bathy
$woss_controller setSedimentDbCreator        $db_sedim
$woss_controller setSSPDbCreator             $db_ssp
$woss_controller setTimeArrResultsDbCreator  $db_res_arr
$woss_controller setWossDbManager            $db_manager
$woss_controller setWossManager              $woss_manager
$woss_controller setWossCreator              $woss_creator
$woss_controller setTransducerhandler        $transducer_handler
$woss_controller initialize


WOSS/ChannelEstimator set debug_     0.0
WOSS/ChannelEstimator set avg_coeff_ 0.5
set channel_estimator [ new "WOSS/ChannelEstimator"]

WOSS/PlugIn/ChannelEstimator set debug_ 0

WOSS/Module/Channel set channel_time_resolution_   -1.0
WOSS/Module/Channel set debug_                     0
set channel [new WOSS/Module/Channel]
$channel setWossManager $woss_manager
$channel setChannelEstimator $channel_estimator

WOSS/MPropagation set debug_ 0
set propagation [new WOSS/MPropagation]
$propagation setWossManager $woss_manager

set data_mask [new MSpectralMask/Rect]
$data_mask setFreq       $opt(freq)
$data_mask setBandwidth  $opt(bw)

Module/UW/CBR set packetSize_          $opt(pktsize)
Module/UW/CBR set period_              $opt(cbr_period)
Module/UW/CBR set PoissonTraffic_      1
Module/UW/CBR set debug_	       0

WOSS/Module/MPhy/BPSK  set debug_                     0
WOSS/Module/MPhy/BPSK  set bitrate_                   $opt(bitrate)
WOSS/Module/MPhy/BPSK  set AcquisitionThreshold_dB_   10.0 
WOSS/Module/MPhy/BPSK  set RxSnrPenalty_dB_           $opt(rx_snr_penalty_db)
WOSS/Module/MPhy/BPSK  set TxSPLMargin_dB_            $opt(tx_margin_db)
WOSS/Module/MPhy/BPSK  set MaxTxSPL_dB_               $opt(txpower)
WOSS/Module/MPhy/BPSK  set MinTxSPL_dB_               10
WOSS/Module/MPhy/BPSK  set MaxTxRange_                50000
WOSS/Module/MPhy/BPSK  set PER_target_                $opt(per_tgt)
WOSS/Module/MPhy/BPSK  set CentralFreqOptimization_   0
WOSS/Module/MPhy/BPSK  set BandwidthOptimization_     0
WOSS/Module/MPhy/BPSK  set SPLOptimization_           1

Module/UW/TP set pkt_delete_time_from_queue_ 1000
Module/UW/TP set expected_ACK_threshold_ 0.5
###############################
# Procedure for creating nodes
###############################

proc createNode { id } {

    global channel propagation data_mask ns cbr position node port portnum ipr ipif channel_estimator
    global phy phy_data posdb opt rvposx rvposy rvposz mhrouting mll mac woss_utilities woss_creator db_manager
    
    set node($id) [$ns create-M_Node $opt(tracefile) $opt(cltracefile)] 

    for {set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
       set cbr($id,$cnt) [new Module/UW/CBR]
    }
    set port($id) [new Module/UW/TP]
    set ipif($id) [new Module/UW/IP]
    set mll($id)  [new Module/UW/MLL] 
    set mac($id)  [new Module/MMac/CSMA_ALOHA]
    set phy_data($id)  [new WOSS/Module/MPhy/BPSK]

    for {set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
       $node($id)  addModule 6 $cbr($id,$cnt)   0  "CBR"
    }
    $node($id)  addModule 5 $port($id)  0  "PRT"
    $node($id)  addModule 4 $ipif($id)  0  "IPF"   
    $node($id) addModule  3 $mll($id)   0  "MLL"
    $node($id)  addModule 2 $mac($id)   0  "MAC"
    $node($id)  addModule 1 $phy_data($id)   0  "DPHY"

    $node($id) setConnection $port($id)  $ipif($id)   0
    $node($id) setConnection $ipif($id)  $mll($id)   0
    $node($id) setConnection $mll($id)   $mac($id)   0
    $node($id) setConnection $mac($id)   $phy_data($id)   0
    $node($id) addToChannel  $channel    $phy_data($id)   1

    for {set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
      $node($id)  setConnection $cbr($id,$cnt) $port($id)  1
      set portnum($id,$cnt) [$port($id) assignPort $cbr($id,$cnt)]
	if {$id > 252} {
	    puts "hostnum > 252!!! exiting"
	    exit
	}

  }

    $ipif($id) addr "1.0.0.${id}"
#     $ipif($id) subnet "0.0.0.0"

    set position($id) [new "WOSS/Position/WayPoint"]
    $node($id) addPosition $position($id)
    set posdb($id) [new "PlugIn/PositionDB"]
    $node($id) addPlugin $posdb($id) 20 "PDB"
    $posdb($id) addpos [$ipif($id) addr] $position($id)

    set curr_x [$rvposx value]
    set curr_y [$rvposy value]

    set curr_lat    [ $woss_utilities getLatfromDistBearing  $opt(start_lat) $opt(start_long) 180.0 $curr_y ]
    set curr_lon    [ $woss_utilities getLongfromDistBearing $opt(start_lat) $opt(start_long) 90.0  $curr_x ]
    set curr_depth  [expr [ $db_manager getBathymetry $curr_lat $curr_lon] + $opt(node_bathy_offset)]

    $position($id) addStartWayPoint  $curr_lat $curr_lon [expr -1.0 * $curr_depth] 0.0 0.0 
 
    set ch_estimator_plugin($id) [ new "WOSS/PlugIn/ChannelEstimator"]
    $node($id) addPlugin $ch_estimator_plugin($id) 19 "CHE"
    $ch_estimator_plugin($id) setChannelEstimator $channel_estimator
    $ch_estimator_plugin($id) insertNode [$ipif($id) addr] $position($id)


    puts "node $id at ([$position($id) getLatitude_], [$position($id) getLongitude_], [$position($id) getAltitude_]) , ([$position($id) getX_], [$position($id) getY_], [$position($id) getZ_])"

    $woss_creator setCustomAngles $position($id) 0 $opt(node_min_angle) $opt(node_max_angle)

    set interf_data($id) [new "WOSS/MInterference/MIV"]
    $interf_data($id) set maxinterval_ $opt(maxinterval_)
    $interf_data($id) set debug_       0

    $phy_data($id) setSpectralMask $data_mask
    $phy_data($id) setInterference $interf_data($id)
    $phy_data($id) setPropagation $propagation
    $phy_data($id) set debug_ 0

    $mac($id) $opt(mac_ack_mode)
    $mac($id) initialize

    $port($id) node_id $id
    $port($id) $opt(port_ack_mode)
    $port($id) $opt(ack_tx_mode)
}


proc connectNodes {id1 des1} {
    global ipif ipr portnum cbr cbr_sink ipif_sink portnum_sink ipr_sink opt port

    puts "Source Node: $id1; Destination Node: $des1"

    $cbr($id1,$des1) set destAddr_ [$ipif($des1) addr]
    $cbr($id1,$des1) set destPort_ $portnum($des1,$id1)
    $port($id1) set destPort_ $portnum($des1,$id1)


    $cbr($des1,$id1) set destAddr_ [$ipif($id1) addr]
    $cbr($des1,$id1) set destPort_ $portnum($id1,$des1) 
    $port($des1) set destPort_ $portnum($id1,$des1)

}



###############################
# create nodes
###############################

for {set id 0} {$id < $opt(nn)} {incr id}  {
    createNode $id
}

################################
#fill ARP tables
################################

for {set id1 0} {$id1 < $opt(nn)} {incr id1}  {
    for {set id2 0} {$id2 < $opt(nn)} {incr id2}  {
      $mll($id1) add_testbed_entry ip_address "1.0.0.$id2" mac_address $id2
    }   
}

###################################
#setup flows & connect nodes
##################################

for {set id1 0} {$id1 < $opt(nn)} {incr id1}  {

if {$id1 != [expr ($opt(nn) - 1)]} {
	set id2 [expr ($id1 + 1)]
} else {
	set id2 0
}

connectNodes $id1 $id2
      
$ns at $opt(starttime)	     "$cbr($id1,$id2) start"
$ns at $opt(stoptime)  	     "$cbr($id1,$id2) stop"

}

######################################
#finish procedure
########################################

proc finish {} {
	global ns opt cbr outfile b 
	global mac propagation cbr_sink mac_sink phy_data phy_data_sink channel db_manager propagation port

	$db_manager closeAllConnections

	puts ""
	puts "Seed    : $opt(rep_num)"
	puts "cbr period	: $opt(cbr_period)"
	puts "No. of modes	: $opt(nn)"

	set sum_cbr_throughput		0
	set sum_cbr_delay  		0
	set sum_cbr_pkts		0
	set sum_cbr_rxpkts		0
	set sum_cbr_per			0

    for {set id4 0} {$id4 < $opt(nn)} {incr id4}  {
      for {set id3 0} {$id3 < $opt(nn)} {incr id3}  {

	set cbr_throughput   [$cbr($id4,$id3) getthr]
	set cbr_delay        [$cbr($id4,$id3) getftt]
	set cbr_per          [$cbr($id4,$id3) getper]
	set cbr_pkts         [$cbr($id4,$id3) getsentpkts]
	set cbr_rxpkts       [$cbr($id4,$id3) getrecvpkts]

	#... create label $sum_cbr_throughput
	set sum_cbr_throughput [expr $sum_cbr_throughput + $cbr_throughput]
	#... create label $sum_cbr_delay
	set sum_cbr_delay      [expr $sum_cbr_delay + $cbr_delay]
	#... create label $sum_cbr_pkts
	set sum_cbr_pkts [expr $sum_cbr_pkts + $cbr_pkts]
	#... create label $sum_cbr_rxpkts
	set sum_cbr_rxpkts [expr $sum_cbr_rxpkts + $cbr_rxpkts]
	#... create label $sum_cbr_rxpkts
	set sum_cbr_per [expr $sum_cbr_per + $cbr_per]

	}
    }	
	
	puts "Total app data pkts created            : $sum_cbr_pkts"
	puts "Total app data pkts received           : $sum_cbr_rxpkts"
	set avg_pdr [expr ((double($sum_cbr_rxpkts) / double($sum_cbr_pkts))*100)]
	puts "Average packet delivery ratio          : $avg_pdr"
	puts "Average throughput                     : [expr ($sum_cbr_throughput/$opt(nn))]"
	set avg_thr [expr ($sum_cbr_throughput / (($opt(pktsize) * 8.0) / $opt(cbr_period)))/$opt(nn)]
	puts "Average normalized throughput          : $avg_thr"
	set avg_nthr [expr {(double($sum_cbr_rxpkts) * $opt(t_time)) / $opt(txduration)}]
	puts "Average classical throughput           : $avg_nthr"
	set avg_dly [expr ($sum_cbr_delay / $opt(nn))]
	puts "Average delay                          : $avg_dly"
	puts "Average packet error rate              : [expr (100 - $avg_pdr)]"
	set avg_PER [expr ($sum_cbr_per/$opt(nn))]
	puts "PER for Noise                          : $avg_PER"
	puts ""

	set sum_mac_tx_pkts             0.0
	set sum_mac_ack_rx              0.0
	set sum_mac_ack_tx              0.0
	set sum_mac_data_rx             0.0

	set sum_phy_dropped_noise       0.0
	set sum_phy_dropped_deaf	0.0
	set sum_phy_error_noise         0.0
        set sum_phy_error_interf        0.0

    for {set id3 0} {$id3 < $opt(nn)} {incr id3}  {

	set mac_sent_data_pkts    [$mac($id3) getDataPktsTx]
	set mac_sent_ack_pkts     [$mac($id3) getAckPktsTx]
	set mac_rcvd_ack_pkts     [$mac($id3) getAckPktsRx]
	set mac_rcvd_data_pkts    [$mac($id3) getDataPktsRx]

	set phy_total_dropped_noise       [$phy_data($id3) getDroppedPktsNoise]
	set phy_total_dropped_deaf        [$phy_data($id3) getDroppedPktsDeaf]
	set phy_total_error_noise         [$phy_data($id3) getErrorPktsNoise]
	set phy_total_error_interf        [$phy_data($id3) getErrorPktsInterf]

	set sum_mac_tx_pkts           [ expr $sum_mac_tx_pkts + $mac_sent_data_pkts]
	set sum_mac_ack_rx 	      [ expr $sum_mac_ack_rx + $mac_rcvd_ack_pkts]
	set sum_mac_ack_tx            [ expr $sum_mac_ack_tx + $mac_sent_ack_pkts]
	set sum_mac_data_rx           [ expr $sum_mac_data_rx + $mac_rcvd_data_pkts]
    
	set sum_phy_dropped_noise  	  [ expr $sum_phy_dropped_noise + $phy_total_dropped_noise ]
	set sum_phy_dropped_deaf          [ expr $sum_phy_dropped_deaf + $phy_total_dropped_deaf ]
	set sum_phy_error_noise           [ expr $sum_phy_error_noise + $phy_total_error_noise ]
	set sum_phy_error_interf          [ expr $sum_phy_error_interf + $phy_total_error_interf ]

    }

	set pdboi_ratio		[ expr $sum_phy_error_interf/$sum_mac_tx_pkts] 
	#pdboi means packet dropped because of interference
  
	puts "average data pkts tx                   : [expr ($sum_mac_tx_pkts/$opt(nn))]"
	puts "average data pkts rx                   : [expr ($sum_mac_data_rx/$opt(nn))]"
	puts "average tx ack pkts                    : [expr ($sum_mac_ack_tx/$opt(nn))]"
	puts "average rx ack pkts                    : [expr ($sum_mac_ack_rx/$opt(nn))]"
	puts "mac success ratio                      : [expr ($sum_mac_data_rx / $sum_mac_tx_pkts)]"

	puts "average phy drp pkts noise             : [expr ($sum_phy_dropped_noise/$opt(nn))]"
	puts "average phy drp pkts interf            : [expr ($sum_phy_dropped_deaf/$opt(nn))]"
	puts "average phy err pkts noise             : [expr ($sum_phy_error_noise/$opt(nn))]"
	puts "average phy err pkts interf            : [expr ($sum_phy_error_interf/$opt(nn))]"
	puts "average Normalized interf              : $pdboi_ratio"

	set mac_pdr [expr (($sum_mac_data_rx / $sum_mac_tx_pkts) * 100)]

	set sum_port_ack_tx         0.0
        set sum_port_nack_tx        0.0
	set sum_port_ack_rx         0.0
        set sum_port_nack_rx        0.0

    for {set id5 0} {$id5 < $opt(nn)} {incr id5}  {
	set port_ack_pkts_tx    [$port($id5) getAckTxCount]
	set port_nack_pkts_tx    [$port($id5) getNackTxCount]
	set port_ack_pkts_rx    [$port($id5) getAckRxCount]
	set port_nack_pkts_rx    [$port($id5) getNackRxCount]

	set sum_port_ack_tx           [ expr $sum_port_ack_tx + $port_ack_pkts_tx]
	set sum_port_nack_tx           [ expr $sum_port_nack_tx + $port_nack_pkts_tx]
	set sum_port_ack_rx           [ expr $sum_port_ack_rx + $port_ack_pkts_rx]
	set sum_port_nack_rx           [ expr $sum_port_nack_rx + $port_nack_pkts_rx]	
    }


    puts "average no of ACK tx                   : [expr $sum_port_ack_tx / $opt(nn)]"
    puts "average no of NACK tx                  : [expr $sum_port_nack_tx / $opt(nn)]"
    puts "average no of ACK rx                   : [expr $sum_port_ack_rx / $opt(nn)]"
    puts "average no of NACK rx                  : [expr $sum_port_nack_rx / $opt(nn)]"


        puts $outfile [format "%s	%s	%s	%s	%s	%s	%s	%s" "$opt(rep_num)" "$opt(cbr_period)" "$opt(nn)" "$avg_pdr" "$avg_nthr" "$avg_dly" "$avg_PER" "$pdboi_ratio"]
    
	set opt(end_clock) [clock seconds]
	puts "done! in [expr $opt(end_clock) - $opt(start_clock)] seconds"
        puts "tracefile: $opt(tracefilename)"
	$ns flush-trace
	close $opt(tracefile)

	file delete -force "$opt(path)"

}


###################
# start simulation
###################

puts -nonewline "Simulating"

for {set t $opt(starttime)} {$t <= $opt(stoptime)} {set t [expr $t + $opt(txduration) / 40.0 ]} {
    $ns at $t "puts -nonewline ."
}
$ns at [expr $opt(stoptime) + 1000.0]  "finish; $ns halt" 

$ns run
