#   +-------------------------+	+-------------------------+
#   |  7. UWCBR               |	|  7. UWCBR           	  |
#   +-------------------------+	+-------------------------+
#   |  6. UWUDP               |	|  6. UWUDP          	  |
#   +-------------------------+	+-------------------------+
#   |  5. SUW Routing         |	|  5. SUW Routing     	  |
#   +-------------------------+	+-------------------------+
#   |  4. UWIP                |	|  4. UWIP           	  |
#   +-------------------------+	+-------------------------+
#   |  3. UWMLL               |	|  3. UWMLL         	  |
#   +-------------------------+	+-------------------------+
#   |  2. MMAC/CSMA_ALOHA     |	|  2. MMAC/CSMA_ALOHA 	  |
#   +-------------------------+	+-------------------------+
#   |  1. MPHY/BPSK/Underwater|	|  1. MPHY/BPSK/Underwater|
#   +-------------------------+	+-------------------------+
#                  |                          |
#   +-------------------------+	+-------------------------+
#   |                  UnderwaterChannel                  |
#   +-------------------------+	+-------------------------+

######################################
# Flags to enable or disable options #
######################################
set opt(ackmode) 0
set opt(randomposition) 0
set opt(mobility) 0
set opt(verbose) 1
set opt(fileoutput) 0
set opt(gm) 1

######################################
# Load                               #
######################################
load libMiracle.so
load libmphy.so
load libmmac.so
load libUwmStd.so
load libcsmaaloha.so
load libmll.so
load libMiracleBasicMovement.so
load libMiracleIp.so
load libuwgmposition.so
load libuwdriftposition.so
load libuwip.so
load libuwicrp.so
load libuwudp.so
load libuwcbr.so

################## If you are running this tcl with WOSS physical layer, uncomment following lines####################
load libWOSS.so
load libWOSSPhy.so
######################################################################################################################

set ns [new Simulator]
$ns use-Miracle

######################################
# Global allocations & misc options  #
######################################
set opt(start_clock) [clock seconds]

set opt(nn) 				4.0 ;# Number of Nodes
set opt(pktsize)	 		500  ;# Pkt sike in byte
set opt(cbr_period)	 		5
set opt(send_period)		20
set opt(starttime)       	1
set opt(stoptime)        	100000 ;# Simulation end in seconds
set opt(txduration)     	[expr $opt(stoptime) - $opt(starttime)]

set opt(txpower)	 		135.0 
set opt(per_tgt)	 		0.1
set opt(rx_snr_penalty_db)	-10.0
set opt(tx_margin_db)		10.0

set opt(node_min_angle)		-90.0
set opt(node_max_angle)		90.0
set opt(sink_min_angle)		-90.0
set opt(sink_max_angle) 	90.0

set opt(maxinterval_)    	20.0
set opt(freq) 			    25000.0
set opt(bw)              	5000.0
set opt(bitrate)	 		4800.0
set opt(ack_mode)	 		"setNoAckMode"

set rng [new RNG]
set rng_position [new RNG]

#######################
# Area of Simulation  #
#######################
set opt(x_length)			4000     ;#In meters.
set opt(y_length)			1000      ;#In meters.
set opt(z_length)			200       ;#In meters.

set opt(num_x_sections) $opt(nn) ;#Set the number of nodes in the x coordinate
set opt(num_y_sections) [expr $opt(nn) / $opt(num_x_sections)]
set opt(num_z_sections) 1

set opt(x_section_length) [expr $opt(x_length) / $opt(num_x_sections)]
set opt(y_section_length) [expr $opt(y_length) / $opt(num_y_sections)]
set opt(z_section_length) [expr $opt(z_length) / $opt(num_z_sections)]

set opt(maximum_node_distance) [expr sqrt(pow($opt(x_section_length),2)+pow($opt(y_section_length),2)+pow($opt(z_section_length),2))]

set node_coordinates(0) "0 0 0"

################################
# Terminal's parameters check  #
################################
if {$argc != 4} {
  puts "The script requires three numbers to be inputed:"
  puts "- the first for the cbr seed;"
  puts "- the second one is the seed of the positions;"
  puts "- the third one is the cbr poisson period;"
  puts "- the fourth one is the the path estimation poisson period."
  puts "For example ns SpreadUW_test.tcl 4 2 100 10"
  puts "Please try again."
  return
} else {
	set opt(seedcbr)		[lindex $argv 0]
	set opt(seedposition)	[lindex $argv 1]
	set opt(cbr_period)		[lindex $argv 2]
	set opt(send_period)	[lindex $argv 3]
  $rng seed		$opt(seedcbr)
  $rng_position seed $opt(seedposition)
}

set rnd_gen [new RandomVariable/Uniform]
$rnd_gen use-rng $rng

###########################
# Trace files and output  #
###########################
set opt(tracefilename) "/dev/null"
set opt(tracefile) [open $opt(tracefilename) w]

set opt(cltracefilename) "/dev/null"
set opt(cltracefile) [open $opt(cltracefilename) w]

if ($opt(fileoutput)) {
	set opt(outfile) [open "./out" a+]
} else {
	set opt(outfile) [open "/dev/null" a+]
}
	
################## If you are running this tcl with WOSS physical layer, uncomment following lines####################
set woss_utilities [new "WOSS/Utilities"]
WOSS/Manager/Simple set debug 0
WOSS/Manager/Simple set space_sampling 0.0
set woss_manager [new "WOSS/Manager/Simple"]
#####################################################################################################################

###########################
# Channel and Propagation #
###########################
set channel [new Module/UnderwaterChannel]
set propagation [new MPropagation/Underwater]

set data_mask [new MSpectralMask/Rect]
$data_mask setFreq       $opt(freq)
$data_mask setBandwidth  $opt(bw)

#########################
# Modules Configuration #
#########################
Module/UW/CBR set packetSize_          $opt(pktsize)
Module/UW/CBR set period_              $opt(cbr_period)
Module/UW/CBR set PoissonTraffic_      1
Module/UW/CBR set debug_			   0

# Attention: timer_ack_waiting_ < opt(cbr_period), and timer_ack_waiting_ depends on the distance between nodes
Module/UW/ICRPNode set printDebug_			0
Module/UW/ICRPNode set maxvaliditytime_		270
Module/UW/ICRPNode set timer_ack_waiting_   10

Module/UW/ICRPSink set printDebug_			0

# Mobility model configuration
# Gauss Markov
set opt(speedMean) 1
Position/UWGM set xFieldWidth_		10000
Position/UWGM set yFieldWidth_		5000
Position/UWGM set yFieldWidth_		200
Position/UWGM set alpha_			0.90
Position/UWGM set alphaPitch_		0.90
Position/UWGM set directionMean_	0
Position/UWGM set pitchMean_		0
Position/UWGM set sigmaPitch_		0
Position/UWGM set updateTime_		5
Position/UWGM set debug_			0

# DRIFT
Position/UWDRIFT set boundx_               1;      # 0 no bounds, 1 yes
Position/UWDRIFT set boundy_               1
Position/UWDRIFT set boundz_               1
Position/UWDRIFT set xFieldWidth_	       100000;  # The module uses these values only if the corrispective bound is set to 1, in meters
Position/UWDRIFT set yFieldWidth_	       2000
Position/UWDRIFT set zFieldWidth_	       200
Position/UWDRIFT set speed_horizontal_     1;      # Horizontal component of the speed, in m/s. 0 means no constant horizontal speed = completely random
Position/UWDRIFT set speed_longitudinal_   0
Position/UWDRIFT set speed_vertical_       0
Position/UWDRIFT set alpha_	               0.7
Position/UWDRIFT set deltax_               1      
Position/UWDRIFT set deltay_               1
Position/UWDRIFT set deltaz_               1
Position/UWDRIFT set starting_speed_x_     0
Position/UWDRIFT set starting_speed_y_     0
Position/UWDRIFT set starting_speed_z_     0
Position/UWDRIFT set updateTime_		   2;      # In seconds
Position/UWDRIFT set debug_			       0

################## If you are running this tcl with WOSS physical layer, uncomment following lines####################
WOSS/Module/MPhy/BPSK set debug_                     0
WOSS/Module/MPhy/BPSK set bitrate_                   $opt(bitrate)
WOSS/Module/MPhy/BPSK set AcquisitionThreshold_dB_   10.0 
WOSS/Module/MPhy/BPSK set RxSnrPenalty_dB_           $opt(rx_snr_penalty_db)
WOSS/Module/MPhy/BPSK set TxSPLMargin_dB_            $opt(tx_margin_db)
WOSS/Module/MPhy/BPSK set MaxTxSPL_dB_               $opt(txpower)
WOSS/Module/MPhy/BPSK set MinTxSPL_dB_               10
WOSS/Module/MPhy/BPSK set MaxTxRange_                50000
WOSS/Module/MPhy/BPSK set PER_target_                $opt(per_tgt)
WOSS/Module/MPhy/BPSK set CentralFreqOptimization_   0
WOSS/Module/MPhy/BPSK set BandwidthOptimization_     0
WOSS/Module/MPhy/BPSK set SPLOptimization_           1
######################################################################################################################

###################################
# Scripts for position setting up #
###################################
proc initializepositions {} {
	
	global opt rnd_gen
	global cube node_coordinates

	set i 0
	for {set z 0} {$z <  $opt(num_z_sections)} {incr z} {
		for {set y 0} {$y <  $opt(num_y_sections)} {incr y} {
			for {set x 0} {$x <  $opt(num_x_sections)} {incr x} {
				set cube($i) "[expr $x*($opt(x_section_length))] [expr ($x + 1)*($opt(x_section_length))] [expr $y*($opt(y_section_length))] [expr ($y + 1)*($opt(y_section_length))] [expr $z*($opt(z_section_length))] [expr ($z + 1)*($opt(z_section_length))]"
				incr i
			}
		}	
	}

	set i 0
	for {set z 0} {$z <  $opt(num_z_sections)} {incr z} {
		for {set y 0} {$y <  $opt(num_y_sections)} {incr y} {
			for {set x 0} {$x <  $opt(num_x_sections)} {incr x} {
				$rnd_gen set min_ [lindex $cube($i) 0]
				$rnd_gen set max_ [lindex $cube($i) 1]
				if {$opt(randomposition)} {
					set x_ [$rnd_gen value]
				} else {
					set x_ [lindex $cube($i) 0]
				}
				$rnd_gen set min_ [lindex $cube($i) 2]
				$rnd_gen set max_ [lindex $cube($i) 3]
				if {$opt(randomposition)} {
					set y_ [$rnd_gen value]
				} else {
					set y_ [lindex $cube($i) 2]
				}
				$rnd_gen set min_ [lindex $cube($i) 4]
				$rnd_gen set max_ [lindex $cube($i) 5]
				if {$opt(randomposition)} {
					set z_ [$rnd_gen value]
				} else {
					set z_ [lindex $cube($i) 4]
				}
				set node_coordinates($i) "$x_ $y_ $z_"
				#puts "node: $i at position ($x_, $y_, $z_)"
				incr i
			}
		}	
	}
}

################################
# Procedure to create the nodes#
################################
proc createNode { id } {

    global channel propagation data_mask ns cbr position node udp portnum ipr ipif channel_estimator
    global phy posdb opt rvposx rvposy rvposz mhrouting mll mac woss_utilities woss_creator db_manager
    global node_coordinates
    
    set node($id) [$ns create-M_Node $opt(tracefile) $opt(cltracefile)] 

    set cbr($id)  [new Module/UW/CBR] 
    set udp($id)  [new Module/UW/UDP]
    set ipr($id)  [new Module/UW/ICRPNode]
    set ipif($id) [new Module/UW/IP]
    set mll($id)  [new Module/MLL] 
    set mac($id)  [new Module/MMac/CSMA_ALOHA] 
    set phy($id)  [new WOSS/Module/MPhy/BPSK]

    $node($id) addModule 7 $cbr($id)   0  "CBR"
    $node($id) addModule 6 $udp($id)   0  "UDP"
    $node($id) addModule 5 $ipr($id)   0  "IPR"
    $node($id) addModule 4 $ipif($id)  0  "IPF"   
    $node($id) addModule 3 $mll($id)   0  "MLL"
    $node($id) addModule 2 $mac($id)   0  "MAC"
    $node($id) addModule 1 $phy($id)   0  "PHY"

    $node($id) setConnection $cbr($id)   $udp($id)   0
    $node($id) setConnection $udp($id)   $ipr($id)   0
    $node($id) setConnection $ipr($id)   $ipif($id)  0
    $node($id) setConnection $ipif($id)  $mll($id)   0
    $node($id) setConnection $mll($id)   $mac($id)   0
    $node($id) setConnection $mac($id)   $phy($id)   0
    $node($id) addToChannel  $channel    $phy($id)   0

    set portnum($id) [$udp($id) assignPort $cbr($id) ]
    if {$id > 254} {
		puts "hostnum > 254!!! exiting"
		exit
    }
	set tmp_ [expr ($id) + 1]
    $ipif($id) addr "1.0.0.${tmp_}"
    $ipr($id) ipsink "1.0.0.254"
    
    if {$opt(mobility)} {
		if {$opt(gm)} {
			set position($id) [new "Position/UWGM"]
		} else {
			set position($id) [new "Position/UWDRIFT"]
		}
	} else {
		set position($id) [new "Position/BM"]
	}
    $node($id) addPosition $position($id)
    set posdb($id) [new "PlugIn/PositionDB"]
    $node($id) addPlugin $posdb($id) 20 "PDB"
    $posdb($id) addpos [$ipif($id) addr] $position($id)
    
    set curr_x [lindex $node_coordinates($id) 0]
    set curr_y [lindex $node_coordinates($id) 1]
    set curr_z [lindex $node_coordinates($id) 2]

    $position($id) setX_ $curr_x
    $position($id) setY_ $curr_y
    $position($id) setZ_ [expr -1.0 * $curr_z]
    if {$opt(mobility)} {
		if {$opt(gm)} {
			$position($id) bound "REBOUNCE"
			$position($id) speedMean $opt(speedMean)
		}
	}
    
    #puts "node [expr $id + 1] at ([$position($id) getX_], [$position($id) getY_], [$position($id) getZ_])\n"

    set interf_data($id) [new "MInterference/MIV"]
    $interf_data($id) set maxinterval_ $opt(maxinterval_)
    $interf_data($id) set debug_       0

    #$phy($id) set debug_ 0
    $phy($id) setPropagation $propagation
    
    $phy($id) setSpectralMask $data_mask
    $phy($id) setInterference $interf_data($id)
    $mac($id) $opt(ack_mode)
    $mac($id) initialize
}

################################
# Procedure to create the sinks#
################################
proc createSink { } {

	global channel propagation smask data_mask ns cbr_sink position_sink node_sink udp_sink portnum_sink interf_data_sink
	global phy_data_sink posdb_sink opt mll_sink mac_sink ipr_sink ipif_sink bpsk interf_sink channel_estimator

	set node_sink [$ns create-M_Node $opt(tracefile) $opt(cltracefile)]

	for {set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
		set cbr_sink($cnt)  [new Module/UW/CBR] 
	}
	set udp_sink       [new Module/UW/UDP]
	set ipr_sink       [new Module/UW/ICRPSink]
	set ipif_sink      [new Module/UW/IP]
	set mll_sink       [new Module/MLL] 
	set mac_sink       [new Module/MMac/CSMA_ALOHA] 
	set phy_data_sink  [new WOSS/Module/MPhy/BPSK]


	for { set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
		$node_sink addModule 7 $cbr_sink($cnt) 0 "CBR"
	}
	$node_sink addModule 6 $udp_sink       0 "UDP"
	$node_sink addModule 5 $ipr_sink       0 "IPR"
	$node_sink addModule 4 $ipif_sink      0 "IPF"   
	$node_sink addModule 3 $mll_sink       0 "MLL"
	$node_sink addModule 2 $mac_sink       0 "MAC"
	$node_sink addModule 1 $phy_data_sink  0 "PHY"

	for { set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
		$node_sink setConnection $cbr_sink($cnt)  $udp_sink   	0   
	}
	$node_sink setConnection $udp_sink  $ipr_sink      		0
	$node_sink setConnection $ipr_sink  $ipif_sink       	0
	$node_sink setConnection $ipif_sink $mll_sink        	0 
	$node_sink setConnection $mll_sink  $mac_sink        	0
	$node_sink setConnection $mac_sink  $phy_data_sink   	0
	$node_sink addToChannel  $channel   $phy_data_sink   	0

	for { set cnt 0} {$cnt < $opt(nn)} {incr cnt} {
		set portnum_sink($cnt) [$udp_sink assignPort $cbr_sink($cnt)]
		if {$cnt > 252} {
			puts "hostnum > 252!!! exiting"
			exit
		}	
	}
    
	$ipif_sink addr "1.0.0.254"

	if {$opt(mobility)} {
		if {$opt(gm)} {
			set position_sink [new "Position/UWGM"]
		} else {
			set position_sink [new "Position/UWDRIFT"]
		}
	} else {
		set position_sink [new "Position/BM"]
	}
	$node_sink addPosition $position_sink
	set posdb_sink [new "PlugIn/PositionDB"]
	$node_sink addPlugin $posdb_sink 20 "PDB"
	$posdb_sink addpos [$ipif_sink addr] $position_sink
	# Set the sink in the middle of the volume
	set curr_x [expr $opt(x_length) / 2]
	set curr_y [expr $opt(y_length) / 2]
	set curr_z [expr $opt(z_length) / 2]
	$position_sink setX_ 0; #$curr_x
	$position_sink setY_ 0; #$curr_y
	$position_sink setZ_ [expr -1.0 * $curr_z]
	if {$opt(mobility)} {
		if {$opt(gm)} {
			$position_sink bound "REBOUNCE"
			$position_sink speedMean $opt(speedMean)
		}
	}
	#puts "node sink at ([$position_sink getX_], [$position_sink getY_], [$position_sink getZ_])\n"

	#$phy_data_sink set debug_ 0

	set interf_data_sink [new "MInterference/MIV"]
	$interf_data_sink set maxinterval_ $opt(maxinterval_)
	$interf_data_sink set debug_       0

	$phy_data_sink setSpectralMask $data_mask
	$phy_data_sink setInterference $interf_data_sink
	$phy_data_sink setPropagation $propagation

	$mac_sink $opt(ack_mode)
	$mac_sink initialize
 }

############################
# Connect the CBRs modules #
############################
proc connectNodes {id1} {
    global ipif ipr portnum cbr cbr_sink ipif_sink portnum_sink ipr_sink

    $cbr($id1) set destAddr_ [$ipif_sink addr]
    $cbr($id1) set destPort_ $portnum_sink($id1)
    $cbr_sink($id1) set destAddr_ [$ipif($id1) addr]
    $cbr_sink($id1) set destPort_ $portnum($id1)  
}

################################
# Computation of the positions #
################################
initializepositions

############################################
# Creation and initialization of the Nodes #
############################################
for {set id 0} {$id < $opt(nn)} {incr id}  {
    createNode $id
	$ipr($id) initialize
}


##############################################
# Creation and initialization of the Sink
##############################################

createSink
$ipr_sink initialize

################################
# Setup flows
################################

for {set id1 0} {$id1 < $opt(nn)} {incr id1}  {
    connectNodes $id1
}

################################
#fill ARP tables
################################

for {set id1 0} {$id1 < $opt(nn)} {incr id1}  {
    for {set id2 0} {$id2 < $opt(nn)} {incr id2}  {
      $mll($id1) addentry [$ipif($id2) addr] [$mac($id2) addr]
    }   
    $mll($id1) addentry [$ipif_sink addr] [ $mac_sink addr]
    $mll_sink addentry [$ipif($id1) addr] [ $mac($id1) addr]
}


################################
#Start cbr(s)
################################

for {set id1 0} {$id1 < $opt(nn)} {incr id1}  {
    $ns at $opt(starttime)	     "$cbr($id1) start"
    $ns at $opt(stoptime)    	 "$cbr($id1) stop"
}

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
		puts "period  send     : $opt(send_period) ms"
		puts "seed cbr         : $opt(seedcbr)"
		puts "seed position    : $opt(seedposition)"
		puts "number of nodes  : $opt(nn)"
		puts "simulation length: $opt(txduration) s"
		puts "tx power         : $opt(txpower) dB"
		puts "tx frequency     : $opt(freq) Hz"
		puts "tx bandwidth     : $opt(bw) Hz"
		puts "bitrate          : $opt(bitrate) bps"
		puts "area             : $opt(x_length) x $opt(y_length) x $opt(z_length) meters"
		puts "max node distance: $opt(maximum_node_distance) meters"
		puts "---------------------------------------------------------------------"
	}
	set sum_cbr_throughput 	0
	set sum_per		0
	set sum_cbr_sent_pkts	0.0
	set sum_cbr_rcv_pkts	0.0
	set totalenergy         0.0
	
	set sink_x [expr $opt(x_length) / 2]
	set sink_y [expr $opt(y_length) / 2]
	set sink_z [expr $opt(z_length) / 2]

	for {set i 0} {$i < $opt(nn)} {incr i}  {
		set cbr_throughput	   	[$cbr_sink($i) getthr]
		set cbr_sent_pkts		[$cbr($i) getsentpkts]
		set cbr_rcv_pkts	   	[$cbr_sink($i) getrecvpkts]
		
		set distance_from_sink	[expr sqrt(pow(abs([lindex  $node_coordinates($i) 0] - $sink_x),2)+pow(abs([lindex  $node_coordinates($i) 1] - $sink_y),2)+pow(abs([lindex  $node_coordinates($i) 2] - $sink_z),2))]

		if ($opt(verbose)) {
			puts "cbr_sink($i) throughput                	: $cbr_throughput"
		}

		set sum_cbr_throughput [expr $sum_cbr_throughput + $cbr_throughput]
		set sum_cbr_sent_pkts [expr $sum_cbr_sent_pkts + $cbr_sent_pkts]
		set sum_cbr_rcv_pkts  [expr $sum_cbr_rcv_pkts + $cbr_rcv_pkts]
		set nodeenergy         [$phy($i) set ConsumedEnergy_]
		set totalenergy		   [expr $totalenergy + $nodeenergy]
	}
	
	#Add the energu consumed by the sink. TODO: fix this for multiple sinks
	set sinkenergy  [$phy_data_sink set ConsumedEnergy_]
	set totalenergy [expr $totalenergy + $sinkenergy]
	
	set statussinkcount		[$ipr_sink getstatuspktcount]
	set statusnodecount		[$ipr(1) getstatuspktcount]
	set acksinkcount		[$ipr_sink getackpktcount]
	set acknodecount		[$ipr(1) getackpktcount]
	set datacount			[$ipr(1) getdatapktcount]
	set ackheadersize       [$ipr(1) getackheadersize]
	set statusheadersize    [$ipr(1) getstatusheadersize]
	set dataheadersize      [$ipr(1) getdataheadersize]
	set ipheadersize        [$ipif(1) getipheadersize]
	set udpheadersize       [$udp(1) getudpheadersize]
	set cbrheadersize       [$cbr(1) getcbrheadersize]
	set totaldata			[expr ($statusnodecount + $statussinkcount)*($statusheadersize + $ipheadersize) + $datacount*($dataheadersize + $opt(pktsize) + $udpheadersize + $ipheadersize) + ($acksinkcount + $acknodecount)*$ackheadersize]
	set usefuldata          [expr $sum_cbr_rcv_pkts * $opt(pktsize)]
	set overhead            [expr 1 - $usefuldata/$totaldata]
	
	if ($opt(verbose)) {
		puts "Mean Throughput          : [expr ($sum_cbr_throughput/($opt(nn)))]"
		puts "Sent Packets             : $sum_cbr_sent_pkts"
		puts "Received Packets         : $sum_cbr_rcv_pkts"
		puts "Packet Delivery Ratio    : [expr $sum_cbr_rcv_pkts / $sum_cbr_sent_pkts * 100]"
		puts "Number of status pkts    : [expr $statussinkcount + $statusnodecount]"
		puts "Number of ack pkts       : [expr $acksinkcount + $acknodecount]"
		puts "Number of data pkts      : $datacount"
		puts "ICRP Header Size         : $statusheadersize"
		puts "Ack Pkt Header Size      : $ackheadersize"
		puts "Data Pkt Header Size     : $dataheadersize"
		puts "IP Pkt Header Size       : $ipheadersize"
		puts "UDP Header Size          : $udpheadersize"
		puts "CBR Header Size          : $cbrheadersize"
		puts "Total Data               : $totaldata"
		puts "Useful bytes received    : $usefuldata"
		puts "Overhead                 : $overhead"
		puts "Total energy consumption : $totalenergy"
		puts [format "lambda           	 : %.3f" [expr 60.0/$opt(cbr_period)]]
		puts "done!"
	}
	if ($opt(fileoutput)) {
		puts $opt(outfile) [format "%s	%s	%s	%s" "$sum_cbr_throughput" "[expr 1 - $sum_cbr_rcv_pkts / $sum_cbr_sent_pkts]" "$overhead" "$totalenergy"]
	}
	
	$ns flush-trace
	close $opt(tracefile)
}


###################
# start simulation
###################
if ($opt(verbose)) {
	puts "\nStarting Simulation\n"
	puts "----------------------------------------------"
}


$ns at [expr $opt(stoptime) + 250.0]  "finish; $ns halt" 

$ns run
