# UWCBR-MFSK_WHOI_MM sample
# Two nodes, an UWCBR module for each one, directly plugged onto a MFSK_WHOI_MM: public UWMPhy_modem module for each node,
# i.e., a module that do UW network _EMULATION with the FSK WHOI micromodem as follows:
# 1) a- write a Packet* to a given diskfile as a SendDown() and send the corresponding line in the
#       payload of a NMEA minipacket ($CCMUC message); 
#    b- read the payload of an incoming NMEA minipacket ($CAMUA message), extract the indication of
#       the diskfile's line where to read the Packet* address and send this to the upper layers via 
#       SendUp().
#
#	+-----------------------+	                       	+-----------------------+
#	| 2. UWCBR (tx or tx/rx)|                               | 2. UWCBR (rx or tx/rx)|
#	+-----------------------+	                       	+-----------------------+
#	|  1. MFSK_WHOI_MM      |                               |  1. MFSK_WHOI_MM      |
#	+-----------------------+	                       	+-----------------------+
#         |          ^        |                                   |       ^        |
#         v          |        |_____ real UW acoustic channel_____|       |        v    
#    disk_file1   disk_file2                                         diskfile1  diskfile2
#


#Initialize the dynamic libraries
load libMiracle.so
load libuwip.so  
load libuwudp.so
load libuwcbr.so 
load libcbrtracer.so 
load libuwmphy_modem.so
load libmphy.so


#Create a new simulator object
set ns [new Simulator]

#Declare the use of a Real Time Schedule (can NOT! be disabled)
$ns use-scheduler RealTime

#Declare the use of Miracle
$ns use-Miracle

#Choose the application: 1) "simplex or default" for tx->rx or 2) "duplex" for tx/rx <-> tx/rx
set application "simplex"

if {$application == "duplex"} {
   
    set tf_name "uwcbr-mfskwhoimm_duplex.tr"

} else {

    set tf_name "uwcbr-mfskwhoimm_simplex.tr"
}

#Open a file for writing the trace data
set tf [open $tf_name w]
$ns trace-all $tf

#Close the trace file and start nam
proc finish {} {
	global ns tf tf_name
	puts "done!"
        puts "tracefile: $tf_name"
	$ns flush-trace
	close $tf
}

#Serial Paths to be used
set serial_path(1) "/dev/ttyUSB0"
set serial_path(2) "/dev/ttyUSB1"

#Procedure for node generation
proc create-node {i} {
	global ns serial_path node_ cbr_ modem_
	
	#Create a new node (empty)
	set node_($i) [$ns create-M_Node]
	
	#Create the modules to add
	#APPLICATION LAYER
	set cbr_($i) [new "Module/UW/CBR"]
	$cbr_($i) set period_ 7
	$cbr_($i) set packetSize_ 256
	
        #PHY LAYER
        set modem_($i) [new "Module/UW/MPhy_modem/FSK_WHOI_MM" $serial_path($i)]
        $modem_($i) set ID_ $i
        $modem_($i) set period_ 1
        $modem_($i) set debug_ 0


        #Insert the modules in the node
        $node_($i) addModule 2 $cbr_($i) 1 "CBR"
        $node_($i) addModule 1 $modem_($i) 1 "FSKwhoiMM"
 
        #Connect the modules
        $node_($i) setConnection $cbr_($i) $modem_($i) trace
}


#Create the actual application
create-node 1
puts "node 1 created!"
create-node 2
puts "node 2 created!"

#Run the simulation
if {$application == "duplex"} {
    
    $ns at 0 "$modem_(1) start"
    $ns at 0 "$modem_(2) start"
    $ns at 0 "$cbr_(1) start"
    $ns at 24 "$cbr_(2) start"
    $ns at 40 "$cbr_(1) stop"
    $ns at 64 "$cbr_(2) stop"
    $ns at 64 "$modem_(2) stop"
    $ns at 64 "$modem_(1) stop"

    $ns at 64 "finish; $ns halt"

} else {
    
    $ns at 0 "$modem_(1) start"
    $ns at 0 "$modem_(2) start"
    $ns at 0 "$cbr_(1) start"
    $ns at 24 "$cbr_(1) stop"
    $ns at 40 "$modem_(1) stop"
    $ns at 40 "$modem_(2) stop"    

    $ns at 40 "finish; $ns halt"

}

$ns run
