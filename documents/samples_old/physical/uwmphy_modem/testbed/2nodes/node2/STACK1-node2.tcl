# STACK1: UWCBR/UWUDP/UWstaticROUTING/UWIP/UWMLL/ALOHA/MPHY_DUMB
# One node with a MFSK_WHOI_MM: public UWMPhy_modem plugged as PHY layer,
# i.e., a module that do UW network _TESTBED with the FSK WHOI.
#
#	                                +-----------------------+
#	                                | 7. UWCBR (rx or tx/rx)|
#		                        +-----------------------+
#	                                |       6. UWUDP        |
#		                        +-----------------------+
#	                                |  5. UWstaticROUTING   |
#		                        +-----------------------+
#	                                |       4. UWIP         |
#	                                +-----------------------+
#	                                |       3. UWMLL        |
#	                                +-----------------------+
#	                                |    2. CSMA-ALOHA      |
#		                       	+-----------------------+
#	                                |  1. MFSK_WHOI_MM      |
#		                       	+-----------------------+
#                                          |       
#        ____ real UW acoustic channel_____|           
#    

#Initialize the dynamic libraries
load libMiracle.so
load libcbrtracer.so
load libcsmaaloha.so
load libuwmll.so
load libuwip.so
load libuwstaticrouting.so
load libuwudp.so
load libuwcbr.so
load libmphy.so
load libuwmphy_modem.so


#Create a new simulator object
set ns [new Simulator]

#Declare the use of Miracle
$ns use-Miracle

#Declare the use of a Real Time Schedule (can be disabled)
$ns use-scheduler RealTime

#Choose the application: 1) "simplex or default" for tx->rx or 2) "duplex" for tx/rx <-> tx/rx
set application "simplex"
#Choose if to send minipacket or binary messages (see CCMUC or CCTXD messages, respectively, in the NMEA standard)
set useminipck "yes"

if {$application == "duplex"} {
   
   if {$useminipck == "yes"} {
       set tf_name "stack1-mfsk_whoi_mm-duplex-node2.tr"
   } else {
       set tf_name "stack1-mfsk_whoi_mm-duplex-node2.tr" 
   }

} else {
   
   if {$useminipck == "yes"} {
       set tf_name "stack1-mfsk_whoi_mm-simplex-node2.tr"
   } else {
       set tf_name "stack1-mfsk_whoi_mm-simplex-node2.tr" 
   }

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
	global ns serial_path node_ cbr_ transport_ port_ routing_ ipif_ mac_ modem_ ipif_ mll_ 
	
	#Create a new node (empty)
	set node_($i) [$ns create-M_Node]
	
	#Create the modules to add
	#APPLICATION LAYER
	set cbr_($i) [new Module/UW/CBR]
	$cbr_($i) set period_ 7
	$cbr_($i) set packetSize_ 256
	
	#TRANSPORT LAYER
	set transport_($i) [new Module/UW/UDP]
	#Assign a port number to the application considered (CBR)
	set port_($i) [$transport_($i) assignPort $cbr_($i)]
	
	#NETWORK LAYER
        #Static Routing
	set routing_($i) [new Module/UW/StaticRouting]
	
        #IP interface
	set ipif_($i) [new Module/UW/IP]
	$ipif_($i) addr "1.0.0.${i}"
	    
        #MAC LAYER
        set mac_($i) [new Module/MMac/ALOHA]
        #set mac_($i) [new Module/MMac/CSMA_ALOHA]
        #$mac_($i) setNoAckMode


	set mll_($i) [new Module/UW/MLL]

        #PHY LAYER
        set modem_($i) [new "Module/UW/MPhy_modem/FSK_WHOI_MM" $serial_path($i)]
        $modem_($i) set ID_ $i
        $modem_($i) set period_ 1
        $modem_($i) set setting_ 1
        $modem_($i) set stack_ 1 

	#Insert the modules in the node
	$node_($i) addModule 7 $cbr_($i) 1 "CBR"
	$node_($i) addModule 6 $transport_($i) 1 "TRANSPORT"
	$node_($i) addModule 5 $routing_($i) 1 "IPR"
	$node_($i) addModule 4 $ipif_($i) 1 "IPIF"
        $node_($i) addModule 3 $mll_($i) 1 "ARP TABLES"
	$node_($i) addModule 2 $mac_($i) 1 "ALOHA"
        $node_($i) addModule 1 $modem_($i) 1 "FSKwhoiMM"
 
	#Connect the modules
	$node_($i) setConnection $cbr_($i) $transport_($i) trace
	$node_($i) setConnection $transport_($i) $routing_($i) trace
	$node_($i) setConnection $routing_($i) $ipif_($i) trace
	$node_($i) setConnection $ipif_($i) $mll_($i) trace
	$node_($i) setConnection $mll_($i) $mac_($i) trace
	$node_($i) setConnection $mac_($i) $modem_($i) trace

}

#Create the actual application
#create-node 1
#puts "node 1 created!"
create-node 2
puts "node 2 created!"
puts  "IP address: [$ipif_(2) addr-string] equals to [$ipif_(2) addr]"
puts  "Port number: $port_(2)"
puts  "MAC address: [$mac_(2) addr]"


#Set topology
set dstIP 16777217
#$cbr_(1) set destAddr_ [$ipif_(2) addr]
#$cbr_(1) set destPort_ $port_(2)
$cbr_(2) set destAddr_ $dstIP
$cbr_(2) set destPort_ 1

#addRoute (destination, mask, next hop)
#$routing_(1) addRoute [$ipif_(2) addr-string] "255.255.255.255" [$ipif_(2) addr-string]
#defaultGateway (address) = addRoute (0, 0, address)
$routing_(2) defaultGateway "1.0.0.1"

#$mll($id1) addentry [$ipif($id2) addr] [$mac($id2) addr]
#$mll_(1) addentry [$ipif_(2) addr] [$mac_(2) addr]
$mll_(2) addentry $dstIP 0

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
    
    #$ns at 0 "$modem_(1) start"
    $ns at 0 "$modem_(2) start"
    #$ns at 0 "$cbr_(1) start"
    #$ns at 24 "$cbr_(1) stop"
    #$ns at 40 "$modem_(1) stop"
    $ns at 40 "$modem_(2) stop"    

    $ns at 40 "finish; $ns halt"

}

$ns run
