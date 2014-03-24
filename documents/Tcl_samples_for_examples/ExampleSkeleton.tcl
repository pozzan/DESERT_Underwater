#  Brief description of the script
# 
#  Simple draw of the simulated network topology
#
#
######################
# Simulation Options #
######################
# Put here flags to enable or disable high level options for the simulation setup (optional)
# e.g., 
# set opt(option_name)    option_value

#####################
# Library Loading   #
#####################
# Load here all the NS-Miracle libraries you need
# e.g.,
load library_name.so

#############################
# NS-Miracle initialization #
#############################
# You always need the following two lines to use the NS-Miracle simulator
set ns [new Simulator]
$ns use-Miracle

##################
# Tcl variables  #
##################
# Put here all the tcl variables you need for simulation management (optional), namely, values for the binded variables, location parameters, module configuration's parameters, ...
# e.g., 
# set opt(variable_name) variable_value


#########################
# Module Configuration  #
#########################
# Put here all the commands to set globally the initialization values of the binded variables (optional)
# e.g., 
# Module/UW/Module_Name set module_variable  variable_value

################################
# Procedure(s) to create nodes #
################################
# Define here one or more procedures that allow you to create as many different kind of nodes
proc createNode { id } {
    
    # include all the global variable you are going to use inside this procedure
    
    # build the NS-Miracle node
    
    # define the module(s) you want to put in the node
    
    # insert the module(s) into the node 
    
    # intra-node module connections (if needed)
    
    # set module and node parameters (optional)
   
    # initialize node's modules (if needed)
    
    # add node positions (optional)  
 
}

#################
# Node Creation #
#################
# Create here all the nodes you want to network together
# e.g., 
# createNode 1

################################
# Inter-node module connection #
################################
# Put here all the commands required to connect nodes in the network (optional), namely, specify end to end connections, fill ARP tables, define routing settings

#####################
# Start/Stop Timers #
#####################
# Set here the timers to start and/or stop modules (optional)
# e.g., 
# $ns at time_value "$module_name start"

###################
# Final Procedure #
###################
# Define here the procedure to call at the end of the simulation
proc finish {} {
	
   # computation of the statics

   # display messages

   # save traces

   # close files

}


##################
# Run simulation #
##################
# Specify the time at which to call the finish procedure and halt ns
# e.g.,
$ns at time_value  "finish; $ns halt" 

# You always need the following line to run the NS-Miracle simulator
$ns run
