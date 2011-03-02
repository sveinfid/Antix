#!/usr/bin/env tclsh8.5
#
# Run a single node instance on the current machine
# Assumes ports are free
#

set path /home/wjs2/Antix/antix2

set ::env(ZMQ_PATH) /home/wjs2/zeromq
set ::env(PROTOBUF_PATH) /home/wjs2/protobuf
set ::env(LD_LIBRARY_PATH) /home/wjs2/zeromq/lib:/home/wjs2/protobuf/lib

if {$argc != 2} {
	puts "Error: Usage: $argv0 <master IP> <number of teams>"
	exit -1
}

set master_ip [lindex $argv 0]
set num_teams [lindex $argv 1]

exec ${path}/node $master_ip [exec ${path}/scripts/get_ip.tcl] 3000 4000 0 $num_teams
