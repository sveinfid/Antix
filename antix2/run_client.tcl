#!/usr/bin/env tclsh8.5
#
# Run a single client instance on the current machine
#

# pomelo.csil.sfu.ca
set master_ip 142.58.35.231
set path /home/wjs2/Antix/antix2

set ::env(ZMQ_PATH) /home/wjs2/zeromq
set ::env(PROTOBUF_PATH) /home/wjs2/protobuf
set ::env(LD_LIBRARY_PATH) /home/wjs2/zeromq/lib:/home/wjs2/protobuf/lib

if {$argc != 1} {
	puts "Usage: $argv0 <number of robots>"
	return -1
}

set num_robots [lindex $argv 0]

exec ${path}/client $master_ip $num_robots
