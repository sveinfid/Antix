#!/usr/bin/env tclsh8.5
#
# Run a single client instance on the current machine
#

set path /home/wjs2/Antix/antix2

set ::env(ZMQ_PATH) /home/wjs2/zeromq
set ::env(PROTOBUF_PATH) /home/wjs2/protobuf
set ::env(LD_LIBRARY_PATH) /home/wjs2/zeromq/lib:/home/wjs2/protobuf/lib

if {$argc != 3} {
	puts "Usage: $argv0 <master ip> <number of robots> <team ID>"
	return -1
}

set master_ip [lindex $argv 0]
set num_robots [lindex $argv 1]
set team_id [lindex $argv 2]

exec ${path}/client $master_ip $num_robots $team_id 0
