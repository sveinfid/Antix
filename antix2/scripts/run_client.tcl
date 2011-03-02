#!/usr/bin/env tclsh8.5
#
# Run a single client instance on the current machine
#

set path /home/wjs2/Antix/antix2

set ::env(ZMQ_PATH) /home/wjs2/zeromq
set ::env(PROTOBUF_PATH) /home/wjs2/protobuf
set ::env(LD_LIBRARY_PATH) /home/wjs2/zeromq/lib:/home/wjs2/protobuf/lib

if {$argc != 2} {
	puts "Usage: $argv0 <number of robots> <team ID>"
	return -1
}

set num_robots [lindex $argv 0]
set team_id [lindex $argv 1]

exec ${path}/client $num_robots $team_id 0
