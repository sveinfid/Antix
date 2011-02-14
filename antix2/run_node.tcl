#!/usr/bin/env tclsh8.5
#
# Run a single node instance on the current machine
# Assumes ports are free
#

set master_ip 142.58.35.231
set path /home/wjs2/Antix/antix2

set ::env(ZMQ_PATH) /home/wjs2/zeromq
set ::env(PROTOBUF_PATH) /home/wjs2/protobuf
set ::env(LD_LIBRARY_PATH) /home/wjs2/zeromq/lib:/home/wjs2/protobuf/lib

exec ${path}/node $master_ip [exec ${path}/get_ip.tcl] 3000 4000 5000 >> /dev/null