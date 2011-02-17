#!/bin/bash
#
# Run one node on each machine
#

for line in `cat workstations.txt_nodes`
do
	ssh -p 24 wjs2@$line.csil.sfu.ca /home/wjs2/Antix/antix2/scripts/run_node.tcl &
done
