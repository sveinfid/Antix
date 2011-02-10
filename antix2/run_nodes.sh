#!/bin/bash

for line in `cat workstations.txt`
do
	ssh -p 24 wjs2@$line.csil.sfu.ca /home/wjs2/Antix/antix2/run_node.tcl &
done
