#!/bin/bash
#
# Connect to every CSIL Linux machine and run a script which in turn starts
# a number of client processes on the local machine
#
# Since we have 78 machines to work with, and we need 1000 separate client
# processes, run 13clients/machine for the first 64, and then 12clients/machine
# for the final 14
#

ROBOTS_PER_CLIENT=10

count=0
for line in `cat workstations.txt_partial`
do
	# 13 for first 64
	#if [ $count -lt 64 ]
	#then
	#	ssh -p 24 wjs2@$line.csil.sfu.ca /home/wjs2/Antix/antix2/run_clients_local.sh 13 $ROBOTS_PER_CLIENT &
	# 12 for the rest
	#else
		ssh -p 24 wjs2@$line.csil.sfu.ca /home/wjs2/Antix/antix2/run_clients_local.sh 12 $ROBOTS_PER_CLIENT &
	#fi

	count=`expr $count + 1`
done
