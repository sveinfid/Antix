#!/bin/bash
#
# This script runs N client processes on the local machine, each with M robots
#

if [ $# -ne 2 ]
then
	echo "Usage: $0 <number of client processes> <robots per client process>"
	exit -1
fi

count=0
while [ $count -lt $1 ]
do
	/home/wjs2/Antix/antix2/run_client.tcl $2
	A=`expr $A + 1`
done
