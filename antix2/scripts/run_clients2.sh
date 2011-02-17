#!/bin/bash
#
# Run COMMAND NUMBER_TO_RUN times in the background
#

NUMBER_TO_RUN=10
COMMAND=./test.tcl

count=0
while [ $count -lt $NUMBER_TO_RUN ]
do
	$COMMAND &
	count=`expr $count + 1`
done
