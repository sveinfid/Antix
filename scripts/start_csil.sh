#!/bin/bash
#
# For each CSIL machine given in the text file, start one node and M client
# processes, until we reach csil machine N = number of nodes
#

WORKSTATIONS=workstations.txt_partial
ANTIX_PATH=/home/wjs2/Antix
USER=wjs2

if [ $# -ne 3 ]
then
  echo "Usage: $0 <IP of master> <# of teams> <robots per team>"
  exit -1
fi

MASTER=$1
NUM_TEAMS=$2
ROBOTS_PER_TEAM=$3

#count=0
for line in `cat $ANTIX_PATH/scripts/$WORKSTATIONS`
do
	ssh -p 24 $USER@$line.csil.sfu.ca $ANTIX_PATH/scripts/start_csil_local.sh $MASTER $NUM_TEAMS $ROBOTS_PER_TEAM &
#  count=`expr $count + 1`
#  if [ $count -eq $NUM_NODES ]
#  then
#    break
#  fi
	sleep 2
done
