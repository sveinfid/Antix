#!/bin/bash
#
# For each CSIL machine given in the text file, start one node and M client
# processes, until we reach csil machine N = number of nodes
#

WORKSTATIONS=workstations.txt_partial
ANTIX_PATH=/home/$USER/Antix

if [ $# -ne 5 ]
then
  echo "Usage: $0 <IP of master> <# of teams> <robots per team> <nodes per machine> <AI library.so>"
  exit -1
fi

MASTER=$1
NUM_TEAMS=$2
ROBOTS_PER_TEAM=$3
NODES_PER_MACHINE=$4
AI_LIBRARY=$5

if [ $NODES_PER_MACHINE -gt 2 -o $NODES_PER_MACHINE -lt 1 ]
then
	echo "Error: Nodes per machine must be between 1 and 2"
	exit -1
fi

#count=0
for line in `cat $ANTIX_PATH/scripts/$WORKSTATIONS`
do
	ssh -oStrictHostKeyChecking=no -p 24 $USER@$line.csil.sfu.ca $ANTIX_PATH/scripts/start_csil_local.sh $MASTER $NUM_TEAMS $ROBOTS_PER_TEAM $NODES_PER_MACHINE $AI_LIBRARY &
#  count=`expr $count + 1`
#  if [ $count -eq $NUM_NODES ]
#  then
#    break
#  fi
	sleep 2
done
