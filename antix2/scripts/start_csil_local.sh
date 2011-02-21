#!/bin/bash
#
# This is run from start_csil.sh as it connects to each machine
#

ANTIX_PATH=/home/wjs2/Antix/antix2

if [ $# -ne 3 ]
then
  echo "Usage: $0 <IP of master> <# of teams> <robots per team>"
  exit -1
fi

MASTER=$1
NUM_TEAMS=$2
ROBOTS_PER_TEAM=$3

# First start the node for this machine
$ANTIX_PATH/scripts/run_node.tcl $MASTER &

# Then start the client processes
count=0
while [ $count -lt $NUM_TEAMS ]
do
  $ANTIX_PATH/scripts/run_client.tcl $MASTER $ROBOTS_PER_TEAM $count &
  count=`expr $count + 1`
done
