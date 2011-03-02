#!/bin/bash
#
# This is run from start_csil.sh as it connects to each machine
#

ANTIX_PATH=/home/$USER/Antix/antix2

if [ $# -ne 3 ]
then
  echo "Usage: $0 <IP of master> <# of teams> <robots per team>"
  exit -1
fi

MASTER=$1
NUM_TEAMS=$2
ROBOTS_PER_TEAM=$3
HOST=`hostname`

# Before running simulation, remove old sockets. This can fail if node sockets
# exist from other users...
rm -rf /tmp/node*
if [ $? -ne 0 ]
then
	echo "Error: couldn't rm /tmp/node*" > /home/$USER/node.$HOST.log
	exit -1
fi

# First start the node for this machine
$ANTIX_PATH/scripts/run_node.tcl $MASTER $NUM_TEAMS &> /home/$USER/node.$HOST.log &

# Then start the client processes
rm /home/$USER/clients.$HOST.log
count=0
while [ $count -lt $NUM_TEAMS ]
do
  $ANTIX_PATH/scripts/run_client.tcl $ROBOTS_PER_TEAM $count &>> /home/$USER/clients.$HOST.log &
  count=`expr $count + 1`
done
