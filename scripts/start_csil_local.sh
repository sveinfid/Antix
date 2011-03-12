#!/bin/bash
#
# This is run from start_csil.sh as it connects to each machine
#

ANTIX_PATH=/home/$USER/Antix
export LD_LIBRARY_PATH=/home/$USER/protobuf/lib:/home/$USER/zeromq/lib

if [ $# -ne 3 ]
then
  echo "Usage: $0 <IP of master> <# of teams> <robots per team>"
  exit -1
fi

MASTER=$1
NUM_TEAMS=$2
ROBOTS_PER_TEAM=$3
HOST=`hostname`
MY_IP=`$ANTIX_PATH/scripts/get_ip.tcl`

# Before running simulation, remove old sockets. This can fail if node sockets
# exist from other users...
rm -f /tmp/$USER-node*
if [ $? -ne 0 ]
then
	echo "Error: couldn't rm /tmp/$USER-node*" &> /home/$USER/node.$HOST.log
	exit -1
fi

# Then start the client processes
rm -f /home/$USER/clients.$HOST.log
count=0
while [ $count -lt $NUM_TEAMS ]
do
#  $ANTIX_PATH/scripts/run_client.tcl $ROBOTS_PER_TEAM $count &>> /home/$USER/clients.$HOST.log &
	# <num robots> <team> <node ipc sock suffix>
	$ANTIX_PATH/client $ROBOTS_PER_TEAM $count 0 &>> /home/$USER/clients.$HOST.log &
  count=`expr $count + 1`
done

# Keep SSH open
#$ANTIX_PATH/scripts/sleep.tcl

# First start the node for this machine
#$ANTIX_PATH/scripts/run_node.tcl $MASTER $NUM_TEAMS &> /home/$USER/node.$HOST.log &
# <master ip> <my ip> <node port> <gui port> <ipc suffix> <num teams>
$ANTIX_PATH/node $MASTER $MY_IP 3000 4000 0 $NUM_TEAMS &> /home/$USER/node.$HOST.log
