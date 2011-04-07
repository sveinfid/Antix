#!/bin/bash
#
# This is run from start_csil.sh as it connects to each machine
#

ANTIX_PATH=/home/$USER/Antix
#export LD_LIBRARY_PATH=/home/$USER/sfuhome/protobuf/lib:/home/$USER/sfuhome/zeromq/lib
export LD_LIBRARY_PATH=/home/$USER/protobuf/lib:/home/$USER/zeromq/lib

if [ $# -ne 5 ]
then
  echo "Usage: $0 <IP of master> <# of teams> <robots per team> <nodes per machine> <AI library.so>"
  exit -1
fi

BASE_NODE_PORT=7919
BASE_GUI_PORT=8972

MASTER=$1
NUM_TEAMS=$2
ROBOTS_PER_TEAM=$3
NODES_PER_MACHINE=$4
AI_LIBRARY=$5
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

pkill -9 client &> /dev/null
pkill -9 node &> /dev/null
sleep 5

rm -f /home/$USER/clients.$HOST.log
rm -f /home/$USER/node.$HOST.log

# Start nodes
node_count=0
while [ $node_count -lt $NODES_PER_MACHINE ]
do
  # <master ip> <my ip> <node port> <gui port> <ipc suffix> <num teams>
  # XXX run node last to see segfault... or different solution for that
  NODE_PORT=`expr $BASE_NODE_PORT + $node_count`
  GUI_PORT=`expr $BASE_GUI_PORT + $node_count`
  $ANTIX_PATH/node $MASTER $MY_IP $NODE_PORT $GUI_PORT $node_count $NUM_TEAMS &>> /home/$USER/node.$HOST.log &

  # Start the clients for this node
  client_count=0
  while [ $client_count -lt $NUM_TEAMS ]
  do
    # <num robots> <team> <node ipc sock suffix>
    $ANTIX_PATH/client $ROBOTS_PER_TEAM $client_count $node_count $ANTIX_PATH/$AI_LIBRARY &>> /home/$USER/clients.$HOST.log &

    client_count=`expr $client_count + 1`
  done

  node_count=`expr $node_count + 1`
done

# Keep SSH open
#$ANTIX_PATH/scripts/sleep.tcl

# First start the node for this machine
#$ANTIX_PATH/scripts/run_node.tcl $MASTER $NUM_TEAMS &> /home/$USER/node.$HOST.log &
# <master ip> <my ip> <node port> <gui port> <ipc suffix> <num teams>
#$ANTIX_PATH/node $MASTER $MY_IP 3000 4000 0 $NUM_TEAMS &> /home/$USER/node.$HOST.log
