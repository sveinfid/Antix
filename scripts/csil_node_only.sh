#!/bin/bash
#
# Run node only on local machine
#

ANTIX_PATH=/home/$USER/Antix
export LD_LIBRARY_PATH=/home/$USER/protobuf/lib:/home/$USER/zeromq/lib

if [ $# -ne 3 ]
then
	echo "Usage: $0 <IP of master> <num teams> <ipc id>"
	exit -1
fi

MASTER=$1
NUM_TEAMS=$2
IPC_ID=$3
HOST=`hostname`
MY_IP=`$ANTIX_PATH/scripts/get_ip.tcl`

rm -rf /tmp/node*
if [ $? -ne 0 ]
then
	echo "Error: couldn't rm /tmp/node*" > /home/$USER/node.$HOST.log
	exit -1
fi

# <master ip> <my ip> <node port> <gui port> <ipc suffix> <num teams>
$ANTIX_PATH/node $MASTER $MY_IP 3000 4000 $IPC_ID $NUM_TEAMS &> /home/$USER/node.$HOST.$IPC_ID.log &
