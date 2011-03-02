#!/bin/bash
#
# Start a simulation using only the local machine
#
# Usage: ./start_local.sh <number of nodes> <number of teams> <robots per team>
#

if [ $# -ne 3 ]
then
	echo "Usage: $0 <number of nodes> <number of teams> <robots per team>"
	exit -1
fi

NUM_NODES=$1
NUM_TEAMS=$2
ROBOTS_PER_TEAM=$3

echo "Removing /tmp/node* ..."
rm /tmp/node*

# Start nodes first
i=0
while [ $i -lt $NUM_NODES ]
do
	neighbour_port=`expr 3000 + $i`
	gui_port=`expr 4000 + $i`
	# <neighbour port> <gui port> <node IPC id>
	./node 127.0.0.1 127.0.0.1 $neighbour_port $gui_port $i $NUM_TEAMS &
	i=`expr $i + 1`
done

# Then clients/teams
i=0
while [ $i -lt $NUM_TEAMS ]
do
	# Create one client process for each node
	node=0
	while [ $node -lt $NUM_NODES ]
	do
		# <#robots> <team id> <node IPC id>
		./client $ROBOTS_PER_TEAM $i $node &
		node=`expr $node + 1`
	done

	i=`expr $i + 1`
done

echo "Started $NUM_TEAMS teams on $NUM_NODES nodes."
