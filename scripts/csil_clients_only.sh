#!/bin/bash
#
# Run clients only on local machine
#

ANTIX_PATH=/home/$USER/Antix
export LD_LIBRARY_PATH=/home/$USER/protobuf/lib:/home/$USER/zeromq/lib

if [ $# -ne 3 ]
then
	echo "Usage: $0 <# of teams> <robots per team> <ipc id>"
	exit -1
fi

NUM_TEAMS=$1
ROBOTS_PER_TEAM=$2
IPC_ID=$3
HOST=`hostname`

rm -f /home/$USER/clients.$HOST.log
count=0
while [ $count -lt $NUM_TEAMS ]
do
	$ANTIX_PATH/client $ROBOTS_PER_TEAM $count $IPC_ID &>> /home/$USER/clients.$HOST.log &
	count=`expr $count + 1`
done
