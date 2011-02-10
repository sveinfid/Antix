#!/bin/bash

for line in `cat workstations.txt`
do
	ssh -p 24 wjs2@$line.csil.sfu.ca killall -9 node &
done
