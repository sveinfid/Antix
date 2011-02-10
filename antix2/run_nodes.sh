#!/bin/bash

for line in `cat workstations.txt`
do
	echo $line
	ssh -p 24 wjs2@$line.csil.sfu.ca uname -a &
done
