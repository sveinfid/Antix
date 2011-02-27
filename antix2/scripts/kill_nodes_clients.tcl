#!/usr/bin/env tclsh8.5
#
# Connect to each workstation & kill both clients and nodes
#

if {$argc != 1} {
	puts "Usage: $argv0 <workstations.txt>"
	exit -1
}

set fname [lindex $argv 0]
set fid [open $fname r]
set workstations [read -nonewline $fid]
close $fid

set workstations [split $workstations \n]

foreach w $workstations {
	puts "${w}:"
 catch {exec -ignorestderr ssh -oStrictHostKeyChecking=no -p 24 ${w}.csil.sfu.ca pkill -9 client} output
 puts "\t${output}"
 catch {exec -ignorestderr ssh -oStrictHostKeyChecking=no -p 24 ${w}.csil.sfu.ca pkill -9 node} output
 puts "\t${output}"
}
