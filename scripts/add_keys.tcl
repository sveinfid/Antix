#!/usr/bin/env tclsh8.5
#
# Connect and add host key for every host in workstations
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
 catch {exec -ignorestderr ssh -oStrictHostKeyChecking=no -p 24 ${w}.csil.sfu.ca uname -a} output
 puts $output
}
