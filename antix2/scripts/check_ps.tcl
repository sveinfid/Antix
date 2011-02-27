#!/usr/bin/env tclsh8.5
#
# Print readable output of processes for each workstation
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
 catch {exec -ignorestderr ssh -oStrictHostKeyChecking=no -p 24 ${w}.csil.sfu.ca ps x} output
 puts "\t${output}"
}
