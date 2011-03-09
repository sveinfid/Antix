#!/usr/bin/env tclsh8.5
#
# Print readable output of processes for each workstation
#

set ANTIX_PATH /home/${::env(USER)}/Antix/antix2

if {$argc != 2} {
	puts "Usage: $argv0 <workstations.txt> <show / count>"
	exit -1
}

set type [lindex $argv 1]
if {$type != "count" && $type != "show"} {
	puts "Usage: $argv0 <workstations.txt> <show / count>"
	exit -1
}

set fname [lindex $argv 0]
set fid [open $fname r]
set workstations [read -nonewline $fid]
close $fid

set workstations [split $workstations \n]

foreach w $workstations {
	puts "${w}:"
	if {$type == "show"} {
		catch {exec -ignorestderr ssh -oStrictHostKeyChecking=no -p 24 ${w}.csil.sfu.ca ps x} output
	} elseif {$type == "count"} {
		catch {exec -ignorestderr ssh -oStrictHostKeyChecking=no -p 24 ${w}.csil.sfu.ca $ANTIX_PATH/scripts/count_ps.tcl} output
	}
	puts "\t${output}"
}
