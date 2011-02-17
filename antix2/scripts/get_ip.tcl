#!/usr/bin/env tclsh8.5
#
# Output the IP of the current machine in CSIL
#

set ifconfig [exec ifconfig eth0]
regexp -- {inet addr:(.+?) } $ifconfig -> ip
puts -nonewline $ip
