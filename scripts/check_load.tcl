#!/usr/bin/env tclsh8.5

catch {exec hostname} host
catch {exec uptime} uptime
puts "${host}: ${uptime}"
