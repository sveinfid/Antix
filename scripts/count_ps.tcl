#!/usr/bin/env tclsh8.5
#
# Count processes of simulation running on current machine; output
#

catch {exec hostname} host
catch {exec ps x | grep client | grep -v "grep" | wc -l} cli
catch {exec ps x | grep node | grep -v "grep" | wc -l} node
#catch {exec ps x | grep client | wc -l} cli
#catch {exec ps x | grep node | wc -l} node
puts "${host}: ${cli} clients, ${node} nodes (poss. incl. grep)"
