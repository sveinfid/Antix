#!/usr/bin/env tclsh8.5

catch {exec -ignorestderr pkill -9 client} output
catch {exec -ignorestderr pkill -9 node} output
catch {exec -ignorestderr pkill -9 tclsh8.5} output
