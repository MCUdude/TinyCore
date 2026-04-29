#!/bin/bash
# Parameters
# $1 = compiler command
# $2 = sketch path
# $3 = build path
# $4 = build project name
# $5 = flag name ('release_flags' or 'debug_flags')

$1 -fpreprocessed -dD -E -x c++ $2/$4 | grep -E "^\s*#\s*pragma\s+arduino\s+$5" | sed -E 's/# *pragma +arduino +(debug|release)_flags//'g | tr '\n' ' ' >$3/options.$5