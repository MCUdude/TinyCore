#!/bin/bash
# Parameters
# $1 = compiler command
# $2 = sketch path
# $3 = build path
# $4 = build project name
# $5 = build_cache path
# $6 = flag name ('release_flags' or 'debug_flags')

# create previous option file if not already there
touch $3/options.$6

# make a backup copy if previous option file
cp $3/options.$6 $3/options.$6.bak

# create new option file
$1 -fpreprocessed -dD -E -x c++ $2/$4 | grep -E "^\s*#\s*pragma\s+arduino\s+$6" | sed -E 's/# *pragma +arduino +(debug|release)_flags//'g | tr '\n' ' ' >$3/options.$6

# compare old an new
if  [[ `diff -q $3/options.$6 $3/options.$6.bak` ]]; then
    echo "Options changed: Delete cache!"
    rm -rf $3/core/*.a
    rm -rf $3/core/*.o
    rm -rf $3/sketch/*.a
    if [[ "$5" == "{build_cache.path}" ]]; then      # if nothing has been specified 
        if [[ "$OSTYPE" == "darwin"* ]]; then        # for the Mac:
            rm -rf ~/Library/Caches/arduino/cores    #    use cache folder in User Library
        else
            if [[ "$XDG_CACHE_HOME" != "" ]]; then   # for Linux check if environment var is set
                rm -rf $XDG_CACHE_HOME             
            else                                     # otherwise use personal cache folder
                rm -rf ~/.config/arduino/cores
            fi
        fi
    else                                             # if build_cache.path has been specified:
        rm -rf $5
    fi
fi

#remove backup options
rm $3/options.$6.bak