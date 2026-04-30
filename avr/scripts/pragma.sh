#!/bin/bash
# Parameters
# $1 = compiler command
# $2 = sketch path
# $3 = build path
# $4 = build project name
# $5 = flag name ('release_flags' or 'debug_flags')

# create previous option file if not already there
touch $3/options.$5

# make a backup copy if previous option file
cp $3/options.$5 $3/options.$5.bak

# create new option file
$1 -fpreprocessed -dD -E -x c++ $2/$4 | grep -E "^\s*#\s*pragma\s+arduino\s+$5" | sed -E 's/# *pragma +arduino +(debug|release)_flags//'g | tr '\n' ' ' >$3/options.$5

# compare old and new options
# if different, then delete all cached *.o and *.a files
# as well as all cached cores (Arduino IDE 2 and arduino-cli)
if  [[ `diff -q $3/options.$5 $3/options.$5.bak` ]]; then
    echo "Options changed: Delete cache!"
    rm -rf $3/core/*.a
    rm -rf $3/core/*.o
    rm -rf $3/sketch/*.a
    
    # delete cores at default locations
    if [[ "$OSTYPE" == "darwin"* ]]; then        # for the Mac:
        rm -rf ~/Library/Caches/arduino/cores    #    use cache folder in User Library
    else
        if [[ "$XDG_CACHE_HOME" != "" ]]; then   # for Linux check if environment var is set
            rm -rf $XDG_CACHE_HOME/arduino/cores             
        else                                     # otherwise use personal cache folder
            rm -rf ~/.config/arduino/cores
        fi
    fi

    # now delete at arduino-cli configured location
    CACHEPATH=`arduino-cli config get build_cache.path 2>/dev/null`
    if [[ "$CACHEPATH" != "" ]]; then
        rm -rf $CACHEPATH/cores
    fi
fi

#remove backup options
rm $3/options.$5.bak