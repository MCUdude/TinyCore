#!/bin/bash
# Parameters
# $1 = compiler command
# $2 = sketch path
# $3 = build path
# $4 = build project name
# $5 = flag name ('build_flags, 'release_flags', or 'debug_flags')

# create previous option file if not already there
touch $3/options.$5

# make a backup copy 
cp $3/options.$5 $3/options.$5.bak

# create new option file
$1 -fpreprocessed -dD -E -x c++ $2/$4 | grep -E "^\s*#\s*pragma\s+arduino\s+$5" | sed -E 's/# *pragma +arduino +(debug|release|build)_flags//'g | tr '\n' ' ' >$3/options.$5

# compare old and new options
# if different, then delete all cached *.o and *.a files
# as well as all cached cores: Go up one directory level; if
# there are only two directories called 'sketches' and 'cores',
# and 'cores' contains only directories with just one file
# 'core.a', then delete the 'cores' folder.
if  [[ `diff -q $3/options.$5 $3/options.$5.bak` ]]; then
    echo "Options changed: Delete cached object files"
    rm -rf $3/core/*.a
    rm -rf $3/core/*.o
    rm -rf $3/sketch/*.a
    
    if [[ -d $3/../../cores ]] &&  [[ -d $3/../../sketches ]]; then
        filenums=`find $3/../../ -maxdepth 1 -print | wc -l`
        if [[ $filenums -eq 3 ]]; then
            corefolders=`find $3/../../cores -type d -print | wc -l`
            immediate=`find $3/../../cores -maxdepth 1 -print | wc -l`
            corefiles=`find $3/../../cores -type f -name "core.a" -print -or -name ".last-used" -print | wc -l`
            if [[ $(($corefiles/2)) -eq $(($corefolders-1)) ]] && [[ $corefolders -eq $immediate ]]; then
                echo "Delete cached cores"
                rm -rf $3/../../cores
            fi
        fi
     fi
fi

#remove backup options
rm $3/options.$5.bak

