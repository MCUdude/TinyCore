#!/bin/bash
# 1st arg: size program
# 2nd arg: sketch
# 3rd arg: max flash
# 4th arg: max ram
# 5th arg: avrdude path
# 6th arg: config path
# 7th arg: MCU
# 8th arg: bootload name (if any)
if [[ -z "$8" ]]; then
    boot=0
else
    if [[ $("$5" -c dryrun -p "$7" -C "$6" "$8" -qq 2>/dev/null) ]]; then
        boot=$("$5" -c dryrun -p "$7" -C "$6" "$8" -qq | awk '{print $2}')
    else
        boot=256
    fi
fi
maxflash=$(($3-boot))
maxram=$4
flash=$($1 -A "$2" | grep -E '^(.text|.data)\s+([0-9]+).*' | awk '{s+=$2}END{print s}')
ram=$($1 -A "$2" | grep -E '^(.data|.bss|.noinit)\s+([0-9]+).*' | awk '{s+=$2}END{print s}')
flashpercent=$((flash*100/maxflash))
printf  '{  "output": "Flash memory used: %d bytes out of %d (%d%%).\\n' "$flash" $maxflash $flashpercent
printf  'RAM used for global variables: %d bytes out of %d.",' "$ram" "$maxram" 
if [[ $ram -gt $maxram ]]; then
    printf '"severity": "error", "error": "Not enough RAM", '
elif [[ $flash -gt $maxflash ]]; then
    printf '"severity": "error", "error": "Sketch is too big", '
else
    printf '"severity": "info", '
fi
printf '"sections": [    { "name": "text", "size": %d, "max_size": %d },' "$flash" $maxflash 
printf '{ "name": "data", "size": %d, "max_size": %d }  ]}' "$ram" "$maxram"
