#!/bin/bash
# computes bootloader size and returns JSON expression to be used by Arduino IDE 2 and arduino-cli
# 1st arg: size program
# 2nd arg: sketch
# 3rd arg: max flash
# 4th arg: max ram
# 5th arg: MCU
# 6th arg: bootload type
if [[ "$6" == "nobootloader" ]]; then #if no bootloader, use entire flash memory
    boot=0
elif [[ "$6" == "original_micronucleus" ]]; then
    boot=2180
elif [[ "$6" == "new_micronucleus" ]]; then
    boot=1412
elif [[ "$6" == "urboot" ]]; then
    boot=256
fi
if [[ -z "$boot" ]]; then
    maxflash=$3
    info="Could not determine bootloader size.\\n"
else
    maxflash=$(($3-boot))
fi
maxram=$4
flash=$("$1" -A "$2" | grep -E '^(.text|.data)\s+([0-9]+).*' | awk '{s+=$2}END{print s}')
ram=$("$1" -A "$2" | grep -E '^(.data|.bss|.noinit)\s+([0-9]+).*' | awk '{s+=$2}END{print s}')
flashpercent=$((flash*100/maxflash))
printf  '{  "output": "%sFlash memory used: %d bytes out of %d (%d%%).\\n' "$info" "$flash" "$maxflash" "$flashpercent"
printf  'RAM used for global variables: %d bytes out of %d.",' "$ram" "$maxram" 
if [[ $ram -gt $maxram ]]; then
    printf '"severity": "error", "error": "Not enough RAM", '
elif [[ $flash -gt $maxflash ]]; then
    printf '"severity": "error", "error": "Sketch is too big", '
elif [[ -z "$boot" ]]; then
    printf '"severity": "warning", "warning": "Could not determine bootloader size.", '
else
    printf '"severity": "info", '
fi
printf '"sections": [    { "name": "text", "size": %d, "max_size": %d },' "$flash" $maxflash 
printf '{ "name": "data", "size": %d, "max_size": %d }  ]}' "$ram" "$maxram"
