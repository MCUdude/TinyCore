@echo off
setlocal enabledelayedexpansion

REM Arguments - strip surrounding quotes so paths with spaces work in sub-commands
set "sizeprog=%~1"
set "sketch=%~2"
set "maxflash=%~3"
set "maxram=%~4"
set "avrdude=%~5"
set "config=%~6"
set "mcu=%~7"
set "bootname=%~8"

REM Determine bootloader size
if "%bootname%"=="" (
    set boot=0
) else (
    for /f "tokens=2" %%a in ('"%avrdude%" -c dryrun -p %mcu% -C "%config%" "%bootname%" -qq 2^>nul') do (
        set boot=%%a
    )
    if not defined boot set boot=256
)

REM Calculate maxflash
set /a maxflash=%maxflash% - %boot%

REM Calculate flash and RAM from avr-size output
set flash=0
set ram=0
for /f "tokens=1,2" %%a in ('"%sizeprog%" -A "%sketch%"') do (
    if "%%a"==".text"   set /a flash+=%%b
    if "%%a"==".data"   set /a flash+=%%b
    if "%%a"==".data"   set /a ram+=%%b
    if "%%a"==".bss"    set /a ram+=%%b
    if "%%a"==".noinit" set /a ram+=%%b
)

REM Calculate percentage
set /a flashpercent=flash*100/maxflash

REM Determine severity
set severity=info
if %ram% GTR %maxram% set severity=error
if %flash% GTR %maxflash% set severity=error

REM Output JSON
echo {"output": "Flash memory used: %flash% bytes out of %maxflash% (%flashpercent%%%). RAM used for global variables: %ram% bytes out of %maxram%.","severity": "%severity%","sections": [{"name": "text","size": %flash%,"max_size": %maxflash%},{"name": "data","size": %ram%,"max_size": %maxram%}]}