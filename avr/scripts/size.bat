@echo off
setlocal enabledelayedexpansion
REM Computes size of sketch (minus bootloader) and returns JSON expression to be used by Arduino IDE 2 and arduino-cli
REM 1st arg: size program
REM 2nd arg: sketch (ELF file)
REM 3rd arg: max flash
REM 4th arg: max ram
REM 5th arg: MCU
REM 6th arg: bootloader type

REM Arguments - strip surrounding quotes so paths with spaces work in sub-commands
set "sizeprog=%~1"
set "sketch=%~2"
set "maxflash=%~3"
set "maxram=%~4"
set "mcu=%~5"
set "boottype=%~6"

REM Determine bootloader size
set "boot="
if "%boottype%"=="nobootloader"          set boot=0
if "%boottype%"=="original_micronucleus" set boot=2180
if "%boottype%"=="new_micronucleus"      set boot=1412
if "%boottype%"=="urboot"                set boot=256

REM Calculate maxflash
set "info="
if defined boot (
   set /a maxflash=maxflash-boot
) else (
   set "info=Could not determine bootloader size.\n"
)

REM Calculate flash and RAM from avr-size output
REM The whole command is wrapped in an extra pair of quotes, because cmd /c
REM strips the outermost quotes; this way quoted paths with spaces or
REM forward slashes survive.
set flash=0
set ram=0
for /f "tokens=1,2" %%a in ('""%sizeprog%" -A "%sketch%""') do (
    if "%%a"==".text"   set /a flash+=%%b
    if "%%a"==".data"   set /a flash+=%%b
    if "%%a"==".data"   set /a ram+=%%b
    if "%%a"==".bss"    set /a ram+=%%b
    if "%%a"==".noinit" set /a ram+=%%b
)

REM Calculate percentage
set flashpercent=0
if %maxflash% GTR 0 set /a flashpercent=flash*100/maxflash

REM Determine severity (same precedence as size.sh: RAM error > flash error > warning > info)
set "severity=info"
set "errstr="
if %ram% GTR %maxram% (
   set "severity=error"
   set errstr="error": "Not enough RAM",
) else if %flash% GTR %maxflash% (
   set "severity=error"
   set errstr="error": "Sketch is too big",
) else if not defined boot (
   set "severity=warning"
   set errstr="warning": "Could not determine bootloader size.",
)

REM Output JSON
echo {"output": "%info%Flash memory used: %flash% bytes out of %maxflash% (%flashpercent%%%).\nRAM used for global variables: %ram% bytes out of %maxram%.","severity": "%severity%",%errstr%"sections": [{"name": "text","size": %flash%,"max_size": %maxflash%},{"name": "data","size": %ram%,"max_size": %maxram%}]}

exit /b 0
