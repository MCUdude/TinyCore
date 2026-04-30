@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM %1 = absolute path to avr-g++
REM %2 = sketch path
REM %3 = build path
REM %4 = build project name
REM %5 = flag name ("release_flags" or "debug_flags")

set "COMPILER=%~1"
set "SKETCH_PATH=%~2"
set "BUILD_PATH=%~3"
set "PROJECT_NAME=%~4"
set "FLAG_NAME=%~5"

if "%COMPILER%"=="" goto :usage
if "%SKETCH_PATH%"=="" goto :usage
if "%BUILD_PATH%"=="" goto :usage
if "%PROJECT_NAME%"=="" goto :usage
if "%FLAG_NAME%"=="" goto :usage

set "IN_FILE=%SKETCH_PATH%\%PROJECT_NAME%"
set "OUT_FILE=%BUILD_PATH%\options.%FLAG_NAME%"
set "BAK_FILE=%OUT_FILE%.bak"
set "TMP_OUT=%BUILD_PATH%\pragma_preproc.tmp"
set "TMP_ERR=%BUILD_PATH%\pragma_preproc.err"
set "OSKETCH=%BUILD_PATH%\sketch\\*.o" 
set "OSKETCH=%OSKETCH:\\=\%"
set "OCORE=%BUILD_PATH%\\core\\*.o" 
set "OCORE=%OCORE:\\=\%"
set "ACORE=%BUILD_PATH%\\core\\*.a" 
set "ACORE=%ACORE:\\=\%"
set "ALL_CORES=%BUILD_PATH%\\..\\..\\cores" 

if not exist "%BUILD_PATH%" mkdir "%BUILD_PATH%" >nul 2>nul

if not exist "%IN_FILE%" (
  echo ERROR: Input file not found: "%IN_FILE%"
  exit /b 3
)

if not exist "%OUT_FILE%" (
   copy NUL "%BAK_FILE%" >nul
) else (
   copy/y "%OUT_FILE%" "%BAK_FILE%" >nul
)

"%COMPILER%" -fpreprocessed -dD -E -x c++ "%IN_FILE%" 1>"%TMP_OUT%" 2>"%TMP_ERR%"
if errorlevel 1 (
  REM keep quiet by default; uncomment next 2 lines for debugging
  REM echo ERROR: Preprocessor failed. stderr:
  REM type "%TMP_ERR%"
  exit /b 4
)

set "OPTIONS="

for /f "usebackq delims=" %%L in ("%TMP_OUT%") do (
  set "LINE=%%L"

  REM 1) Convert TABs to spaces
  for /f "delims=" %%T in ("!LINE!") do set "LINE=%%T"
  REM Above looks like a no-op, but it keeps delayed expansion happy.

  REM Actual tab-to-space replacement:
  REM (batch can contain literal TAB between the quotes below)
  set "LINE=!LINE:	= !"

  REM 2) Left-trim spaces (repeat enough times)
  call :ltrim LINE

  REM 3) Drop leading '#' if present, then trim again
  if "!LINE:~0,1!"=="#" (
    set "LINE=!LINE:~1!"
    call :ltrim LINE
  )

  REM 4) Normalize multiple spaces down to single spaces
  call :collapse_spaces LINE

  REM Now we can match deterministically
  REM Accept both "pragma arduino debug_flags" and "pragma arduino release_flags"
  echo(!LINE! | findstr /I /C:"pragma arduino %FLAG_NAME% " >nul
  if not errorlevel 1 (
    REM Strip prefix
    set "LINE=!LINE:pragma arduino %FLAG_NAME% =!"

    REM Trim again, collapse again (in case there were extra spaces)
    call :ltrim LINE
    call :collapse_spaces LINE

    if defined LINE (
      if defined OPTIONS (
        set "OPTIONS=!OPTIONS! !LINE!"
      ) else (
        set "OPTIONS=!LINE!"
      )
    )
  )
)

> "%OUT_FILE%" (<nul set /p ="%OPTIONS% ")

FC %OUT_FILE% %BAK_FILE% > nul
if errorlevel 1 (
   echo "Options changed: Deleting cache"
   echo "%OSKETCH%"
   echo "%OCORE%"
   echo "%ACORE%"
   echo "%ALL_CORES%"
   del "%OSKETCH%" 2>nul
   del "%OCORE%" 2>nul
   del "%ACORE%" 2>nul
   rd /s/q "%ALL_CORES%" 2>nul
)
   
endlocal
exit /b 0

:ltrim
setlocal EnableDelayedExpansion
set "s=!%~1!"
:lt_loop
if defined s if "!s:~0,1!"==" " (
  set "s=!s:~1!"
  goto :lt_loop
)
endlocal & set "%~1=%s%"
exit /b

:collapse_spaces
setlocal EnableDelayedExpansion
set "s=!%~1!"
:cs_loop
set "t=!s:  = !"
if not "!t!"=="!s!" (
  set "s=!t!"
  goto :cs_loop
)
endlocal & set "%~1=%s%"
exit /b

:usage
echo Usage: %~nx0 ^<path-to-avr-g++^> ^<sketch_path^> ^<build_path^> ^<project_name^> ^<debug_flags^|release_flags^>
exit /b 2
