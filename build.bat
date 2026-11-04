@echo off
rem ===================================================================
rem  Headless build of 40_player_release  (synthesis + place-and-route + bitgen)
rem  IMPORTANT: close the TD GUI before running this.
rem
rem  Why: the TD GUI keeps the source-file list in memory and rewrites
rem  lab_ex5_i2s_v1.0.al when it saves, which can drop 6 modules and
rem  produce "black box" errors. This script restores the known-good
rem  file list (*.al.good / *.prj.good) before compiling.
rem ===================================================================
setlocal enabledelayedexpansion
set TD=D:\Anlogic\bin\td_commands_prompt.exe
set FLOW=D:/Anlogic/doc/scripts/DefaultFlow.tcl
if not exist "%TD%" set TD=D:\TD\bin\td_commands_prompt.exe
if not exist "%TD%" set TD=D:\TD\bin\td_commands_prompt.exe
if not exist "D:\Anlogic\doc\scripts\DefaultFlow.tcl" set FLOW=D:/TD/doc/scripts/DefaultFlow.tcl
set RUNS=%~dp0src\td_project\lab_ex5_i2s_v1.0_Runs
set PRJDIR=%~dp0src\td_project

if exist "%PRJDIR%\lab_ex5_i2s_v1.0.al.good" copy /y "%PRJDIR%\lab_ex5_i2s_v1.0.al.good" "%PRJDIR%\lab_ex5_i2s_v1.0.al" >nul
if exist "%RUNS%\syn_1\lab_ex5_i2s_v1.0.prj.good" copy /y "%RUNS%\syn_1\lab_ex5_i2s_v1.0.prj.good" "%RUNS%\syn_1\lab_ex5_i2s_v1.0.prj" >nul
if exist "%RUNS%\syn_1\settings.cfg.good" copy /y "%RUNS%\syn_1\settings.cfg.good" "%RUNS%\syn_1\settings.cfg" >nul
if exist "%RUNS%\phy_1\lab_ex5_i2s_v1.0.prj.good" copy /y "%RUNS%\phy_1\lab_ex5_i2s_v1.0.prj.good" "%RUNS%\phy_1\lab_ex5_i2s_v1.0.prj" >nul
if exist "%RUNS%\phy_1\settings.cfg.good" copy /y "%RUNS%\phy_1\settings.cfg.good" "%RUNS%\phy_1\settings.cfg" >nul

for %%F in ("%RUNS%\phy_1\*.bit") do set BEFORE=%%~tF
for %%F in ("%RUNS%\syn_1\lab_ex5_i2s_v1.0_gate.db") do set GATEBEFORE=%%~tF
echo Started        : %DATE% %TIME%
echo Previous bit   : !BEFORE!
echo.

echo [1/2] Synthesis ...
cd /d "%RUNS%\syn_1"
"%TD%" "%FLOW%"

set "SYNLOG="
for /f "delims=" %%F in ('dir /b /o-d "%RUNS%\syn_1\td_*.log" 2^>nul') do if not defined SYNLOG set "SYNLOG=%RUNS%\syn_1\%%F"
echo Newest syn log : !SYNLOG!
findstr /C:"HDL-8007 ERROR" "!SYNLOG!" >nul 2>&1
if not errorlevel 1 (
    echo.
    echo *** SYNTHESIS ERROR - black box module, file list is wrong ***
    findstr /C:"HDL-8007 ERROR" "!SYNLOG!"
    exit /b 1
)
findstr /C:"Successfully analyzed 46 source files" "!SYNLOG!" >nul 2>&1
if errorlevel 1 (
    echo.
    echo *** WARNING: source file count is not 46 - check the file list ***
)
for %%F in ("%RUNS%\syn_1\lab_ex5_i2s_v1.0_gate.db") do set GATEAFTER=%%~tF
if "!GATEBEFORE!"=="!GATEAFTER!" (
    echo *** SYNTHESIS DID NOT RUN *** gate.db unchanged: !GATEAFTER!
    exit /b 1
)
echo Synthesis OK - gate.db : !GATEAFTER!

echo.
echo [2/2] Place ^& Route + bitgen ...
cd /d "%RUNS%\phy_1"
"%TD%" "%FLOW%"

echo.
for %%F in ("%RUNS%\phy_1\*.bit") do set AFTER=%%~tF
echo === RESULT ===
echo Previous bit : !BEFORE!
echo Current  bit : !AFTER!
if "!BEFORE!"=="!AFTER!" (
    echo.
    echo *** FAILED *** the .bit was NOT regenerated - look at run.log
    exit /b 1
)
echo.
if not exist "%~dp0bit" mkdir "%~dp0bit"
copy /y "%RUNS%\phy_1\lab_ex5_i2s_v1.0.bit" "%~dp0bit\player_v34_release.bit" >nul
echo.
echo === DONE ===
echo Bitstream : %RUNS%\phy_1\lab_ex5_i2s_v1.0.bit
echo Copy      : %~dp0bit\player_v34_release.bit
exit /b 0
