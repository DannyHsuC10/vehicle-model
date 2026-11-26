@echo off
setlocal
call "%~dp0Start-14DOF-Lap.bat" --gui
set "TTR_GUI_EXIT=%ERRORLEVEL%"
if not "%TTR_GUI_EXIT%"=="0" pause
exit /b %TTR_GUI_EXIT%
