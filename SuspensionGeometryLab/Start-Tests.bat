@echo off
setlocal
cd /d "%~dp0"
if not exist ".test-tmp" mkdir ".test-tmp"
set "TEMP=%~dp0.test-tmp"
set "TMP=%TEMP%"
set "SUSPENSION_GEOMETRY_LAB_GUI_TEST=1"
set "TOOL_PYTHON=python"
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" set "TOOL_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%~dp0.venv\Scripts\python.exe" set "TOOL_PYTHON=%~dp0.venv\Scripts\python.exe"
"%TOOL_PYTHON%" -B -m unittest discover -s tests -v
set "TEST_EXIT=%ERRORLEVEL%"
echo.
echo Test exit code: %TEST_EXIT%
pause
exit /b %TEST_EXIT%
