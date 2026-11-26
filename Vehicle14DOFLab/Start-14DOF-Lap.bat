@echo off
setlocal
set "TTR_LAP_PYTHON="
call :probe "%~dp0.venv\Scripts\python.exe"
call :probe "%~dp0..\.venv\Scripts\python.exe"
call :probe "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
call :probe py -3
call :probe python
if not defined TTR_LAP_PYTHON (
    echo Python 3.10 or newer and NumPy are required.
    echo Install NumPy with: py -3 -m pip install -r "%~dp0requirements.txt"
    pause
    exit /b 1
)
%TTR_LAP_PYTHON% -B -X utf8 "%~dp0run.py" %*
set "TTR_LAP_EXIT=%ERRORLEVEL%"
if "%~1"=="" pause
exit /b %TTR_LAP_EXIT%

:probe
if defined TTR_LAP_PYTHON exit /b 0
%* -B -c "import sys, numpy; sys.exit(sys.version_info < (3, 10))" >nul 2>&1
if not errorlevel 1 set "TTR_LAP_PYTHON=%*"
exit /b 0
