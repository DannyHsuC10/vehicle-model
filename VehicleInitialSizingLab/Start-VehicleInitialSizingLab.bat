@echo off
cd /d "%~dp0"
if exist "%~dp0.venv\Scripts\python.exe" (
  "%~dp0.venv\Scripts\python.exe" run.py %*
  goto done
)
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" (
  "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" run.py %*
  goto done
)
py -3 --version >nul 2>&1
if not errorlevel 1 (
  py -3 run.py %*
  goto done
)
python run.py %*
:done
if errorlevel 1 pause
