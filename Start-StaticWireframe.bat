@echo off
cd /d "%~dp0"
if exist "%~dp0.venv\Scripts\python.exe" (
  "%~dp0.venv\Scripts\python.exe" "%~dp0new\static\fv\tk_wireframe.py"
  goto done
)
if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" (
  "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" "%~dp0new\static\fv\tk_wireframe.py"
  goto done
)
python "%~dp0new\static\fv\tk_wireframe.py"
:done
if errorlevel 1 pause
