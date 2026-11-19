@echo off
setlocal
pushd "%~dp0.."
if exist "%~dp0.venv\Scripts\python.exe" (
  "%~dp0.venv\Scripts\python.exe" -X utf8 -m JacobianKinematicsLab %*
) else if exist "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" (
  "%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe" -X utf8 -m JacobianKinematicsLab %*
) else (
  python -X utf8 -m JacobianKinematicsLab %*
)
set "exit_status=%ERRORLEVEL%"
popd
if not "%exit_status%"=="0" pause
exit /b %exit_status%
