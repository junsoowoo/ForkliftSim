@echo off
setlocal
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -ExecutionPolicy Bypass -File "%~dp0Scripts\Start.ps1" %*
set "LaunchExit=%errorlevel%"
if not "%LaunchExit%"=="0" pause
exit /b %LaunchExit%
