@echo off
pwsh -NoProfile -ExecutionPolicy Bypass -File "%~dp0Scripts\Run.ps1"
if errorlevel 1 pause
