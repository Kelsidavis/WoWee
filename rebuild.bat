@echo off
REM Convenience wrapper — launches the PowerShell clean rebuild script.
powershell -ExecutionPolicy Bypass -File "%~dp0rebuild.ps1" %*
