@echo off
REM Convenience wrapper — launches the PowerShell texture debug script.
powershell -ExecutionPolicy Bypass -File "%~dp0debug_texture.ps1" %*
