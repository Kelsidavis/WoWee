@echo off
REM Convenience wrapper — launches the PowerShell asset extraction script.
REM Usage: extract_assets.bat C:\Games\WoW\Data [expansion]
powershell -ExecutionPolicy Bypass -File "%~dp0extract_assets.ps1" %*
