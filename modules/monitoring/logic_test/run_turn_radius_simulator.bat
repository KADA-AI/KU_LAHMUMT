@echo off
setlocal
set "SCRIPT_DIR=%~dp0"

if exist "%SCRIPT_DIR%turn_radius_simulator.exe" (
    start "" "%SCRIPT_DIR%turn_radius_simulator.exe"
    exit /b 0
)

if exist "%SCRIPT_DIR%..\..\..\..\miniconda3\python.exe" (
    "%SCRIPT_DIR%..\..\..\..\miniconda3\python.exe" "%SCRIPT_DIR%turn_radius_simulator.py"
    exit /b %errorlevel%
)

python "%SCRIPT_DIR%turn_radius_simulator.py"
