@echo off
setlocal

set "ROOT=%~dp0"
set "PY="

rem Preferred layout: <BUNDLE>\DSS_KU\run_offline_runtime_check.bat
rem                   <BUNDLE>\miniconda3\python.exe
if exist "%ROOT%..\miniconda3\python.exe" set "PY=%ROOT%..\miniconda3\python.exe"

rem Alternate layout: script and miniconda3 in same folder.
if not defined PY if exist "%ROOT%miniconda3\python.exe" set "PY=%ROOT%miniconda3\python.exe"

rem Fallback (common path used in this project).
if not defined PY if exist "C:\DSS_OFFLINE_BUNDLE\miniconda3\python.exe" set "PY=C:\DSS_OFFLINE_BUNDLE\miniconda3\python.exe"

if not defined PY (
    echo [ERR] Bundled Python was not found.
    echo [ERR] Expected one of:
    echo       "%ROOT%..\miniconda3\python.exe"
    echo       "%ROOT%miniconda3\python.exe"
    echo       "C:\DSS_OFFLINE_BUNDLE\miniconda3\python.exe"
    pause
    exit /b 1
)

if not exist "%ROOT%offline_runtime_check.py" (
    echo [ERR] Missing "%ROOT%offline_runtime_check.py"
    pause
    exit /b 1
)

for %%I in ("%PY%") do set "PYDIR=%%~dpI"
if exist "%PYDIR%Library\bin" (
    set "PATH=%PYDIR%;%PYDIR%Library\bin;%PYDIR%Scripts;%PATH%"
)

echo [INFO] Python: "%PY%"
echo [INFO] Check script: "%ROOT%offline_runtime_check.py"
echo.
"%PY%" "%ROOT%offline_runtime_check.py"
set "EC=%ERRORLEVEL%"
echo.

if "%EC%"=="0" (
    echo [OK] Offline runtime check PASS
) else (
    echo [FAIL] Offline runtime check FAIL ^(exit=%EC%^)
)

echo [INFO] Report: "%ROOT%offline_runtime_check_report.json"
pause
exit /b %EC%

