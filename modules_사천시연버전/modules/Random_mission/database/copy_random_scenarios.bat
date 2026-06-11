@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "BASE=%~dp0"
set "TARGET=%BASE%Scenario"

if not exist "%TARGET%\" (
    mkdir "%TARGET%"
    if errorlevel 1 (
        echo Failed to create target folder: "%TARGET%"
        exit /b 1
    )
)

set /a copied=0
set /a failed=0
set /a missing=0

for /d %%D in ("%BASE%Random_Scenario_*") do (
    set /a found=0

    call :CopyScenarioJson "%%~fD\Scenario"

    for /d %%P in ("%%~fD\*") do (
        call :CopyScenarioJson "%%~fP\Scenario"
    )

    if !found! equ 0 (
        echo Missing Scenario folder: "%%~nxD"
        set /a missing+=1
    )
)

echo.
echo Done.
echo Copied files: !copied!
echo Failed files: !failed!
echo Missing Scenario folders: !missing!

if !failed! gtr 0 exit /b 1
exit /b 0

:CopyScenarioJson
set "SOURCE=%~1"
if not exist "%SOURCE%\" exit /b 0

set /a found=1
for %%F in ("%SOURCE%\*.json") do (
    if exist "%%~fF" (
        copy /Y "%%~fF" "%TARGET%\" >nul
        if errorlevel 1 (
            echo Failed: "%%~fF"
            set /a failed+=1
        ) else (
            echo Copied: "%%~nxF"
            set /a copied+=1
        )
    )
)
exit /b 0
