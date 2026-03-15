@echo off
REM graduateSoftware Build Script
REM Usage: build.bat [Release|Debug]

setlocal EnableExtensions

set "PROJ_DIR=%~dp0"
pushd "%PROJ_DIR%" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Cannot enter project directory: %PROJ_DIR%
    exit /b 1
)

set "CONFIG=Release"
if /I "%~1"=="Debug" set "CONFIG=Debug"
if /I "%~1"=="Release" set "CONFIG=Release"

set "MSBUILD_EXE=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe"
if not exist "%MSBUILD_EXE%" (
    echo [ERROR] MSBuild not found: "%MSBUILD_EXE%"
    popd
    exit /b 1
)

if not exist "logs" mkdir "logs"
set "LOGFILE=logs\build_%CONFIG%.log"

echo.
echo ============================================================================
echo Building twoProjector.sln (%CONFIG% ^| x64^)
echo ============================================================================
echo.

echo [%date% %time%] Build started: Configuration=%CONFIG%, Platform=x64 > "%LOGFILE%"

REM Verified build command pattern
"%MSBUILD_EXE%" "twoProjector.sln" "-p:Configuration=%CONFIG%" "-p:Platform=x64" >> "%LOGFILE%" 2>&1
set "BUILD_RESULT=%ERRORLEVEL%"

if not "%BUILD_RESULT%"=="0" (
    echo [ERROR] Build failed. See log: %CD%\%LOGFILE%
    popd
    exit /b %BUILD_RESULT%
)

set "EXE_PATH=build\%CONFIG%\twoProjector.exe"
if not exist "%EXE_PATH%" (
    echo [ERROR] Build reported success, but executable was not found:
    echo         %CD%\%EXE_PATH%
    echo [INFO] Check log: %CD%\%LOGFILE%
    popd
    exit /b 1
)

echo [OK] Build succeeded.
echo [OK] Executable: %CD%\%EXE_PATH%
echo [OK] Log file:   %CD%\%LOGFILE%

popd
endlocal
exit /b 0
