@echo off
REM ============================================================================
REM graduateSoftware Build Script
REM   Builds twoProjector.sln for Visual Studio 2019 (x64 platform)
REM   Output: build\Release\ or build\Debug\
REM   Logs:   logs\build_YYYYMMDD_HHMM.log
REM ============================================================================

setlocal enabledelayedexpansion

REM Parse command line
set CONFIG=%~1
if "%CONFIG%"=="" set CONFIG=Release

echo.
echo ============================================================================
echo                   graduateSoftware Build System
echo ============================================================================
echo Configuration: %CONFIG%
echo Platform:     x64
echo.

REM Create required directories
if not exist "build" mkdir build
if not exist "build\Debug\obj" mkdir build\Debug\obj
if not exist "build\Release\obj" mkdir build\Release\obj
if not exist "logs" mkdir logs

REM Generate timestamp for log filename
for /f "tokens=2-4 delims=/ " %%a in ('date /t') do (set mydate=%%c%%a%%b)
for /f "tokens=1-2 delims=/:" %%a in ('time /t') do (set mytime=%%a%%b)
set LOGFILE=logs\build_%mydate%_%mytime%.log

echo [%date% %time%] === Build Started === > "%LOGFILE%"
echo Configuration: %CONFIG% >> "%LOGFILE%"

echo.
echo [Step 1] Verifying and preparing required files...
set FILES_OK=1

if not exist "twoProjector.sln" (
    echo ERROR: twoProjector.sln not found!
    echo ERROR: twoProjector.sln not found! >> "%LOGFILE%"
    set FILES_OK=0
)

if not exist "twoProjector.vcxproj" (
    echo ERROR: twoProjector.vcxproj not found!
    echo ERROR: twoProjector.vcxproj not found! >> "%LOGFILE%"
    set FILES_OK=0
)

if not exist "twoProjector.ui" (
    echo ERROR: twoProjector.ui not found!
    echo ERROR: twoProjector.ui not found! >> "%LOGFILE%"
    set FILES_OK=0
)

if %FILES_OK% EQU 0 (
    echo.
    echo BUILD FAILED - Missing required source files
    exit /b 1
)

REM Regenerate ui_twoProjector.h from .ui file using Anaconda's uic.exe
set UIC_EXE=D:\software\anaconda\Library\bin\uic.exe
if exist "%UIC_EXE%" (
    echo   Generating ui_twoProjector.h via uic.exe...
    "%UIC_EXE%" twoProjector.ui -o ui_twoProjector.h >> "%LOGFILE%" 2>&1
    if !ERRORLEVEL! NEQ 0 (
        echo ERROR: uic.exe failed to generate ui_twoProjector.h!
        exit /b 1
    )
    echo   ui_twoProjector.h generated successfully.
    echo ui_twoProjector.h generated OK >> "%LOGFILE%"
) else (
    echo WARNING: uic.exe not found at %UIC_EXE%
    echo   Using existing ui_twoProjector.h (may be out of date)
    if not exist "ui_twoProjector.h" (
        echo ERROR: ui_twoProjector.h not found and uic.exe unavailable!
        exit /b 1
    )
)

echo Required files verified OK >> "%LOGFILE%"

REM Also generate moc_ProjectorManager.cpp
set MOC_EXE=D:\software\anaconda\Library\bin\moc.exe
if exist "%MOC_EXE%" (
    echo   Generating moc_ProjectorManager.cpp via moc.exe...
    "%MOC_EXE%" -Isrc -I"D:\software\anaconda\Library\include\qt" -I"D:\software\anaconda\Library\include\qt\QtCore" -I"D:\software\anaconda\Library\include\qt\QtGui" -I"D:\software\anaconda\Library\include\qt\QtWidgets" src\common\ProjectorManager.h -o src\moc_ProjectorManager.cpp >> "%LOGFILE%" 2>&1
    echo   moc_ProjectorManager.cpp generated.
) else (
    echo WARNING: moc.exe not found at %MOC_EXE%
    echo   Using existing moc_ProjectorManager.cpp
)

echo.
echo [Step 2] Locating Visual Studio 2019...
set VS_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019
set VSDEVENV_FOUND=0

if exist "%VS_PATH%\Enterprise\Common7\Tools\VsDevCmd.bat" (
    echo   Found: VS2019 Enterprise
    call "%VS_PATH%\Enterprise\Common7\Tools\VsDevCmd.bat" -arch=x64 >> "%LOGFILE%" 2>&1
    set VSDEVENV_FOUND=1
) else if exist "%VS_PATH%\Professional\Common7\Tools\VsDevCmd.bat" (
    echo   Found: VS2019 Professional
    call "%VS_PATH%\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64 >> "%LOGFILE%" 2>&1
    set VSDEVENV_FOUND=1
) else if exist "%VS_PATH%\Community\Common7\Tools\VsDevCmd.bat" (
    echo   Found: VS2019 Community
    call "%VS_PATH%\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 >> "%LOGFILE%" 2>&1
    set VSDEVENV_FOUND=1
)

if %VSDEVENV_FOUND% EQU 0 (
    echo ERROR: Visual Studio 2019 not found at "%VS_PATH%"
    echo ERROR: Visual Studio 2019 not found >> "%LOGFILE%"
    exit /b 1
)

echo VS2019 environment initialized >> "%LOGFILE%"

echo.
echo [Step 3] Starting MSBuild...
echo Building %CONFIG%^|x64 ...
echo.
msbuild twoProjector.sln /p:Configuration=%CONFIG% /p:Platform=x64 /m /nologo >> "%LOGFILE%" 2>&1

set BUILD_RESULT=%ERRORLEVEL%

if %BUILD_RESULT% NEQ 0 (
    echo.
    echo ============================================================================
    echo BUILD FAILED
    echo ============================================================================
    echo Error Code: %BUILD_RESULT%
    echo Log File:  %LOGFILE%
    echo.
    echo Showing last 50 lines of build log:
    echo ------------------------------------------------------------------------
    for /f %%i in ('find /c /v "" "%LOGFILE%"') do set lines=%%i
    set skip=!lines!
    if !skip! gtr 50 set /a skip=!skip!-50
    if !skip! gtr 0 (
        more +!skip! "%LOGFILE%"
    ) else (
        type "%LOGFILE%"
    )
    echo.
    echo [%date% %time%] === Build Failed === >> "%LOGFILE%"
    exit /b %BUILD_RESULT%
)

echo.
echo ============================================================================
echo BUILD SUCCEEDED!
echo ============================================================================
echo.
echo [%date% %time%] === Build Succeeded === >> "%LOGFILE%"

REM Step 5: Copy runtime dependencies
echo [Step 4] Copying runtime dependencies...

set DLL_SRC=Project2\Project2\cyusbserial.dll
if exist "%DLL_SRC%" (
    echo Copying cyusbserial.dll ...
    xcopy /Y /Q "%DLL_SRC%" "build\%CONFIG%\" >> "%LOGFILE%" 2>&1
    if !ERRORLEVEL! EQU 0 (
        echo   cyusbserial.dll copied successfully
        echo cyusbserial.dll copied successfully >> "%LOGFILE%"
    ) else (
        echo   WARNING: Failed to copy cyusbserial.dll
        echo WARNING: cyusbserial.dll copy failed >> "%LOGFILE%"
    )
) else (
    echo WARNING: %DLL_SRC% not found (optional)
    echo WARNING: %DLL_SRC% not found >> "%LOGFILE%"
)

REM Step 6: Verify executable
echo.
echo [Step 5] Verifying output...
if exist "build\%CONFIG%\twoProjector.exe" (
    echo.
    echo [OK] Executable ready: build\%CONFIG%\twoProjector.exe
    echo [OK] Build log: %LOGFILE%
    echo.
    echo Next step: run.bat
    echo.
) else (
    echo.
    echo ERROR: Executable not found at build\%CONFIG%\twoProjector.exe
    echo This may indicate incomplete compilation.
    exit /b 1
)

endlocal
exit /b 0
