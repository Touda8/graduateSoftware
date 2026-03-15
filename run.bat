@echo off
REM ============================================================
REM graduateSoftware Application Launcher
REM   Usage: run.bat [Release|Debug]
REM ============================================================
setlocal

REM Always change to the script directory (double-click safe)
set "PROJ=%~dp0"
pushd "%PROJ%" >nul 2>&1

set "CONFIG=Release"
if /I "%~1"=="Debug" set "CONFIG=Debug"

set "EXE=build\%CONFIG%\twoProjector.exe"

REM Auto-build if exe is missing
if not exist "%EXE%" (
    echo [INFO] %EXE% not found, building %CONFIG% first...
    call build.bat %CONFIG%
)

if not exist "%EXE%" (
    echo [ERROR] Executable not found after build: %EXE%
    pause
    popd & endlocal & exit /b 1
)

REM ---- DLL search paths (order matters; build dir first to avoid conflicts) ----
set "BUILD_DIR=%PROJ%build\%CONFIG%"
set "QT_DEPLOY=%PROJ%qt_deploy"
set "ANACONDA_BIN=D:\software\anaconda\Library\bin"
set "OPENCV_BIN=F:\project\Envlib\opencv455\opencv\build\x64\vc15\bin"
set "PCL_BIN=F:\project\Envlib\PCL1.12.1\bin"
set "VTK_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\VTK\bin"
set "BOOST_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\Boost\lib"
set "QHULL_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\Qhull\bin"

set "QT_PLUGIN_PATH=%PROJ%qt_deploy\plugins"
set "PATH=%BUILD_DIR%;%QT_DEPLOY%;%ANACONDA_BIN%;%VTK_BIN%;%OPENCV_BIN%;%PCL_BIN%;%BOOST_BIN%;%QHULL_BIN%;%PATH%"

REM Basic runtime preflight checks
if not exist "%QT_DEPLOY%\Qt5Core.dll" (
    echo [ERROR] Missing Qt runtime: %QT_DEPLOY%\Qt5Core.dll
    echo [ERROR] Please ensure qt_deploy is complete.
    pause
    popd & endlocal & exit /b 1
)

if not exist "%QT_PLUGIN_PATH%\platforms\qwindows.dll" (
    echo [ERROR] Missing Qt platform plugin: %QT_PLUGIN_PATH%\platforms\qwindows.dll
    pause
    popd & endlocal & exit /b 1
)

echo.
echo ============================================================
echo   Running twoProjector [%CONFIG%]
echo   EXE: %PROJ%%EXE%
echo ============================================================
echo.

start "twoProjector" /D "%PROJ%" "%PROJ%%EXE%"
set "RUN_RESULT=%ERRORLEVEL%"
if not "%RUN_RESULT%"=="0" (
    echo [WARN] Launcher returned non-zero exit code: %RUN_RESULT%
)

popd
endlocal & exit /b %RUN_RESULT%
