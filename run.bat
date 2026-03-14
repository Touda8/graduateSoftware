@echo off
REM graduateSoftware Application Launcher
REM Double-click to launch; auto-build if executable is missing

setlocal

REM Always run from script directory (double-click safe)
set "PROJ_DIR=%~dp0"
pushd "%PROJ_DIR%" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Cannot enter project directory: %PROJ_DIR%
    pause
    exit /b 1
)

REM Parse configuration (default Release)
set "CONFIG=Release"
if /I "%~1"=="Debug" set "CONFIG=Debug"

set "EXE_PATH=build\%CONFIG%\twoProjector.exe"

REM If exe missing, auto build once
if not exist "%EXE_PATH%" (
    echo.
    echo [INFO] %EXE_PATH% not found, auto building %CONFIG% ...
    call build.bat %CONFIG%
)

if not exist "%EXE_PATH%" (
    echo.
    echo ============================================================================
    echo ERROR: Executable still not found after build attempt.
    echo ============================================================================
    echo Expected: %CD%\%EXE_PATH%
    echo.
    pause
    popd
    exit /b 1
)

REM Runtime paths
set "QT_DEPLOY=%CD%\qt_deploy"
set "QT_PLUGIN_PATH=%QT_DEPLOY%\plugins"
set "BUILD_DIR=%CD%\build\%CONFIG%"
set "OPENCV_BIN=F:\project\Envlib\opencv455\opencv\build\x64\vc15\bin"
set "PCL_BIN=F:\project\Envlib\PCL1.12.1\bin"
set "VTK_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\VTK\bin"
set "BOOST_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\Boost\lib"
set "QHULL_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\Qhull\bin"
set "ANACONDA_BIN=D:\software\anaconda\Library\bin"

set "PATH=%BUILD_DIR%;%QT_DEPLOY%;%VTK_BIN%;%OPENCV_BIN%;%PCL_BIN%;%BOOST_BIN%;%QHULL_BIN%;%ANACONDA_BIN%;%PATH%"

echo.
echo ============================================================================
echo                   Running twoProjector Application
echo ============================================================================
echo Configuration:  %CONFIG%
echo Executable:     %CD%\%EXE_PATH%
echo Qt Plugins:     %QT_PLUGIN_PATH%
echo Working Dir:    %CD%
echo.

REM Launch GUI app (do not pass Debug/Release token to exe)
echo Launching application...
start "twoProjector" /D "%CD%" "%CD%\%EXE_PATH%"
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%"=="0" (
    echo.
    echo [ERROR] Failed to start application. Exit code: %EXIT_CODE%
    pause
)

popd
endlocal & exit /b %EXIT_CODE%
