import os

BUILD_BAT = r"""@echo off
REM ============================================================
REM graduateSoftware Build Script   Usage: build.bat [Release|Debug]
REM ============================================================
setlocal enabledelayedexpansion

set "MSBUILD=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\MSBuild.exe"
set "UIC=D:\software\anaconda\Library\bin\uic.exe"
set "MOC=D:\software\anaconda\Library\bin\moc.exe"
set "MOCINC=-Isrc -I"D:\software\anaconda\Library\include\qt" -I"D:\software\anaconda\Library\include\qt\QtCore" -I"D:\software\anaconda\Library\include\qt\QtGui" -I"D:\software\anaconda\Library\include\qt\QtWidgets" -I"F:\project\Envlib\PCL1.12.1\3rdParty\VTK\include\vtk-9.1""

set "CONFIG=%~1"
if "%CONFIG%"=="" set "CONFIG=Release"

if not exist "build\Release\obj" mkdir "build\Release\obj"
if not exist "build\Debug\obj"   mkdir "build\Debug\obj"
if not exist "logs"              mkdir "logs"

for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value 2^>nul') do set "DT=%%I"
set "LOGFILE=logs\build_%DT:~0,8%_%DT:~8,6%.log"

echo.
echo ============================================================
echo   graduateSoftware Build  [%CONFIG%^|x64]
echo ============================================================
echo [%DATE% %TIME%] Build started > "%LOGFILE%"

echo [1/4] Check MSBuild...
if not exist "%MSBUILD%" (
    echo [ERROR] MSBuild not found: %MSBUILD%
    goto :fail
)
echo       OK

echo [2/4] uic - ui_twoProjector.h...
if exist "%UIC%" (
    "%UIC%" "twoProjector.ui" -o "ui_twoProjector.h" >> "%LOGFILE%" 2>&1
    if !ERRORLEVEL! NEQ 0 (echo [ERROR] uic failed & goto :fail)
    echo       OK
) else (
    echo [WARN] uic.exe not found, using existing ui_twoProjector.h
    if not exist "ui_twoProjector.h" (echo [ERROR] ui_twoProjector.h missing! & goto :fail)
)

echo [3/4] moc...
if exist "%MOC%" (
    "%MOC%" %MOCINC% "src\MainWindow.h" -o "src\moc_MainWindow.cpp" >> "%LOGFILE%" 2>&1
    if !ERRORLEVEL! NEQ 0 (echo [ERROR] moc MainWindow.h failed & goto :fail)
    echo       OK: moc_MainWindow.cpp

    "%MOC%" %MOCINC% "src\visualization\VtkWidget.h" -o "src\visualization\moc_VtkWidget.cpp" >> "%LOGFILE%" 2>&1
    if !ERRORLEVEL! NEQ 0 (echo [ERROR] moc VtkWidget.h failed & goto :fail)
    echo       OK: moc_VtkWidget.cpp

    "%MOC%" %MOCINC% "src\common\ProjectorManager.h" -o "src\moc_ProjectorManager.cpp" >> "%LOGFILE%" 2>&1
    if !ERRORLEVEL! NEQ 0 (echo [ERROR] moc ProjectorManager.h failed & goto :fail)
    echo       OK: moc_ProjectorManager.cpp
) else (
    echo [WARN] moc.exe not found, skipping
)

echo [4/4] MSBuild (%CONFIG%^|x64)...
echo.
"%MSBUILD%" "twoProjector.sln" /p:Configuration=%CONFIG% /p:Platform=x64 /m /nologo >> "%LOGFILE%" 2>&1
set "BUILD_EC=!ERRORLEVEL!"

if !BUILD_EC! NEQ 0 (
    echo.
    echo [ERROR] MSBuild failed (exit !BUILD_EC!)
    echo --- Last 60 lines ---
    powershell -NoProfile -Command "Get-Content '%LOGFILE%' | Select-Object -Last 60" 2>nul
    echo Log: %LOGFILE%
    goto :fail
)

if exist "Project2\Project2\cyusbserial.dll" xcopy /Y /Q "Project2\Project2\cyusbserial.dll" "build\%CONFIG%\"

echo.
echo ============================================================
echo   BUILD SUCCEEDED
echo   EXE : build\%CONFIG%\twoProjector.exe
echo   LOG : %LOGFILE%
echo ============================================================
echo [%DATE% %TIME%] Build succeeded >> "%LOGFILE%"
endlocal & exit /b 0

:fail
echo.
echo ============================================================
echo   BUILD FAILED  -  Log: %LOGFILE%
echo ============================================================
echo [%DATE% %TIME%] Build FAILED >> "%LOGFILE%"
endlocal & exit /b 1
"""

RUN_BAT = r"""@echo off
REM ============================================================
REM graduateSoftware Application Launcher
REM   Usage: run.bat [Release|Debug]
REM ============================================================
setlocal

set "PROJ=%~dp0"
pushd "%PROJ%" >nul 2>&1

set "CONFIG=Release"
if /I "%~1"=="Debug" set "CONFIG=Debug"

set "EXE=build\%CONFIG%\twoProjector.exe"

if not exist "%EXE%" (
    echo [INFO] %EXE% not found, building %CONFIG% first...
    call build.bat %CONFIG%
)

if not exist "%EXE%" (
    echo [ERROR] Executable not found after build: %EXE%
    pause
    popd & endlocal & exit /b 1
)

REM ---- DLL search paths ---
set "BUILD_DIR=%PROJ%build\%CONFIG%"
set "ANACONDA_BIN=D:\software\anaconda\Library\bin"
set "OPENCV_BIN=F:\project\Envlib\opencv455\opencv\build\x64\vc15\bin"
set "PCL_BIN=F:\project\Envlib\PCL1.12.1\bin"
set "VTK_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\VTK\bin"
set "BOOST_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\Boost\lib"
set "QHULL_BIN=F:\project\Envlib\PCL1.12.1\3rdParty\Qhull\bin"
set "QT_PLUGIN_PATH=%PROJ%qt_deploy\plugins"

set "PATH=%BUILD_DIR%;%ANACONDA_BIN%;%VTK_BIN%;%OPENCV_BIN%;%PCL_BIN%;%BOOST_BIN%;%QHULL_BIN%;%PATH%"

echo.
echo ============================================================
echo   Running twoProjector [%CONFIG%]
echo   EXE: %PROJ%%EXE%
echo ============================================================
echo.

start "twoProjector" /D "%PROJ%" "%PROJ%%EXE%"
set "EC=%ERRORLEVEL%"

if not "%EC%"=="0" (
    echo [ERROR] Failed to start (exit %EC%)
    pause
)

popd
endlocal & exit /b %EC%
"""

proj = r'f:/project/graduateSoftware'
with open(proj + '/build.bat', 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(BUILD_BAT.lstrip('\n'))
with open(proj + '/run.bat', 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(RUN_BAT.lstrip('\n'))

print('build.bat and run.bat written OK')
