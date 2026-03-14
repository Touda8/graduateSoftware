@echo off
REM ============================================================================
REM Debug Script - Check project status before building
REM ============================================================================

echo.
echo ============================================================================
echo                    graduateSoftware Build Diagnostics
echo ============================================================================
echo.

echo [1] Checking required source files...
if exist "twoProjector.sln" (
    echo   ✓ twoProjector.sln found
) else (
    echo   ✗ ERROR: twoProjector.sln not found!
)

if exist "twoProjector.vcxproj" (
    echo   ✓ twoProjector.vcxproj found
) else (
    echo   ✗ ERROR: twoProjector.vcxproj not found!
)

if exist "twoProjector.ui" (
    echo   ✓ twoProjector.ui found
) else (
    echo   ✗ ERROR: twoProjector.ui not found!
)

echo.
echo [2] Checking generated files...
if exist "ui_twoProjector.h" (
    echo   ✓ ui_twoProjector.h found (%~z0 bytes)
) else (
    echo   ✗ MISSING: ui_twoProjector.h
    echo   Run: python3 generate_ui.py
)

if exist "src\moc_MainWindow.cpp" (
    echo   ✓ src\moc_MainWindow.cpp found
) else (
    echo   ✗ MISSING: src\moc_MainWindow.cpp (critical!)
)

if exist "src\visualization\moc_VtkWidget.cpp" (
    echo   ✓ src\visualization\moc_VtkWidget.cpp found
) else (
    echo   ✗ MISSING: src\visualization\moc_VtkWidget.cpp (critical!)
)

echo.
echo [3] Checking source directories...
for /d %%D in (src\*) do (
    echo   ✓ %%~nxD
)

echo.
echo [4] Checking library dependencies...
if exist "F:\project\Envlib\opencv455\opencv" (
    echo   ✓ OpenCV 4.5.5 found
) else (
    echo   ✗ OpenCV not found at F:\project\Envlib\opencv455\opencv
)

if exist "F:\project\Envlib\PCL1.12.1" (
    echo   ✓ PCL 1.12.1 found
) else (
    echo   ✗ PCL not found at F:\project\Envlib\PCL1.12.1
)

if exist "F:\project\Envlib\eigen-3.4.0" (
    echo   ✓ Eigen 3.4.0 found
) else (
    echo   ✗ Eigen not found at F:\project\Envlib\eigen-3.4.0
)

if exist "qt_deploy" (
    echo   ✓ Qt 5 deployment found
) else (
    echo   ✗ Qt deployment not found
)

echo.
echo [5] Checking Visual Studio 2019...
set VS_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019
if exist "%VS_PATH%\Community" (
    echo   ✓ Visual Studio 2019 Community found
) else if exist "%VS_PATH%\Professional" (
    echo   ✓ Visual Studio 2019 Professional found
) else if exist "%VS_PATH%\Enterprise" (
    echo   ✓ Visual Studio 2019 Enterprise found
) else (
    echo   ✗ Visual Studio 2019 not found!
    echo     Expected at: %VS_PATH%
)

echo.
echo ============================================================================
echo Next steps:
echo   1. build.bat Release       - Compile Release build
echo   2. run.bat                 - Run the application
echo.
echo For Debug build:
echo   1. build.bat Debug         - Compile Debug build
echo   2. run.bat Debug           - Run Debug executable
echo ============================================================================
echo.
