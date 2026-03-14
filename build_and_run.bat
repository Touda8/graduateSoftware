@echo off
REM ============================================================================
REM One-Touch Build and Run Script
REM Usage: build_and_run.bat [Release|Debug]
REM ============================================================================

setlocal enabledelayedexpansion

set CONFIG=%~1
if "%CONFIG%"=="" set CONFIG=Release

echo.
echo ============================================================================
echo              One-Touch Build and Run - graduateSoftware
echo ============================================================================
echo.
echo Configuration: %CONFIG%
echo.

REM ========== STEP 1: BUILD ==========
echo [Step 1/2] Building project...
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.

call build.bat %CONFIG%

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ============================================================================
    echo BUILD FAILED - Cannot proceed to run
    echo ============================================================================
    echo.
    echo Troubleshooting:
    echo   1. Check the build log in logs\ directory
    echo   2. Run check_build.bat for diagnostics
    echo   3. Ensure all dependencies are installed
    echo.
    pause
    exit /b 1
)

echo.
echo [Step 1/2] ✓ Build completed successfully
echo.

REM ========== STEP 2: RUN ==========
echo.
echo [Step 2/2] Launching application...
echo ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
echo.

if "%CONFIG%"=="Debug" (
    call run.bat Debug
) else (
    call run.bat
)

set RUN_RESULT=%ERRORLEVEL%

echo.
echo ============================================================================
echo Application execution completed (exit code: %RUN_RESULT%)
echo ============================================================================
echo.

endlocal
exit /b %RUN_RESULT%
