@echo off
title TRACK4FOCUS Launcher
color 0b

echo ====================================
echo TRACK4FOCUS - System Launcher
echo ====================================
echo Target-Responsive Actuating using
echo Calibrated Kinematics for 
echo Face-Oriented Controlled Unified System
echo ====================================
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7 or newer from python.org
    echo.
    pause
    exit /b 1
)

echo Starting TRACK4FOCUS...
echo.
echo The web interface will open at: http://localhost:65535
echo Press Ctrl+C to stop the application
echo.

REM Run the launcher
python install_and_run.py

pause