#!/bin/bash

# TRACK4FOCUS Launcher Script
# Target-Responsive Actuating using Calibrated Kinematics for Face-Oriented Controlled Unified System

clear
echo "===================================="
echo "TRACK4FOCUS - System Launcher"
echo "===================================="
echo "Target-Responsive Actuating using"
echo "Calibrated Kinematics for"
echo "Face-Oriented Controlled Unified System"
echo "===================================="
echo

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    if ! command -v python &> /dev/null; then
        echo "ERROR: Python is not installed"
        echo "Please install Python 3.7 or newer"
        echo
        read -p "Press Enter to exit..."
        exit 1
    else
        PYTHON_CMD="python"
    fi
else
    PYTHON_CMD="python3"
fi

echo "Starting TRACK4FOCUS..."
echo
echo "The web interface will open at: http://localhost:65535"
echo "Press Ctrl+C to stop the application"
echo

# Run the launcher
$PYTHON_CMD install_and_run.py

read -p "Press Enter to exit..."