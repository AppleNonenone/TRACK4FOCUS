#!/usr/bin/env python3
"""
TRACK4FOCUS Installation and Launcher
Target-Responsive Actuating using Calibrated Kinematics for Face-Oriented Controlled Unified System
"""

import os
import sys
import subprocess
import webbrowser
import time
import threading

def check_python_version():
    """Check if Python version is compatible"""
    if sys.version_info < (3, 7):
        print("Error: Python 3.7 or higher is required")
        print(f"Current version: {sys.version}")
        return False
    print(f"Python version: {sys.version.split()[0]} ✓")
    return True

def install_requirements():
    """Install required packages"""
    packages = [
        "flask>=3.0.0",
        "opencv-python>=4.5.0", 
        "pyserial>=3.4",
        "numpy>=1.20.0"
    ]
    
    print("Installing required packages...")
    for package in packages:
        try:
            print(f"Installing {package}...")
            subprocess.check_call([
                sys.executable, "-m", "pip", "install", package
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"✓ {package}")
        except subprocess.CalledProcessError:
            print(f"✗ Failed to install {package}")
            return False
    
    print("All packages installed successfully!")
    return True

def open_browser():
    """Open browser after a delay"""
    time.sleep(3)
    webbrowser.open('http://localhost:65535')

def main():
    """Main launcher function"""
    print("=" * 60)
    print("TRACK4FOCUS - System Launcher")
    print("Target-Responsive Actuating using Calibrated Kinematics")
    print("for Face-Oriented Controlled Unified System")
    print("=" * 60)
    print()
    
    # Check Python version
    if not check_python_version():
        input("Press Enter to exit...")
        return
    
    # Install requirements
    print("Checking dependencies...")
    if not install_requirements():
        print("Failed to install dependencies!")
        input("Press Enter to exit...")
        return
    
    print()
    print("Starting TRACK4FOCUS server...")
    print("The web interface will open automatically in your browser.")
    print("If it doesn't open, go to: http://localhost:65535")
    print()
    print("Press Ctrl+C to stop the server")
    print("-" * 60)
    
    # Start browser in background
    browser_thread = threading.Thread(target=open_browser)
    browser_thread.daemon = True
    browser_thread.start()
    
    # Import and run the app
    try:
        from app import main as app_main
        app_main()
    except KeyboardInterrupt:
        print("\nShutting down TRACK4FOCUS...")
    except ImportError as e:
        print(f"Error importing app: {e}")
        print("Make sure all files are in the same directory")
    except Exception as e:
        print(f"Error starting application: {e}")
    
    input("Press Enter to exit...")

if __name__ == "__main__":
    main()