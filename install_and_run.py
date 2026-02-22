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

    # Import and run the app (안전하게)
    try:
        import sys
        import signal
        
        # Segmentation fault 감지를 위한 signal handler
        def signal_handler(sig, frame):
            print("\n\nShutting down TRACK4FOCUS (signal received).")
            print("If this was unexpected, check logs for errors or OpenCV/camera issues.")
            sys.exit(0)
        
        # SIGSEGV는 macOS에서 사용할 수 없을 수 있으므로 SIGINT만 처리
        signal.signal(signal.SIGINT, signal_handler)
        
        print("Loading modules...")
        # 모듈을 단계별로 임포트하여 어디서 문제가 발생하는지 확인
        # OpenCV는 app.py에서 안전하게 임포트하므로 여기서는 임포트하지 않음
        try:
            print("  - Importing Flask...")
            from flask import Flask
            print("  - Flask imported successfully")
            print("  - Importing app module (OpenCV will be imported inside app.py)...")
            print("    (OpenCV는 app.py 내부에서 안전하게 임포트됩니다)")
            # app.py 모듈 임포트 - OpenCV는 app.py 내부에서 안전하게 임포트됨
            from app import main as app_main
            print("  - All modules loaded successfully!")
            print()
            app_main()
        except Exception as import_error:
            print(f"\nError during import: {import_error}")
            import traceback
            traceback.print_exc()
            print("\nTroubleshooting:")
            print("1. Make sure all required packages are installed")
            print("2. Try: pip3 install --upgrade opencv-python flask pyserial numpy")
            print("3. If segmentation fault occurs, try:")
            print("   - Close all other camera applications")
            print("   - Reinstall OpenCV: pip3 uninstall opencv-python && pip3 install opencv-python")
            print("   - Try different Python version: python3.9 or python3.10")
            raise
            
    except KeyboardInterrupt:
        print("\nShutting down TRACK4FOCUS...")
    except ImportError as e:
        print(f"Error importing app: {e}")
        print("Make sure all files are in the same directory")
        import traceback
        traceback.print_exc()
    except Exception as e:
        print(f"Error starting application: {e}")
        import traceback
        traceback.print_exc()
    
    input("\nPress Enter to exit...")

if __name__ == "__main__":
    main()