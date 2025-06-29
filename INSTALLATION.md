# Installation & Usage Guide

## 1. Requirements
- Python 3.8 or higher
- pip (Python package manager)
- OpenCV (cv2)
- Flask
- numpy
- (Optional) pyserial (for Arduino communication)
- USB webcam
- Arduino board (Uno, Nano, etc.)
- 4 servo motors (base, shoulder, elbow, wrist)

## 2. Setup

### Clone the repository
```bash
git clone https://github.com/yourusername/track4focus.git
cd track4focus
```

### Install dependencies
```bash
pip install -r requirements.txt
# or
pip install flask opencv-python numpy pyserial
```

### (Optional) Create a virtual environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

## 3. Hardware Connection
- Connect your USB webcam to the computer.
- Connect your Arduino to the computer via USB.
- Wire the servos to the Arduino as per your hardware setup.

## 4. Running the Application
```bash
python app.py
```
- The server will start on `http://localhost:65535` by default.
- Open this address in your web browser.

## 5. Web Interface Usage
- **Start Preview**: Test the camera feed without tracking.
- **Start Tracking**: Begin face tracking and robotic arm control.
- **Camera/Port Selection**: Use the dropdowns to select the correct camera and Arduino port.
- **Parameter Adjustment**: Tune sensitivity, smoothing, threshold, and arm lengths in real time.
- **Command Log**: View recent commands sent to the Arduino.
- **Laser Auto Control**: Toggle automatic laser on/off logic.

## 6. Troubleshooting
- **Camera not detected?**
  - Try different camera indices (0, 1, 2...)
  - Ensure no other application is using the camera
  - Check camera permissions (macOS: System Preferences > Security & Privacy > Camera)
- **Arduino not detected?**
  - Check the port in the web UI
  - Ensure the correct drivers are installed
  - Try reconnecting the USB cable
- **Face not detected?**
  - Ensure good lighting and clear view of the face
  - Adjust the camera position
- **Performance issues?**
  - Close unnecessary applications
  - Use a wired USB connection for the camera

## 7. FAQ
- **Q: Can I use a different Arduino board?**
  - A: Yes, as long as it supports serial communication and enough PWM pins for servos.
- **Q: Can I run this on Windows/Mac/Linux?**
  - A: Yes, the project is cross-platform (tested on Windows/macOS/Linux).
- **Q: How do I change the default camera or port?**
  - A: Use the dropdowns in the web UI. The selection is applied immediately.

## 8. Uninstallation
- Simply delete the project folder. No system-wide changes are made.

## Easy Install (Recommended)

### Windows
1. Double-click `launch_trackfocus.bat`
2. The launcher will check Python, install all dependencies, and start the server automatically.
3. Your browser will open to: http://localhost:65535

### Mac/Linux
1. Open a terminal in the project folder
2. Run: `chmod +x launch_trackfocus.sh`
3. Run: `./launch_trackfocus.sh`
4. The launcher will check Python, install all dependencies, and start the server automatically.
5. Your browser will open to: http://localhost:65535

### Universal (Any OS)
1. Run: `python install_and_run.py`
2. The script will check Python, install all dependencies, and start the server automatically.
3. Your browser will open to: http://localhost:65535

---
For more details, see README.md or open an issue on GitHub.