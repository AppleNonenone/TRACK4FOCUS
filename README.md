# TRACK4FOCUS

A face-tracking 4-axis robotic arm control system with a modern web interface.

## Features
- Real-time face detection and tracking using OpenCV
- 4-axis robotic arm control (Arduino)
- Live camera feed and status monitoring via web UI
- Adjustable tracking parameters (sensitivity, smoothing, threshold, arm lengths)
- Command log and error/success notifications
- Camera and Arduino port selection
- Responsive, modern UI (light/dark theme ready)

## Folder Structure
```
track4focus/
├── app.py                  # Main Flask application
├── face_tracker.py         # Face detection/tracking logic
├── arduino_controller.py   # Arduino serial communication
├── kinematics.py           # Inverse kinematics calculations
├── static/                 # Frontend JS, CSS, manifest, service worker
│   ├── script.js
│   ├── style.css
│   ├── manifest.json
│   └── sw.js
├── templates/
│   └── index.html          # Main web interface template
├── README.md               # (You are here)
├── INSTALLATION.md         # Installation & usage guide
├── pyproject.toml          # Python dependencies
├── uv.lock                 # Dependency lock file
```

## Requirements
- Python 3.8+
- pip (Python package manager)
- OpenCV (cv2)
- Flask
- numpy
- (Optional) pyserial (for Arduino)

## Quick Start
1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/track4focus.git
   cd track4focus
   ```
2. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   # or
   pip install flask opencv-python numpy pyserial
   ```
3. **Connect your Arduino and camera**
4. **Run the server**
   ```bash
   python app.py
   ```
5. **Open your browser**
   - Go to `http://localhost:65535` (or the port shown in the terminal)

## Notes
- Camera and Arduino port can be selected in the web UI.
- All tracking parameters are adjustable in real time.
- For troubleshooting, see INSTALLATION.md.

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
MIT License