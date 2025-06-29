from flask import Flask, render_template, jsonify, Response, request
import cv2
import json
import threading
import time
from face_tracker import FaceTracker
from arduino_controller import ArduinoController
import os
from kinematics import InverseKinematics

app = Flask(__name__)

# Global variables
face_tracker = None
arduino_controller = None
tracking_active = False
tracking_thread = None
preview_active = False
current_status = {
    'tracking': False,
    'preview': False,
    'face_detected': False,
    'servo_angles': [90, 90, 90, 90],
    'laser_state': 0,
    'error': None,
    'camera_index': 0,
    'available_cameras': [],
    'center_threshold': 0,
    'laser_auto_control': True,
    'port': None
}
kinematics = InverseKinematics()  # 앱 시작 시 글로벌로 생성

def initialize_camera_only():
    """Initialize camera for preview mode"""
    global face_tracker, current_status
    try:
        # 기존 face_tracker 해제
        if face_tracker:
            try:
                face_tracker.cleanup()
            except Exception:
                pass
            face_tracker = None
        camera_index = current_status.get('camera_index', 0)
        face_tracker = FaceTracker(camera_index)
        if not face_tracker.is_initialized():
            current_status['error'] = f"Camera {camera_index} initialization failed"
            return False
        face_tracker.set_preview_mode(True)
        current_status['error'] = None
        current_status['available_cameras'] = face_tracker.get_available_cameras()
        return True
    except Exception as e:
        current_status['error'] = f"Camera initialization error: {str(e)}"
        return False

def initialize_systems():
    """Initialize camera and Arduino systems (아두이노 연결 실패해도 진행)"""
    global face_tracker, arduino_controller, current_status
    try:
        # 기존 face_tracker 해제
        if face_tracker:
            try:
                face_tracker.cleanup()
            except Exception:
                pass
            face_tracker = None
        camera_index = current_status.get('camera_index', 0)
        face_tracker = FaceTracker(camera_index)
        if not face_tracker.is_initialized():
            current_status['error'] = f"Camera {camera_index} initialization failed"
            return False
        face_tracker.set_preview_mode(False)  # Enable face detection
        # Initialize Arduino controller (연결 실패해도 객체는 생성)
        serial_port = os.getenv('ARDUINO_PORT', '/dev/ttyUSB0')
        arduino_controller = ArduinoController(serial_port)
        if not arduino_controller.connect():
            current_status['error'] = "Arduino connection failed (명령은 로그로만 송출됩니다)"
        else:
            current_status['error'] = None
            current_status['port'] = serial_port
        current_status['available_cameras'] = face_tracker.get_available_cameras()
        return True
    except Exception as e:
        current_status['error'] = f"System initialization error: {str(e)}"
        return False

def tracking_loop():
    """Main tracking loop running in separate thread"""
    global tracking_active, current_status, face_tracker, arduino_controller
    while tracking_active:
        try:
            if face_tracker and arduino_controller:
                try:
                    frame, face_data = face_tracker.process_frame()
                except Exception as e:
                    current_status['error'] = f"Camera read error: {str(e)}"
                    time.sleep(1)
                    continue
                if face_data:
                    current_status['face_detected'] = True
                    try:
                        # 레이저 자동제어: 체크박스가 꺼져있으면 항상 OFF
                        if not current_status.get('laser_auto_control', True):
                            laser_state = 0
                        else:
                            laser_state = 1
                            center_x = face_tracker.camera_width // 2
                            center_y = face_tracker.camera_height // 2
                            x = face_data['center_x']
                            y = face_data['center_y']
                            w = face_data['width']
                            h = face_data['height']
                            # 십자가가 얼굴 박스 내부에 있으면 OFF
                            if (center_x >= x - w//2 and center_x <= x + w//2 and
                                center_y >= y - h//2 and center_y <= y + h//2):
                                laser_state = 0
                        servo_angles, _ = arduino_controller.calculate_servo_positions(face_data)
                        arduino_controller.send_command(servo_angles, laser_state)
                        current_status['servo_angles'] = servo_angles
                        current_status['laser_state'] = laser_state
                    except Exception as e:
                        current_status['error'] = f"Servo command error: {str(e)}"
                else:
                    current_status['face_detected'] = False
                    neutral_angles = [90, 90, 90, 90]
                    try:
                        arduino_controller.send_command(neutral_angles, 0)
                        current_status['servo_angles'] = neutral_angles
                        current_status['laser_state'] = 0
                    except Exception as e:
                        current_status['error'] = f"Servo command error: {str(e)}"
                current_status['error'] = None
            time.sleep(0.1)
        except Exception as e:
            current_status['error'] = f"Tracking error: {str(e)}"
            time.sleep(1)

@app.route('/')
def index():
    """Main page"""
    return render_template('index.html')

@app.route('/start_preview', methods=['POST'])
def start_preview():
    """Start camera preview without Arduino"""
    global preview_active, current_status
    
    if preview_active:
        return jsonify({'success': False, 'message': 'Preview already active'})
    
    if not initialize_camera_only():
        return jsonify({'success': False, 'message': current_status['error']})
    
    preview_active = True
    current_status['preview'] = True
    
    return jsonify({'success': True, 'message': 'Camera preview started'})

@app.route('/stop_preview', methods=['POST'])
def stop_preview():
    """Stop camera preview"""
    global preview_active, current_status, face_tracker
    
    preview_active = False
    current_status['preview'] = False
    
    if face_tracker:
        face_tracker.cleanup()
        face_tracker = None
    
    return jsonify({'success': True, 'message': 'Camera preview stopped'})

@app.route('/start_tracking', methods=['POST'])
def start_tracking():
    """Start face tracking with or without Arduino"""
    global tracking_active, tracking_thread, current_status, preview_active
    if tracking_active:
        return jsonify({'success': False, 'message': 'Tracking already active'})
    # Stop preview if active
    if preview_active:
        stop_preview()
    # initialize_systems는 아두이노 연결 실패해도 True 반환
    if not initialize_systems():
        return jsonify({'success': False, 'message': current_status['error']})
    tracking_active = True
    current_status['tracking'] = True
    tracking_thread = threading.Thread(target=tracking_loop)
    tracking_thread.daemon = True
    tracking_thread.start()
    msg = 'Face tracking started'
    if current_status['error']:
        msg += f' (주의: {current_status["error"]})'
    return jsonify({'success': True, 'message': msg})

@app.route('/stop_tracking', methods=['POST'])
def stop_tracking():
    """Stop face tracking"""
    global tracking_active, current_status, arduino_controller
    
    tracking_active = False
    current_status['tracking'] = False
    current_status['face_detected'] = False
    
    # Return servos to neutral position
    if arduino_controller:
        neutral_angles = [90, 90, 90, 90]
        arduino_controller.send_command(neutral_angles, 0)
        current_status['servo_angles'] = neutral_angles
        current_status['laser_state'] = 0
    
    return jsonify({'success': True, 'message': 'Face tracking stopped'})

@app.route('/status')
def get_status():
    """Get current system status"""
    global arduino_controller, current_status
    status = current_status.copy()
    if arduino_controller:
        status['tracking_settings'] = arduino_controller.get_status().get('tracking_settings', {})
    else:
        status['tracking_settings'] = {}
    status['laser_auto_control'] = current_status.get('laser_auto_control', True)
    return jsonify(status)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        global face_tracker, tracking_active, preview_active
        while True:
            if face_tracker and (tracking_active or preview_active):
                frame, _ = face_tracker.get_display_frame()
                if frame is not None:
                    ret, buffer = cv2.imencode('.jpg', frame)
                    if ret:
                        frame_bytes = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.1)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/change_camera', methods=['POST'])
def change_camera():
    """Change camera index"""
    global current_status, face_tracker, preview_active, tracking_active
    
    try:
        data = request.get_json()
        camera_index = int(data.get('camera_index', 0))
        current_status['camera_index'] = camera_index
        
        # If preview or tracking is active, reinitialize with new camera
        if preview_active and face_tracker:
            if face_tracker.change_camera(camera_index):
                return jsonify({'success': True, 'message': f'Switched to camera {camera_index}'})
            else:
                return jsonify({'success': False, 'message': f'Failed to switch to camera {camera_index}'})
        elif tracking_active:
            return jsonify({'success': False, 'message': 'Stop tracking first before changing camera'})
        else:
            return jsonify({'success': True, 'message': f'Camera {camera_index} will be used on next start'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': f'Camera change failed: {str(e)}'})

@app.route('/set_parameters', methods=['POST'])
def set_parameters():
    """Set robotic arm parameters"""
    try:
        global kinematics
        data = request.get_json()
        params = {}
        if 'movement_sensitivity' in data:
            params['movement_sensitivity'] = float(data['movement_sensitivity'])
        if 'smoothing_factor' in data:
            params['smoothing_factor'] = float(data['smoothing_factor'])
        if 'center_threshold' in data:
            current_status['center_threshold'] = int(data['center_threshold'])
        if 'L1' in data and 'L2' in data:
            kinematics.update_link_lengths(float(data['L1']), float(data['L2']))
        if arduino_controller:
            arduino_controller.update_parameters(params)
        return jsonify({'success': True, 'message': 'Parameters updated'})
    except Exception as e:
        return jsonify({'success': False, 'message': f'Parameter update failed: {str(e)}'})

@app.route('/arduino_ports')
def arduino_ports():
    ports = ArduinoController.list_serial_ports()
    return jsonify({'ports': ports})

@app.route('/set_arduino_port', methods=['POST'])
def set_arduino_port():
    global arduino_controller, current_status
    data = request.get_json()
    port = data.get('port')
    if not port:
        return jsonify({'success': False, 'message': '포트가 지정되지 않았습니다.'})
    try:
        arduino_controller = ArduinoController(port)
        if arduino_controller.connect():
            current_status['error'] = None
            current_status['port'] = port
            return jsonify({'success': True, 'message': f'{port} 포트로 연결 성공'})
        else:
            return jsonify({'success': False, 'message': f'{port} 포트로 연결 실패'})
    except Exception as e:
        return jsonify({'success': False, 'message': f'포트 연결 오류: {str(e)}'})

@app.route('/command_log')
def command_log():
    global arduino_controller
    if arduino_controller:
        log = arduino_controller.get_command_log(30)
    else:
        log = []
    return jsonify({'log': log})

@app.route('/set_laser_auto_control', methods=['POST'])
def set_laser_auto_control():
    global current_status
    data = request.get_json()
    auto = data.get('auto', True)
    current_status['laser_auto_control'] = bool(auto)
    return jsonify({'success': True, 'auto': current_status['laser_auto_control']})

def main():
    """Main entry point for the application"""
    app.run(host='0.0.0.0', port=65535, debug=False)

if __name__ == '__main__':
    main()
