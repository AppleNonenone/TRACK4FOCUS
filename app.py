from flask import Flask, render_template, jsonify, Response, request
import json
import threading
import time
import os

# OpenCV를 안전하게 임포트 (segmentation fault 방지)
# 지연 로딩으로 변경하여 segmentation fault 가능성 감소
CV2_AVAILABLE = False
cv2 = None

def _import_opencv():
    """OpenCV를 지연 로딩 (필요할 때만 임포트)"""
    global CV2_AVAILABLE, cv2
    if CV2_AVAILABLE:
        return True
    
    try:
        import cv2
        # 간단한 테스트로 OpenCV가 제대로 로드되었는지 확인
        # cv2.__version__ 접근은 안전하지만, 실제 사용은 나중에
        _ = cv2.__version__
        CV2_AVAILABLE = True
        print("OpenCV imported successfully")
        return True
    except Exception as e:
        print(f"Warning: OpenCV import failed: {e}")
        CV2_AVAILABLE = False
        cv2 = None
        return False

# OpenCV는 필요할 때만 임포트 (지연 로딩)
# 앱 시작 시에는 임포트하지 않음 - 카메라 사용 시점에만 임포트

# 다른 모듈들을 안전하게 임포트 (지연 로딩)
# face_tracker는 OpenCV를 사용하므로 필요할 때만 임포트
FACE_TRACKER_AVAILABLE = False
FaceTracker = None

def _import_face_tracker():
    """FaceTracker를 지연 로딩"""
    global FACE_TRACKER_AVAILABLE, FaceTracker
    if FACE_TRACKER_AVAILABLE:
        return True
    
    # OpenCV가 먼저 필요
    if not CV2_AVAILABLE:
        _import_opencv()
    
    if not CV2_AVAILABLE:
        return False
    
    try:
        from face_tracker import FaceTracker
        FACE_TRACKER_AVAILABLE = True
        return True
    except Exception as e:
        print(f"Warning: FaceTracker import failed: {e}")
        FACE_TRACKER_AVAILABLE = False
        FaceTracker = None
        return False

try:
    from arduino_controller import ArduinoController
    ARDUINO_CONTROLLER_AVAILABLE = True
except Exception as e:
    print(f"Warning: ArduinoController import failed: {e}")
    ARDUINO_CONTROLLER_AVAILABLE = False
    ArduinoController = None

try:
    from kinematics import InverseKinematics
    KINEMATICS_AVAILABLE = True
except Exception as e:
    print(f"Warning: InverseKinematics import failed: {e}")
    KINEMATICS_AVAILABLE = False
    InverseKinematics = None

app = Flask(__name__)

# Global variables
face_tracker = None
arduino_controller = None
tracking_active = False
tracking_thread = None
preview_active = False
heartbeat_thread = None
heartbeat_active = False
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
# kinematics는 필요할 때만 생성 (lazy initialization)
kinematics = None

def initialize_camera_only():
    """Initialize camera for preview mode"""
    global face_tracker, current_status, CV2_AVAILABLE, cv2, FACE_TRACKER_AVAILABLE, FaceTracker
    
    # OpenCV와 FaceTracker가 아직 임포트되지 않았으면 시도
    if not CV2_AVAILABLE:
        _import_opencv()
    
    if not FACE_TRACKER_AVAILABLE:
        _import_face_tracker()
    
    if not CV2_AVAILABLE or not FACE_TRACKER_AVAILABLE:
        current_status['error'] = "OpenCV or FaceTracker not available"
        return False
    try:
        # 기존 face_tracker 해제
        if face_tracker:
            try:
                face_tracker.cleanup()
            except Exception:
                pass
            face_tracker = None
        camera_index = current_status.get('camera_index', 0)
        # cv2 모듈을 전달하여 임포트 문제 방지
        face_tracker = FaceTracker(camera_index, cv2_module=cv2)
        # Allow preview mode even if face cascade failed to load; only camera required
        camera_ready = getattr(face_tracker, 'cap', None) is not None and getattr(face_tracker, 'cap', None).isOpened()
        if not camera_ready:
            current_status['error'] = f"Camera {camera_index} initialization failed"
            try:
                face_tracker.cleanup()
            except Exception:
                pass
            face_tracker = None
            return False
        # If camera ready but cascade missing, still allow preview
        face_tracker.set_preview_mode(True)
        current_status['error'] = None
        # 안전하게 카메라 목록 가져오기
        try:
            current_status['available_cameras'] = face_tracker.get_available_cameras()
        except Exception as e:
            print(f"Warning: Could not get available cameras: {e}")
            current_status['available_cameras'] = [0]  # 기본값
        return True
    except Exception as e:
        current_status['error'] = f"Camera initialization error: {str(e)}"
        import traceback
        traceback.print_exc()
        return False

def initialize_systems():
    """Initialize camera and Arduino systems (아두이노 연결 실패해도 진행)"""
    global face_tracker, arduino_controller, current_status, CV2_AVAILABLE, cv2, FACE_TRACKER_AVAILABLE, FaceTracker
    
    # OpenCV와 FaceTracker가 아직 임포트되지 않았으면 시도
    if not CV2_AVAILABLE:
        _import_opencv()
    
    if not FACE_TRACKER_AVAILABLE:
        _import_face_tracker()
    
    if not CV2_AVAILABLE or not FACE_TRACKER_AVAILABLE:
        current_status['error'] = "OpenCV or FaceTracker not available"
        return False
    try:
        # 기존 face_tracker 해제
        if face_tracker:
            try:
                face_tracker.cleanup()
            except Exception:
                pass
            face_tracker = None
        camera_index = current_status.get('camera_index', 0)
        # cv2 모듈을 전달하여 임포트 문제 방지
        face_tracker = FaceTracker(camera_index, cv2_module=cv2)
        # For full tracking we require both camera and cascade classifier
        if not face_tracker.is_initialized():
            current_status['error'] = f"Camera {camera_index} initialization failed or face detector unavailable"
            try:
                face_tracker.cleanup()
            except Exception:
                pass
            face_tracker = None
            return False
        face_tracker.set_preview_mode(False)  # Enable face detection
        
        # 기존 아두이노 연결 확인 및 재사용
        # 우선순위: 1. UI에서 선택한 포트 (current_status['port']) 2. 환경 변수 3. 자동 감지
        serial_port = current_status.get('port')  # UI에서 선택한 포트가 최우선
        if not serial_port:
            # 환경 변수 확인
            serial_port = os.getenv('ARDUINO_PORT')
        if not serial_port:
            # 자동으로 사용 가능한 포트 찾기
            try:
                available_ports = ArduinoController.list_serial_ports()
                if available_ports:
                    serial_port = available_ports[0]
                    print(f"자동 감지된 Arduino 포트: {serial_port}")
                    # 자동 감지된 포트를 current_status에 저장
                    current_status['port'] = serial_port
                else:
                    print("사용 가능한 Arduino 포트가 없습니다. UI에서 포트를 선택하세요.")
                    serial_port = None
            except Exception as e:
                print(f"포트 자동 감지 실패: {e}")
                serial_port = None
        else:
            # UI에서 선택한 포트나 환경 변수 포트를 사용
            print(f"사용할 Arduino 포트: {serial_port}")
        
        # 포트가 없으면 아두이노 연결 시도하지 않음
        if not serial_port:
            print("Arduino 포트가 지정되지 않았습니다. UI에서 포트를 선택하거나 환경 변수 ARDUINO_PORT를 설정하세요.")
            current_status['error'] = "Arduino port not specified (UI에서 포트를 선택하세요)"
            current_status['available_cameras'] = face_tracker.get_available_cameras()
            return True  # 카메라는 정상이므로 True 반환
        
        # 기존 연결이 있고 같은 포트를 사용 중이면 재사용
        if arduino_controller and arduino_controller.is_connected():
            if arduino_controller.port == serial_port:
                print(f"Arduino 재사용: {serial_port}")
                current_status['error'] = None
                current_status['port'] = serial_port
                current_status['available_cameras'] = face_tracker.get_available_cameras()
                return True
        
        # 기존 연결이 있으면 닫기 (다른 포트이거나 연결이 끊어진 경우)
        if arduino_controller:
            try:
                # 연결이 열려있을 때만 닫기
                if arduino_controller.serial_conn and arduino_controller.serial_conn.is_open:
                    arduino_controller.close()
            except:
                pass
        
        # Initialize Arduino controller (연결 실패해도 객체는 생성)
        try:
            arduino_controller = ArduinoController(serial_port)
            if arduino_controller.connect():
                current_status['error'] = None
                current_status['port'] = serial_port
                print(f"Arduino 연결 성공: {serial_port}")
            else:
                # 연결 실패했지만 포트가 열려있으면 연결된 것으로 간주
                if arduino_controller.serial_conn and arduino_controller.serial_conn.is_open:
                    arduino_controller.connected = True
                    current_status['error'] = None
                    current_status['port'] = serial_port
                    print(f"Arduino 연결됨 (응답 없음): {serial_port}")
                else:
                    current_status['error'] = f"Arduino connection failed on {serial_port} (명령은 로그로만 송출됩니다)"
                    print(f"Arduino 연결 실패: {serial_port}")
        except Exception as e:
            print(f"Arduino 초기화 오류: {e}")
            import traceback
            traceback.print_exc()
            current_status['error'] = f"Arduino initialization error: {str(e)}"
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
            if not face_tracker:
                time.sleep(0.5)
                continue
                
            try:
                frame, face_data = face_tracker.process_frame()
            except Exception as e:
                current_status['error'] = f"Camera read error: {str(e)}"
                time.sleep(1)
                continue
            
            # 얼굴 인식 상태 업데이트 (아두이노와 무관)
            if face_data:
                current_status['face_detected'] = True
                
                # 서보 각도 계산 (아두이노 연결 여부와 무관)
                if arduino_controller:
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
                        # 명령 전송 (연결되지 않아도 로그에 기록됨)
                        arduino_controller.send_command(servo_angles, laser_state)
                        current_status['servo_angles'] = servo_angles
                        current_status['laser_state'] = laser_state
                    except Exception as e:
                        current_status['error'] = f"Servo command error: {str(e)}"
                        print(f"Arduino command error: {e}")
            else:
                current_status['face_detected'] = False
                
                # 얼굴이 없을 때 중립 위치로 이동
                if arduino_controller:
                    neutral_angles = [90, 90, 90, 90]
                    try:
                        # 명령 전송 (연결되지 않아도 로그에 기록됨)
                        arduino_controller.send_command(neutral_angles, 0)
                        current_status['servo_angles'] = neutral_angles
                        current_status['laser_state'] = 0
                    except Exception as e:
                        current_status['error'] = f"Servo command error: {str(e)}"
                        print(f"Arduino neutral position error: {e}")
            
            # 에러가 없으면 에러 상태 초기화
            if not current_status.get('error'):
                pass  # 에러가 있으면 유지
            
            time.sleep(0.1)
        except Exception as e:
            current_status['error'] = f"Tracking error: {str(e)}"
            print(f"Tracking loop error: {e}")
            import traceback
            traceback.print_exc()
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
        global face_tracker, tracking_active, preview_active, CV2_AVAILABLE, cv2
        
        # OpenCV가 없으면 시도
        if not CV2_AVAILABLE:
            _import_opencv()
        
        if not CV2_AVAILABLE:
            # OpenCV가 없으면 빈 응답 반환
            while True:
                time.sleep(1)
                yield b''
        
        while True:
            try:
                if face_tracker and (tracking_active or preview_active):
                    if face_tracker.is_initialized():
                        frame, _ = face_tracker.get_display_frame()
                        if frame is not None:
                            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                            if ret:
                                frame_bytes = buffer.tobytes()
                                yield (b'--frame\r\n'
                                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                            else:
                                # 인코딩 실패 시 약간 대기
                                time.sleep(0.2)
                        else:
                            time.sleep(0.2)
                    else:
                        # 카메라가 초기화되지 않았으면 대기
                        time.sleep(0.5)
                else:
                    # face_tracker가 없거나 tracking/preview가 비활성화된 경우
                    time.sleep(0.5)
            except Exception as e:
                print(f"Video feed error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1)
    
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
            global kinematics
            if kinematics is None:
                kinematics = InverseKinematics()
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
    
    # 포트를 current_status에 저장 (UI에서 선택한 포트)
    current_status['port'] = port
    print(f"UI에서 선택한 Arduino 포트: {port}")
    
    # 기존 연결 확인 및 재사용 (같은 포트면)
    if arduino_controller and arduino_controller.is_connected():
        if arduino_controller.port == port:
            current_status['error'] = None
            print(f"{port} 포트는 이미 연결되어 있습니다")
            return jsonify({'success': True, 'message': f'{port} 포트는 이미 연결되어 있습니다'})
    
    # 기존 연결이 있으면 닫기 (다른 포트이거나 연결이 끊어진 경우)
    if arduino_controller:
        try:
            # 연결이 열려있을 때만 닫기
            if arduino_controller.serial_conn and arduino_controller.serial_conn.is_open:
                arduino_controller.close()
        except:
            pass
    
    try:
        arduino_controller = ArduinoController(port)
        if arduino_controller.connect():
            current_status['error'] = None
            current_status['port'] = port
            print(f"UI에서 선택한 포트로 연결 성공: {port}")
            return jsonify({'success': True, 'message': f'{port} 포트로 연결 성공'})
        else:
            # 연결 실패했지만 포트가 열려있으면 연결된 것으로 간주
            if arduino_controller.serial_conn and arduino_controller.serial_conn.is_open:
                arduino_controller.connected = True
                current_status['error'] = None
                current_status['port'] = port
                print(f"UI에서 선택한 포트로 연결됨 (응답 없음): {port}")
                return jsonify({'success': True, 'message': f'{port} 포트로 연결됨 (응답 없음)'})
            else:
                # 연결 실패 시 더 자세한 정보 제공
                available_ports = ArduinoController.list_serial_ports()
                error_msg = f'{port} 포트로 연결 실패'
                if available_ports:
                    error_msg += f'. 사용 가능한 포트: {", ".join(available_ports)}'
                print(f"포트 연결 실패: {error_msg}")
                return jsonify({'success': False, 'message': error_msg})
    except Exception as e:
        import traceback
        error_detail = str(e)
        traceback.print_exc()
        print(f"포트 연결 오류: {error_detail}")
        return jsonify({'success': False, 'message': f'포트 연결 오류: {error_detail}'})

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

def heartbeat_loop():
    """Heartbeat loop to continuously check Arduino connection"""
    global heartbeat_active, arduino_controller
    while heartbeat_active:
        try:
            if arduino_controller:
                arduino_controller.heartbeat()
            time.sleep(1)  # 1초마다 체크
        except Exception as e:
            print(f"Heartbeat error: {e}")
            time.sleep(5)

def main():
    """Main entry point for the application"""
    global heartbeat_thread, heartbeat_active
    
    # Start heartbeat thread
    heartbeat_active = True
    heartbeat_thread = threading.Thread(target=heartbeat_loop)
    heartbeat_thread.daemon = True
    heartbeat_thread.start()
    print("Arduino heartbeat 모니터링 시작...")
    
    try:
        app.run(host='0.0.0.0', port=65535, debug=False)
    finally:
        heartbeat_active = False
        if heartbeat_thread:
            heartbeat_thread.join(timeout=1)

if __name__ == '__main__':
    main()
