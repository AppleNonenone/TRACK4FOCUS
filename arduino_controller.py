import time
import math
import os

# Note: import pyserial lazily in methods to avoid import-time failures
class ArduinoController:
    def __init__(self, port=None, baudrate=9600):
        """
        Initialize Arduino controller for 4-axis robotic arm face tracking
        
        Robot Configuration:
        - Base: Horizontal rotation (left/right) - azimuth control
        - Shoulder: Vertical rotation (up/down) - elevation control  
        - Elbow: Reach extension (up/down) - distance control
        - Wrist: Fine positioning (up/down) - final adjustment
        
        Args:
            port: Serial port for Arduino communication
            baudrate: Communication speed
        """
        # Auto-detect port if not provided
        if port is None:
            port = self._detect_arduino_port()
        
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.connected = False
        
        # Camera parameters
        self.camera_width = 640
        self.camera_height = 480
        self.camera_fov_h = 60  # degrees horizontal field of view
        self.camera_fov_v = 45  # degrees vertical field of view
        
        # 4-Axis Robot Configuration
        self.servo_limits = {
            'base': (0, 180),      # Base: 0=full left, 90=center, 180=full right
            'shoulder': (30, 150), # Shoulder: 30=down, 90=level, 150=up
            'elbow': (45, 135),    # Elbow: 45=retracted, 90=normal, 135=extended
            'wrist': (45, 135)     # Wrist: 45=down, 90=level, 135=up
        }
        
        # Home/neutral positions
        self.home_positions = {
            'base': 90,      # Center position
            'shoulder': 90,  # Level position
            'elbow': 90,     # Normal reach
            'wrist': 90,     # Level position
            'laser': 0       # Laser off
        }
        
        # Current servo positions
        self.current_positions = self.home_positions.copy()
        
        # Movement sensitivity and smoothing
        self.movement_sensitivity = 1.0  # Multiplier for face tracking responsiveness
        self.smoothing_factor = 0.3      # 0.1=very smooth, 1.0=no smoothing
        self.last_targets = None         # For smoothing calculations
        self.command_log = []  # 송출 명령 로그
        self.last_heartbeat_time = 0  # 마지막 heartbeat 시간
        self.heartbeat_interval = 5.0  # 5초마다 heartbeat
    
    def _detect_arduino_port(self):
        """Auto-detect Arduino serial port"""
        # Default Arduino port based on environment variable
        arduino_port = os.getenv('ARDUINO_PORT')
        if arduino_port:
            return arduino_port
        
        # Use list_serial_ports to find available ports (safe import)
        try:
            available_ports = self.list_serial_ports()
            if available_ports:
                # Return the first available port
                return available_ports[0]
        except Exception as e:
            print(f"Error listing serial ports: {e}")
        
        # Fallback to common ports if list_serial_ports fails
        import platform
        system = platform.system()
        
        if system == "Windows":
            # Try COM ports 1-20
            for i in range(1, 21):
                try:
                    import serial
                    test_serial = serial.Serial(f'COM{i}', self.baudrate, timeout=0.1)
                    test_serial.close()
                    return f'COM{i}'
                except:
                    continue
            return 'COM3'  # Default fallback
        elif system == "Darwin":  # macOS
            # Try common Mac ports
            import glob
            mac_patterns = ['/dev/cu.usbmodem*', '/dev/cu.usbserial*', '/dev/tty.usbmodem*', '/dev/tty.usbserial*']
            for pattern in mac_patterns:
                ports = glob.glob(pattern)
                for port in ports:
                    try:
                        import serial
                        test_serial = serial.Serial(port, self.baudrate, timeout=0.1)
                        test_serial.close()
                        return port
                    except:
                        continue
            return '/dev/cu.usbmodem1411'  # Default fallback for Mac
        else:
            # Linux
            common_ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1', '/dev/ttyACM1']
            for port in common_ports:
                try:
                    test_serial = serial.Serial(port, self.baudrate, timeout=0.1)
                    test_serial.close()
                    return port
                except:
                    continue
            return '/dev/ttyUSB0'  # Default fallback
    
    def connect(self):
        """Establish serial connection with Arduino"""
        # Close existing connection if any
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except:
                pass
        
        # Check if port exists (for Mac/Linux)
        import platform
        if platform.system() != "Windows":
            if not os.path.exists(self.port):
                error_msg = f"Port {self.port} does not exist. Available ports: {', '.join(self.list_serial_ports())}"
                print(f"Failed to connect: {error_msg}")
                self.connected = False
                return False
        
        try:
            # Lazy import of serial to allow module import without pyserial installed
            try:
                import serial
            except Exception as e:
                print(f"pyserial not available: {e}")
                self.connected = False
                self.serial_conn = None
                return False
            # Try to open serial connection
            self.serial_conn = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=1,
                write_timeout=1
            )
            
            # Arduino가 리셋될 시간을 주기 위해 DTR을 조작
            self.serial_conn.setDTR(False)
            time.sleep(0.1)
            self.serial_conn.setDTR(True)
            time.sleep(0.1)
            
            # Wait for Arduino to reset and initialize
            time.sleep(2)
            
            # Arduino가 보낸 초기 메시지 읽기 (버퍼 비우기)
            try:
                while self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"Arduino: {line}")
            except:
                pass
            
            # Clear any residual data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # Verify connection
            if not self.serial_conn.is_open:
                print(f"Failed to connect: Port {self.port} opened but not ready")
                self.connected = False
                return False
            
            # 연결 테스트: 홈 위치 명령을 보내고 응답 확인
            self.connected = False  # 일단 False로 설정 후 테스트
            
            try:
                # 간단한 테스트 명령 전송 (홈 위치)
                test_command = "90,90,90,90,0\n"
                self.serial_conn.write(test_command.encode())
                self.serial_conn.flush()
                
                # 응답 대기 (최대 1초)
                time.sleep(0.5)
                response_received = False
                start_time = time.time()
                while time.time() - start_time < 1.0:
                    if self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                            if line and ("OK:" in line or "Ready" in line or len(line) > 0):
                                response_received = True
                                print(f"Arduino response: {line}")
                                break
                        except:
                            pass
                    time.sleep(0.1)
                
                # 응답이 없어도 연결은 성공으로 간주 (일부 아두이노는 응답을 안 보낼 수 있음)
                # 하지만 실제로 명령이 전송되었는지는 확인
                self.connected = True
                print(f"✓ Connected to Arduino on {self.port} at {self.baudrate} baud")
                
                # 버퍼 정리
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                
                # Send home position on connect
                try:
                    time.sleep(0.3)  # 추가 대기
                    self._send_home_position()
                except Exception as e:
                    print(f"Warning: Could not send home position: {e}")
                
                return True
                
            except Exception as e:
                print(f"Connection test failed: {e}")
                # 연결 자체는 성공했을 수 있으므로 True 반환
                self.connected = True
                print(f"✓ Connected to Arduino on {self.port} (response test failed but connection open)")
                return True
            
        except Exception as e:
            error_msg = f"Serial port error: {str(e)}"
            if "Permission denied" in str(e) or "Access denied" in str(e):
                error_msg += f"\n포트 {self.port}에 접근 권한이 없습니다. "
                if platform.system() == "Linux":
                    error_msg += "sudo를 사용하거나 사용자를 dialout 그룹에 추가하세요: sudo usermod -a -G dialout $USER"
                elif platform.system() == "Darwin":
                    error_msg += "시스템 설정에서 시리얼 포트 접근 권한을 확인하세요."
            elif "could not open port" in str(e).lower():
                available = self.list_serial_ports()
                if available:
                    error_msg += f"\n사용 가능한 포트: {', '.join(available)}"
                else:
                    error_msg += "\n사용 가능한 시리얼 포트가 없습니다."
            print(f"Failed to connect to Arduino on {self.port}: {error_msg}")
            self.connected = False
            self.serial_conn = None
            return False
        except Exception as e:
            error_msg = f"Unexpected error: {str(e)}"
            print(f"Failed to connect to Arduino on {self.port}: {error_msg}")
            import traceback
            traceback.print_exc()
            self.connected = False
            self.serial_conn = None
            return False
    
    def is_connected(self):
        """Check if Arduino is connected"""
        return self.connected and self.serial_conn and self.serial_conn.is_open
    
    def check_connection(self):
        """
        Heartbeat를 통해 아두이노 연결 상태 확인
        Returns:
            bool: 연결이 활성 상태면 True, 아니면 False
        """
        if not self.is_connected():
            return False
        
        try:
            # PING 명령 전송
            self.serial_conn.write(b"PING\n")
            self.serial_conn.flush()
            
            # 응답 대기 (최대 0.5초)
            start_time = time.time()
            while time.time() - start_time < 0.5:
                if self.serial_conn.in_waiting > 0:
                    try:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if "PONG" in line.upper():
                            return True
                    except:
                        pass
                time.sleep(0.05)
            
            # 응답이 없어도 포트가 열려있으면 연결된 것으로 간주
            if self.serial_conn.is_open:
                return True
            else:
                self.connected = False
                return False
                
        except Exception as e:
            print(f"Heartbeat check failed: {e}")
            self.connected = False
            return False
    
    def heartbeat(self):
        """
        주기적으로 연결 상태를 확인하고 터미널에 출력
        """
        # 포트가 없거나 None이면 heartbeat 스킵
        if not self.port:
            return
        
        current_time = time.time()
        if current_time - self.last_heartbeat_time >= self.heartbeat_interval:
            self.last_heartbeat_time = current_time
            
            if self.is_connected():
                is_alive = self.check_connection()
                if is_alive:
                    print(f"[{time.strftime('%H:%M:%S')}] ✓ Arduino 연결 확인: {self.port} - 정상")
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] ✗ Arduino 연결 끊김: {self.port}")
                    self.connected = False
            else:
                # 연결이 안 되어 있을 때는 로그를 너무 자주 출력하지 않음 (1분에 한 번)
                if current_time - getattr(self, '_last_disconnected_log', 0) >= 60:
                    print(f"[{time.strftime('%H:%M:%S')}] ⚠ Arduino 연결 안됨: {self.port}")
                    self._last_disconnected_log = current_time
    
    def camera_to_robot_angles(self, face_data):
        """
        Convert camera face coordinates to robot servo angles for 4-axis tracking
        
        Args:
            face_data: Dictionary containing face detection data
            
        Returns:
            tuple: (base_angle, shoulder_angle, elbow_angle, wrist_angle) in degrees
        """
        if not face_data or not face_data.get('detected'):
            return None, None, None, None
        
        # Get face center in pixels
        face_x = face_data['center_x']
        face_y = face_data['center_y']
        face_size = face_data.get('width', 100)
        
        # Convert to normalized coordinates (-1 to 1)
        norm_x = (face_x - self.camera_width / 2) / (self.camera_width / 2)
        norm_y = (face_y - self.camera_height / 2) / (self.camera_height / 2)
        
        # Calculate angular offsets from center
        angle_offset_h = norm_x * (self.camera_fov_h / 2) * self.movement_sensitivity
        angle_offset_v = norm_y * (self.camera_fov_v / 2) * self.movement_sensitivity
        
        # BASE: Horizontal tracking (left/right)
        # Positive angle_offset_h = face is to the right, rotate base right
        target_base = self.home_positions['base'] + angle_offset_h
        
        # SHOULDER: Vertical tracking (up/down) 
        # Positive angle_offset_v = face is down in image, shoulder should go down
        # Note: Camera Y is inverted (0 at top), so we invert the offset
        target_shoulder = self.home_positions['shoulder'] - angle_offset_v
        
        # ELBOW: Distance-based positioning using face size
        # Larger face = closer = retract elbow, smaller face = farther = extend elbow
        face_size_normalized = face_size / (self.camera_width * 0.2)  # Expected face size ratio
        elbow_offset = (1.0 - face_size_normalized) * 15  # Adjust reach based on distance
        target_elbow = self.home_positions['elbow'] + elbow_offset
        
        # WRIST: Fine vertical adjustment (follows shoulder but with offset)
        # Helps maintain camera orientation toward face
        wrist_offset = angle_offset_v * 0.4  # Partial compensation
        target_wrist = self.home_positions['wrist'] - wrist_offset
        
        # Apply smoothing to reduce jitter
        if self.last_targets:
            target_base = self._smooth_angle(target_base, self.last_targets[0])
            target_shoulder = self._smooth_angle(target_shoulder, self.last_targets[1])
            target_elbow = self._smooth_angle(target_elbow, self.last_targets[2])
            target_wrist = self._smooth_angle(target_wrist, self.last_targets[3])
        
        # Store for next smoothing iteration
        self.last_targets = (target_base, target_shoulder, target_elbow, target_wrist)
        
        return target_base, target_shoulder, target_elbow, target_wrist
    
    def _smooth_angle(self, target, previous):
        """Apply smoothing to reduce servo jitter"""
        return previous + (target - previous) * self.smoothing_factor
    
    def calculate_servo_positions(self, face_data):
        """
        Calculate servo angles for 4-axis face tracking
        
        Args:
            face_data: Dictionary containing face detection data
            
        Returns:
            tuple: (servo_angles, laser_state)
        """
        # Get target angles from face position
        base_angle, shoulder_angle, elbow_angle, wrist_angle = self.camera_to_robot_angles(face_data)
        
        if base_angle is None:
            # No face detected, return to home position
            servo_angles = [
                self.home_positions['base'],
                self.home_positions['shoulder'], 
                self.home_positions['elbow'],
                self.home_positions['wrist']
            ]
            laser_state = 0
            return servo_angles, laser_state
        
        # Clamp all angles to servo limits
        base_angle = self.clamp_angle(base_angle, 'base')
        shoulder_angle = self.clamp_angle(shoulder_angle, 'shoulder')
        elbow_angle = self.clamp_angle(elbow_angle, 'elbow')
        wrist_angle = self.clamp_angle(wrist_angle, 'wrist')
        
        servo_angles = [int(base_angle), int(shoulder_angle), int(elbow_angle), int(wrist_angle)]
        laser_state = 1  # Turn on laser when tracking face
        
        return servo_angles, laser_state
    
    def clamp_angle(self, angle, servo_name):
        """
        Clamp angle to servo limits
        
        Args:
            angle: Input angle in degrees
            servo_name: Name of the servo for limit lookup
            
        Returns:
            int: Clamped angle value
        """
        if servo_name in self.servo_limits:
            min_angle, max_angle = self.servo_limits[servo_name]
            return max(min_angle, min(max_angle, angle))
        return angle
    
    def send_command(self, servo_angles, laser_state):
        """
        Send servo command to Arduino or log if not connected
        """
        command = f"{servo_angles[0]},{servo_angles[1]},{servo_angles[2]},{servo_angles[3]},{laser_state}"
        self.command_log.append(command)
        if len(self.command_log) > 100:
            self.command_log = self.command_log[-100:]
        
        # 연결 상태 재확인
        if not self.is_connected():
            # 연결이 끊어진 것 같으면 다시 확인
            if self.serial_conn is not None and self.serial_conn.is_open:
                self.connected = True
            else:
                return True  # 로그만 기록하면 성공으로 간주
        
        try:
            if self.serial_conn is None:
                print("serial_conn이 None입니다. 명령을 전송할 수 없습니다.")
                return False
                
            if not self.serial_conn.is_open:
                print("시리얼 포트가 열려있지 않습니다.")
                self.connected = False
                return False
            
            # 명령 전송
            self.serial_conn.write((command + '\n').encode())
            self.serial_conn.flush()  # 버퍼 비우기
            
            # 현재 위치 업데이트
            self.current_positions['base'] = servo_angles[0]
            self.current_positions['shoulder'] = servo_angles[1]
            self.current_positions['elbow'] = servo_angles[2]
            self.current_positions['wrist'] = servo_angles[3]
            self.current_positions['laser'] = laser_state
            
            # Read confirmation (optional, timeout 짧게)
            try:
                self.serial_conn.timeout = 0.1
                response = self.serial_conn.readline().decode().strip()
                if response.startswith("OK:"):
                    return True
            except:
                pass  # Ignore read timeout (응답 없어도 성공으로 간주)
            
            return True
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"Error sending command: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _send_home_position(self):
        """Send robot to home position"""
        servo_angles = [
            self.home_positions['base'],
            self.home_positions['shoulder'],
            self.home_positions['elbow'],
            self.home_positions['wrist']
        ]
        self.send_command(servo_angles, 0)
    
    def update_parameters(self, params):
        """
        Update controller parameters for 4-axis robot
        
        Args:
            params: Dictionary containing parameter updates
        """
        if 'camera_width' in params:
            self.camera_width = params['camera_width']
        if 'camera_height' in params:
            self.camera_height = params['camera_height']
        if 'camera_fov_h' in params:
            self.camera_fov_h = params['camera_fov_h']
        if 'camera_fov_v' in params:
            self.camera_fov_v = params['camera_fov_v']
        if 'movement_sensitivity' in params:
            self.movement_sensitivity = params['movement_sensitivity']
        if 'smoothing_factor' in params:
            self.smoothing_factor = params['smoothing_factor']
    
    def get_status(self):
        """
        Get 4-axis robot controller status information
        
        Returns:
            dict: Status information
        """
        return {
            'connected': self.connected,
            'port': self.port,
            'current_positions': self.current_positions,
            'servo_limits': self.servo_limits,
            'home_positions': self.home_positions,
            'camera_settings': {
                'width': self.camera_width,
                'height': self.camera_height,
                'fov_h': self.camera_fov_h,
                'fov_v': self.camera_fov_v
            },
            'tracking_settings': {
                'movement_sensitivity': self.movement_sensitivity,
                'smoothing_factor': self.smoothing_factor
            },
            'robot_configuration': {
                'base': 'Horizontal rotation (left/right)',
                'shoulder': 'Vertical rotation (up/down)', 
                'elbow': 'Reach extension (near/far)',
                'wrist': 'Fine positioning (up/down)'
            }
        }
    
    def close(self):
        """Close Arduino connection"""
        if self.serial_conn and self.serial_conn.is_open:
            # Send home position before closing
            self._send_home_position()
            time.sleep(0.5)
            self.serial_conn.close()
            self.connected = False
            print("Arduino connection closed")
    
    def __del__(self):
        """Destructor to ensure connection is closed"""
        try:
            # 안전하게 연결 닫기 (에러 무시)
            if hasattr(self, 'serial_conn') and self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.close()
                except:
                    pass
        except:
            pass

    @staticmethod
    def list_serial_ports():
        """List all available serial ports"""
        try:
            import serial.tools.list_ports as list_ports
            ports = list_ports.comports()
            port_list = [p.device for p in ports]
            return sorted(port_list)
        except Exception:
            # Fallback: try to find common ports manually
            import platform
            import glob
            system = platform.system()
            
            if system == "Windows":
                # Try COM ports
                ports = []
                for i in range(1, 21):
                    ports.append(f'COM{i}')
                return ports
            elif system == "Darwin":  # macOS
                # Find Mac USB ports
                mac_patterns = ['/dev/cu.usbmodem*', '/dev/cu.usbserial*', 
                               '/dev/tty.usbmodem*', '/dev/tty.usbserial*']
                ports = []
                for pattern in mac_patterns:
                    ports.extend(glob.glob(pattern))
                return sorted(ports)
            else:  # Linux
                # Find Linux USB ports
                linux_patterns = ['/dev/ttyUSB*', '/dev/ttyACM*']
                ports = []
                for pattern in linux_patterns:
                    ports.extend(glob.glob(pattern))
                return sorted(ports)

    def get_command_log(self, n=20):
        return self.command_log[-n:]