import serial
import time
import math
import os

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
    
    def _detect_arduino_port(self):
        """Auto-detect Arduino serial port"""
        # Default Arduino port based on environment variable or common ports
        arduino_port = os.getenv('ARDUINO_PORT')
        if arduino_port:
            return arduino_port
        
        # Try common Arduino ports
        import platform
        system = platform.system()
        
        if system == "Windows":
            # Try COM ports 1-20
            for i in range(1, 21):
                try:
                    test_serial = serial.Serial(f'COM{i}', self.baudrate, timeout=0.1)
                    test_serial.close()
                    return f'COM{i}'
                except:
                    continue
            return 'COM3'  # Default fallback
        else:
            # Linux/Mac
            common_ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/cu.usbmodem*', '/dev/cu.usbserial*']
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
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            self.connected = True
            print(f"Connected to Arduino on {self.port}")
            
            # Send home position on connect
            self._send_home_position()
            return True
            
        except Exception as e:
            print(f"Failed to connect to Arduino on {self.port}: {e}")
            self.connected = False
            return False
    
    def is_connected(self):
        """Check if Arduino is connected"""
        return self.connected and self.serial_conn and self.serial_conn.is_open
    
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
        if not self.is_connected():
            return False
        try:
            if self.serial_conn is not None:
                self.serial_conn.write((command + '\n').encode())
            else:
                print("serial_conn이 None입니다. 명령을 전송할 수 없습니다.")
                return False
            # 현재 위치 업데이트
            self.current_positions['base'] = servo_angles[0]
            self.current_positions['shoulder'] = servo_angles[1]
            self.current_positions['elbow'] = servo_angles[2]
            self.current_positions['wrist'] = servo_angles[3]
            self.current_positions['laser'] = laser_state
            
            # Read confirmation (optional)
            try:
                response = self.serial_conn.readline().decode().strip()
                if response.startswith("OK:"):
                    return True
            except:
                pass  # Ignore read timeout
            
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
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
        self.close()

    @staticmethod
    def list_serial_ports():
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def get_command_log(self, n=20):
        return self.command_log[-n:]