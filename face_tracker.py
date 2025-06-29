import cv2
import numpy as np

class FaceTracker:
    def __init__(self, camera_index=0):
        """Initialize face tracker with camera and cascade classifier"""
        self.cap = None
        self.face_cascade = None
        self.current_frame = None
        self.camera_width = 640
        self.camera_height = 480
        self.camera_index = camera_index
        self.preview_mode = False
        
        # Initialize face cascade classifier first
        try:
            # cv2.data.haarcascades가 없는 경우를 대비하여 경로를 안전하게 가져오기
            try:
                import cv2.data
                cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            except (ImportError, AttributeError):
                # 일반적인 설치 경로나 상대경로로 대체
                cascade_path = "haarcascade_frontalface_default.xml"
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            
            if self.face_cascade.empty():
                raise Exception("Cannot load face cascade classifier")
        except Exception as e:
            print(f"Face cascade initialization error: {e}")
            
        # Initialize camera
        self.init_camera(camera_index)
    
    def init_camera(self, camera_index):
        """Initialize camera with given index"""
        try:
            if self.cap:
                self.cap.release()
            
            self.camera_index = camera_index
            self.cap = cv2.VideoCapture(camera_index)
            
            if not self.cap.isOpened():
                raise Exception(f"Cannot open camera {camera_index}")
            
            # Set camera resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            
            return True
                
        except Exception as e:
            print(f"Camera initialization error: {e}")
            self.cap = None
            return False
    
    def is_initialized(self):
        """Check if face tracker is properly initialized"""
        return self.cap is not None and self.cap.isOpened() and self.face_cascade is not None
    
    def set_preview_mode(self, preview=True):
        """Set preview mode - shows video without face detection processing"""
        self.preview_mode = preview
    
    def change_camera(self, camera_index):
        """Change to a different camera"""
        return self.init_camera(camera_index)
    
    def get_available_cameras(self):
        """Get list of available camera indices"""
        available_cameras = []
        for i in range(10):  # Check first 10 camera indices
            test_cap = cv2.VideoCapture(i)
            if test_cap.isOpened():
                available_cameras.append(i)
            test_cap.release()
        return available_cameras
    
    def process_frame(self):
        """Process current frame and detect faces"""
        if not self.is_initialized():
            return None, None

        if self.cap is None:
            return None, None

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None, None

        # Resize frame for consistent processing
        frame = cv2.resize(frame, (self.camera_width, self.camera_height))
        self.current_frame = frame.copy()
        
        # Add crosshair at screen center for reference
        cv2.line(self.current_frame, (self.camera_width//2 - 20, self.camera_height//2), 
                (self.camera_width//2 + 20, self.camera_height//2), (255, 0, 0), 2)
        cv2.line(self.current_frame, (self.camera_width//2, self.camera_height//2 - 20), 
                (self.camera_width//2, self.camera_height//2 + 20), (255, 0, 0), 2)
        
        # If in preview mode, skip face detection
        if self.preview_mode:
            return self.current_frame, None
        
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 얼굴 감지
        if self.face_cascade is not None:
            faces = self.face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.3, 
                minNeighbors=5,
                minSize=(30, 30)
            )
        else:
            faces = []

        if len(faces) > 0:
            # Use the largest face (first in the array after sorting by area)
            face_areas = [(w * h, (x, y, w, h)) for x, y, w, h in faces]
            face_areas.sort(reverse=True)
            x, y, w, h = face_areas[0][1]
            
            # Calculate face center
            face_center_x = x + w // 2
            face_center_y = y + h // 2
            
            # Draw face rectangle and center point on current frame
            cv2.rectangle(self.current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(self.current_frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)
            
            return self.current_frame, {
                'center_x': face_center_x,
                'center_y': face_center_y,
                'width': w,
                'height': h,
                'confidence': 1.0,  # Haar cascades don't provide confidence scores
                'detected': True
            }
        
        return self.current_frame, None
    
    def get_display_frame(self):
        """Get current frame for display purposes"""
        return self.process_frame()
    
    def get_camera_dimensions(self):
        """Get camera resolution"""
        return self.camera_width, self.camera_height
    
    def cleanup(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        self.cleanup()
