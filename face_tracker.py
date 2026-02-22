# 안전한 임포트 (segmentation fault 방지)
# OpenCV는 app.py에서 임포트되므로 여기서는 사용 가능한지만 확인
# cv2는 전역 변수로 받아서 사용 (임포트하지 않음)
CV2_AVAILABLE = False
cv2 = None
NUMPY_AVAILABLE = False
np = None

# numpy만 임포트 (OpenCV는 사용하는 쪽에서 전달받음)
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except:
    pass

class FaceTracker:
    def __init__(self, camera_index=0, cv2_module=None):
        """Initialize face tracker with camera and cascade classifier
        
        Args:
            camera_index: Camera index to use
            cv2_module: OpenCV module (cv2) - must be passed from app.py
        """
        # Use module-level cv2/CV2_AVAILABLE and numpy flags
        global cv2, CV2_AVAILABLE, NUMPY_AVAILABLE, np

        if cv2_module is not None:
            cv2 = cv2_module
            CV2_AVAILABLE = True
        else:
            # Try importing cv2 if not provided
            try:
                import cv2 as _cv2
                cv2 = _cv2
                CV2_AVAILABLE = True
            except Exception:
                CV2_AVAILABLE = False

        if not CV2_AVAILABLE or cv2 is None:
            raise Exception("OpenCV is not available. cv2 module must be passed or already imported.")
        
        self.cap = None
        self.face_cascade = None
        self.current_frame = None
        self.camera_width = 640
        self.camera_height = 480
        self.camera_index = camera_index
        self.preview_mode = False
        
        # Initialize face cascade classifier first (안전하게)
        try:
            # cv2.data.haarcascades가 없는 경우를 대비하여 경로를 안전하게 가져오기
            try:
                import cv2.data
                cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            except (ImportError, AttributeError):
                # 일반적인 설치 경로나 상대경로로 대체
                cascade_path = "haarcascade_frontalface_default.xml"
            
            # 안전하게 cascade 로드
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            
            if self.face_cascade is None or self.face_cascade.empty():
                raise Exception("Cannot load face cascade classifier")
        except Exception as e:
            print(f"Face cascade initialization error: {e}")
            self.face_cascade = None
            
        # Initialize camera (에러가 발생해도 객체는 생성)
        # init_camera는 안전하게 처리되므로 여기서 호출
        try:
            self.init_camera(camera_index)
        except Exception as e:
            print(f"Camera initialization failed in __init__: {e}")
            # 카메라 초기화 실패해도 객체는 생성 (나중에 재시도 가능)
            self.cap = None
    
    def init_camera(self, camera_index):
        """Initialize camera with given index"""
        try:
            # 기존 카메라 안전하게 해제
            if self.cap is not None:
                try:
                    self.cap.release()
                except:
                    pass
                self.cap = None
            
            # 짧은 대기 (시스템 안정화)
            import time
            time.sleep(0.1)
            
            self.camera_index = camera_index
            self.cap = cv2.VideoCapture(camera_index)
            
            # 안전성 체크
            if self.cap is None:
                raise Exception(f"Failed to create VideoCapture object for camera {camera_index}")
            
            if not self.cap.isOpened():
                raise Exception(f"Cannot open camera {camera_index}")
            
            # 카메라가 실제로 작동하는지 테스트 (프레임 읽기 시도)
            ret, frame = self.cap.read()
            if not ret or frame is None:
                raise Exception(f"Camera {camera_index} opened but cannot read frames")
            
            # Set camera resolution (안전하게)
            try:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            except:
                # 해상도 설정 실패해도 계속 진행
                pass
            
            return True
                
        except Exception as e:
            print(f"Camera initialization error: {e}")
            # 안전하게 정리
            if self.cap is not None:
                try:
                    self.cap.release()
                except:
                    pass
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
        # 안전하게 카메라를 테스트 (segmentation fault 방지)
        import time
        for i in range(10):  # Check first 10 camera indices
            test_cap = None
            try:
                test_cap = cv2.VideoCapture(i)
                if test_cap is not None and test_cap.isOpened():
                    # 실제로 프레임을 읽을 수 있는지 테스트
                    ret, frame = test_cap.read()
                    if ret and frame is not None:
                        available_cameras.append(i)
            except Exception:
                # 에러 발생 시 해당 카메라는 스킵
                pass
            finally:
                # 안전하게 해제
                if test_cap is not None:
                    try:
                        test_cap.release()
                    except Exception:
                        pass
                # 다음 카메라 테스트 전에 짧은 대기 (시스템 안정화)
                time.sleep(0.05)
        return available_cameras
    
    def process_frame(self):
        """Process current frame and detect faces"""
        if not self.is_initialized():
            return None, None

        if self.cap is None:
            return None, None
        
        try:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                return None, None

            # 프레임 크기 체크 (segmentation fault 방지)
            if frame.size == 0:
                return None, None

            # Resize frame for consistent processing
            try:
                frame = cv2.resize(frame, (self.camera_width, self.camera_height))
                self.current_frame = frame.copy()
            except Exception as e:
                print(f"Frame resize error: {e}")
                return None, None
        except Exception as e:
            print(f"Frame read error: {e}")
            return None, None
        
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
        
        # 얼굴 감지 (파라미터 조정: 더 민감하게 감지)
        if self.face_cascade is not None:
            faces = self.face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.2,  # 더 자세히 검색 (1.3 -> 1.2)
                minNeighbors=4,   # 더 낮은 임계값 (5 -> 4)
                minSize=(20, 20)  # 더 작은 얼굴도 감지 (30 -> 20)
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
        # cv2 may be None or may not provide destroyAllWindows in some headless setups
        try:
            if cv2 is not None and hasattr(cv2, 'destroyAllWindows'):
                cv2.destroyAllWindows()
        except Exception:
            pass
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        self.cleanup()
