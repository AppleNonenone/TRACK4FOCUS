class Track4FocusApp {
    constructor() {
        this.isTracking = false;
        this.isPreview = false;
        this.statusUpdateInterval = null;
        this.initializeElements();
        this.bindEvents();
        this.updateStatus();
    }

    initializeElements() {
        // Control elements
        this.startBtn = document.getElementById('startBtn');
        this.stopBtn = document.getElementById('stopBtn');
        this.previewBtn = document.getElementById('previewBtn');
        this.stopPreviewBtn = document.getElementById('stopPreviewBtn');
        this.updateParamsBtn = document.getElementById('updateParamsBtn');
        this.cameraSelect = document.getElementById('cameraSelect');
        
        // Status elements
        this.systemStatus = document.getElementById('systemStatus');
        this.previewStatus = document.getElementById('previewStatus');
        this.trackingStatus = document.getElementById('trackingStatus');
        this.faceStatus = document.getElementById('faceStatus');
        this.laserStatus = document.getElementById('laserStatus');
        this.portStatus = document.getElementById('portStatus');
        this.cameraStatus = document.getElementById('cameraStatus');
        
        // Servo angle elements
        this.baseAngle = document.getElementById('baseAngle');
        this.shoulderAngle = document.getElementById('shoulderAngle');
        this.elbowAngle = document.getElementById('elbowAngle');
        this.wristAngle = document.getElementById('wristAngle');
        
        // Parameter inputs
        this.l1Input = document.getElementById('l1Input');
        this.l2Input = document.getElementById('l2Input');
        this.thresholdInput = document.getElementById('thresholdInput');
        
        // Video feed
        this.videoFeed = document.getElementById('videoFeed');
        
        // Message containers
        this.errorContainer = document.getElementById('errorContainer');
        this.successContainer = document.getElementById('successContainer');
        this.errorText = document.getElementById('errorText');
        this.successText = document.getElementById('successText');
        this.closeError = document.getElementById('closeError');
        this.closeSuccess = document.getElementById('closeSuccess');
        
        this.arduinoPortSelect = document.getElementById('arduinoPortSelect');
        this.setArduinoPortBtn = document.getElementById('setArduinoPortBtn');
        this.refreshPortsBtn = document.getElementById('refreshPortsBtn');
        this.commandLogTerminal = document.getElementById('commandLogTerminal');
        
        this.sensitivitySlider = document.getElementById('movement-sensitivity');
        this.sensitivityValue = document.getElementById('sensitivity-value');
        this.smoothingSlider = document.getElementById('smoothing-factor');
        this.smoothingValue = document.getElementById('smoothing-value');
        
        this.laserAutoToggle = document.getElementById('laserAutoToggle');
        this.laserAutoToggleLabel = document.getElementById('laserAutoToggleLabel');
        
        this.resizerBar = document.getElementById('resizerBar');
        this.sidebar = document.querySelector('.sidebar.menu-scroll');
        this.mainFeed = document.querySelector('.main-feed');
        
        this.servoLaser = document.getElementById('servoLaser');
    }

    bindEvents() {
        // Control buttons
        this.startBtn.addEventListener('click', async () => {
            try {
                await this.startTracking();
            } catch (e) { this.showError(e.message); }
        });
        this.stopBtn.addEventListener('click', async () => {
            try {
                await this.stopTracking();
            } catch (e) { this.showError(e.message); }
        });
        this.previewBtn.addEventListener('click', async () => {
            try {
                await this.startPreview();
            } catch (e) { this.showError(e.message); }
        });
        this.stopPreviewBtn.addEventListener('click', async () => {
            try {
                await this.stopPreview();
            } catch (e) { this.showError(e.message); }
        });
        this.updateParamsBtn.addEventListener('click', async () => {
            try {
                await this.updateParameters();
            } catch (e) { this.showError(e.message); }
        });
        this.cameraSelect.addEventListener('change', async () => {
            try {
                await this.changeCamera();
            } catch (e) { this.showError(e.message); }
        });
        
        // Parameter sliders
        this.sensitivitySlider.addEventListener('input', () => {
            // 실시간 값 반영
            if (this.sensitivityValue) this.sensitivityValue.textContent = this.sensitivitySlider.value;
        });
        
        this.smoothingSlider.addEventListener('input', () => {
            if (this.smoothingValue) this.smoothingValue.textContent = this.smoothingSlider.value;
        });
        
        // Message close buttons
        if (this.closeError) this.closeError.addEventListener('click', () => this.hideError());
        if (this.closeSuccess) this.closeSuccess.addEventListener('click', () => this.hideSuccess());
        
        // Auto-hide messages after 5 seconds
        setTimeout(() => this.hideError(), 5000);
        setTimeout(() => this.hideSuccess(), 5000);
        
        if (this.setArduinoPortBtn) this.setArduinoPortBtn.addEventListener('click', async () => {
            try {
                await this.setArduinoPort();
            } catch (e) { this.showError(e.message); }
        });
        
        if (this.refreshPortsBtn) this.refreshPortsBtn.addEventListener('click', async () => {
            try {
                this.refreshPortsBtn.disabled = true;
                this.refreshPortsBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i>';
                await this.fetchArduinoPorts();
                this.showSuccess('포트 목록이 새로고침되었습니다');
            } catch (e) { 
                this.showError(e.message); 
            } finally {
                this.refreshPortsBtn.disabled = false;
                this.refreshPortsBtn.innerHTML = '<i class="fas fa-sync-alt"></i>';
            }
        });
        
        if (this.laserAutoToggle) this.laserAutoToggle.addEventListener('change', async () => {
            try {
                await this.setLaserAutoControl();
            } catch (e) { this.showError(e.message); }
        });
        
        // 리사이저 바 드래그 이벤트
        let isResizing = false;
        let startX = 0;
        let startSidebarPercent = 0;
        this.resizerBar.addEventListener('mousedown', (e) => {
            isResizing = true;
            startX = e.clientX;
            const layoutRect = document.querySelector('.layout').getBoundingClientRect();
            const sidebarRect = this.sidebar.getBoundingClientRect();
            startSidebarPercent = ((sidebarRect.width) / layoutRect.width) * 100;
            document.body.style.cursor = 'ew-resize';
            document.body.style.userSelect = 'none';
        });
        document.addEventListener('mousemove', (e) => {
            if (!isResizing) return;
            const layoutRect = document.querySelector('.layout').getBoundingClientRect();
            let dx = e.clientX - startX;
            let newSidebarPercent = startSidebarPercent + (dx / layoutRect.width) * 100;
            newSidebarPercent = Math.max(12, Math.min(45, newSidebarPercent));
            this.sidebar.style.flexBasis = newSidebarPercent + '%';
            this.mainFeed.style.flexBasis = (100 - newSidebarPercent) + '%';
        });
        document.addEventListener('mouseup', () => {
            if (isResizing) {
                isResizing = false;
                document.body.style.cursor = '';
                document.body.style.userSelect = '';
            }
        });
    }

    async startPreview() {
        try {
            this.previewBtn.disabled = true;
            this.previewBtn.innerHTML = '<span class="loading"></span> Starting...';
            
            const response = await fetch('/start_preview', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                cache: 'no-store'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.isPreview = true;
                this.updateControlButtons();
                this.startStatusUpdates();
                this.showSuccess(result.message);
            } else {
                this.showError(result.message);
            }
        } catch (error) {
            this.showError(`Failed to start preview: ${error.message}`);
        } finally {
            this.previewBtn.disabled = false;
            this.previewBtn.innerHTML = '<i class="fas fa-eye"></i> Start Preview';
        }
    }

    async stopPreview() {
        try {
            this.stopPreviewBtn.disabled = true;
            this.stopPreviewBtn.innerHTML = '<span class="loading"></span> Stopping...';
            
            const response = await fetch('/stop_preview', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                cache: 'no-store'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.isPreview = false;
                this.updateControlButtons();
                this.stopStatusUpdates();
                this.showSuccess(result.message);
            } else {
                this.showError(result.message);
            }
        } catch (error) {
            this.showError(`Failed to stop preview: ${error.message}`);
        } finally {
            this.stopPreviewBtn.disabled = false;
            this.stopPreviewBtn.innerHTML = '<i class="fas fa-eye-slash"></i> Stop Preview';
        }
    }

    async startTracking() {
        try {
            this.startBtn.disabled = true;
            this.startBtn.innerHTML = '<span class="loading"></span> Starting...';
            
            const response = await fetch('/start_tracking', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                cache: 'no-store'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.isTracking = true;
                this.isPreview = false;
                this.updateControlButtons();
                this.startStatusUpdates();
                this.showSuccess(result.message);
                document.getElementById('videoFeedOverlay').style.display = 'none';
                document.getElementById('videoFeedOverlay').textContent = '';
            } else {
                this.showError(result.message);
            }
        } catch (error) {
            this.showError(`Failed to start tracking: ${error.message}`);
        } finally {
            this.startBtn.disabled = false;
            this.startBtn.innerHTML = '<i class="fas fa-play"></i> Start Tracking';
        }
    }

    async stopTracking() {
        try {
            this.stopBtn.disabled = true;
            this.stopBtn.innerHTML = '<span class="loading"></span> Stopping...';
            
            const response = await fetch('/stop_tracking', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                cache: 'no-store'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.isTracking = false;
                this.updateControlButtons();
                this.stopStatusUpdates();
                this.showSuccess(result.message);
                document.getElementById('videoFeedOverlay').style.display = 'flex';
                document.getElementById('videoFeedOverlay').textContent = 'Tracking stopped';
            } else {
                this.showError(result.message);
            }
        } catch (error) {
            this.showError(`Failed to stop tracking: ${error.message}`);
        } finally {
            this.stopBtn.disabled = false;
            this.stopBtn.innerHTML = '<i class="fas fa-stop"></i> Stop Tracking';
        }
    }

    async updateParameters() {
        try {
            const parameters = {
                movement_sensitivity: parseFloat(this.sensitivitySlider.value),
                smoothing_factor: parseFloat(this.smoothingSlider.value),
                center_threshold: parseInt(this.thresholdInput.value),
                L1: this.l1Input ? parseFloat(this.l1Input.value) : undefined,
                L2: this.l2Input ? parseFloat(this.l2Input.value) : undefined
            };
            const response = await fetch('/set_parameters', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(parameters),
                cache: 'no-store'
            });
            const result = await response.json();
            if (result.success) {
                this.showSuccess('Parameters updated successfully');
            } else {
                this.showError(result.message);
            }
        } catch (error) {
            this.showError(`Failed to update parameters: ${error.message}`);
        }
    }

    async changeCamera() {
        try {
            const cameraIndex = parseInt(this.cameraSelect.value);
            
            const response = await fetch('/change_camera', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ camera_index: cameraIndex }),
                cache: 'no-store'
            });
            
            const result = await response.json();
            
            if (result.success) {
                this.showSuccess(result.message);
            } else {
                this.showError(result.message);
            }
        } catch (error) {
            this.showError(`Failed to change camera: ${error.message}`);
        }
    }

    async updateStatus() {
        try {
            const response = await fetch('/status', {
                cache: 'no-store'
            });
            const status = await response.json();
            
            // Update available cameras dropdown
            if (status.available_cameras && status.available_cameras.length > 0) {
                this.updateCameraOptions(status.available_cameras, status.camera_index);
            }
            
            // Update system status
            if (status.error) {
                this.systemStatus.textContent = 'Error';
                this.systemStatus.className = 'status disconnected';
                this.showError(status.error);
            } else if (status.tracking) {
                this.systemStatus.textContent = 'Tracking';
                this.systemStatus.className = 'status tracking';
            } else if (status.preview) {
                this.systemStatus.textContent = 'Preview';
                this.systemStatus.className = 'status connected';
            } else {
                this.systemStatus.textContent = 'Ready';
                this.systemStatus.className = 'status connected';
            }
            
            // Update preview status
            this.previewStatus.textContent = status.preview ? 'Active' : 'Inactive';
            this.previewStatus.style.color = status.preview ? '#22543d' : '#742a2a';
            
            // Update tracking status
            this.trackingStatus.textContent = status.tracking ? 'Active' : 'Inactive';
            this.trackingStatus.style.color = status.tracking ? '#22543d' : '#742a2a';
            
            // Update face detection status
            this.faceStatus.textContent = status.face_detected ? 'Yes' : 'No';
            this.faceStatus.style.color = status.face_detected ? '#22543d' : '#742a2a';
            
            // Update laser status
            this.laserStatus.textContent = status.laser_state ? 'ON' : 'OFF';
            this.laserStatus.style.color = status.laser_state ? '#22543d' : '#742a2a';
            if (this.servoLaser) this.servoLaser.textContent = status.laser_state;
            
            // Update servo angles
            if (status.servo_angles && status.servo_angles.length >= 4) {
                this.baseAngle.textContent = `${status.servo_angles[0]}°`;
                this.shoulderAngle.textContent = `${status.servo_angles[1]}°`;
                this.elbowAngle.textContent = `${status.servo_angles[2]}°`;
                this.wristAngle.textContent = `${status.servo_angles[3]}°`;
            }
            
            // 파라미터 실시간 반영
            if (status.tracking_settings) {
                if (status.tracking_settings.movement_sensitivity !== undefined) {
                    this.sensitivitySlider.value = status.tracking_settings.movement_sensitivity;
                    this.sensitivityValue.textContent = status.tracking_settings.movement_sensitivity;
                }
                if (status.tracking_settings.smoothing_factor !== undefined) {
                    this.smoothingSlider.value = status.tracking_settings.smoothing_factor;
                    this.smoothingValue.textContent = status.tracking_settings.smoothing_factor;
                }
            }
            
            if (status.laser_auto_control !== undefined) {
                this.laserAutoToggle.checked = status.laser_auto_control;
                this.laserAutoToggleLabel.textContent = status.laser_auto_control ? 'ON' : 'OFF';
            }
            
            // 트래킹 상태에 따라 카메라 feed 오버레이 표시
            if (status.tracking) {
                document.getElementById('videoFeedOverlay').style.display = 'none';
                document.getElementById('videoFeedOverlay').textContent = '';
            } else {
                document.getElementById('videoFeedOverlay').style.display = 'flex';
                document.getElementById('videoFeedOverlay').textContent = 'Tracking stopped';
            }
            
            this.portStatus.textContent = status.port !== undefined ? status.port : '-';
            this.cameraStatus.textContent = status.camera_index !== undefined ? status.camera_index : '-';
            
            // Update Arduino port dropdown if it's empty
            if (this.arduinoPortSelect && this.arduinoPortSelect.options.length === 0) {
                await this.fetchArduinoPorts();
            }
            
        } catch (error) {
            console.error('Failed to update status:', error);
        }
    }

    updateCameraOptions(availableCameras, currentIndex) {
        const currentValue = this.cameraSelect.value;
        this.cameraSelect.innerHTML = '';
        
        availableCameras.forEach(index => {
            const option = document.createElement('option');
            option.value = index;
            option.textContent = `Camera ${index}`;
            if (index === currentIndex) {
                option.selected = true;
            }
            this.cameraSelect.appendChild(option);
        });
        
        // If no cameras found, add default option
        if (availableCameras.length === 0) {
            const option = document.createElement('option');
            option.value = 0;
            option.textContent = 'No cameras detected';
            option.disabled = true;
            this.cameraSelect.appendChild(option);
        }
    }

    updateControlButtons() {
        this.startBtn.disabled = this.isTracking;
        this.stopBtn.disabled = !this.isTracking;
        this.previewBtn.disabled = this.isPreview || this.isTracking;
        this.stopPreviewBtn.disabled = !this.isPreview;
        this.cameraSelect.disabled = this.isTracking;
    }

    startStatusUpdates() {
        if (this.statusUpdateInterval) {
            clearInterval(this.statusUpdateInterval);
        }
        this.statusUpdateInterval = setInterval(() => {
            this.updateStatus();
            this.fetchCommandLog();
        }, 1000);
    }

    stopStatusUpdates() {
        if (this.statusUpdateInterval) {
            clearInterval(this.statusUpdateInterval);
            this.statusUpdateInterval = null;
        }
    }

    showError(message) {
        this.errorText.textContent = message;
        this.errorContainer.style.display = 'block';
        this.hideSuccess(); // Hide success message if showing
        
        // Auto-hide after 10 seconds
        setTimeout(() => this.hideError(), 10000);
    }

    showSuccess(message) {
        this.successText.textContent = message;
        this.successContainer.style.display = 'block';
        this.hideError(); // Hide error message if showing
        
        // Auto-hide after 5 seconds
        setTimeout(() => this.hideSuccess(), 5000);
    }

    hideError() {
        this.errorContainer.style.display = 'none';
    }

    hideSuccess() {
        this.successContainer.style.display = 'none';
    }

    async fetchArduinoPorts() {
        try {
            const res = await fetch('/arduino_ports', {
                cache: 'no-store'
            });
            const data = await res.json();
            this.arduinoPortSelect.innerHTML = '';
            
            if (data.ports && data.ports.length > 0) {
                data.ports.forEach(port => {
                    const option = document.createElement('option');
                    option.value = port;
                    option.textContent = port;
                    this.arduinoPortSelect.appendChild(option);
                });
            } else {
                // 포트가 없을 때 기본 옵션 추가
                const option = document.createElement('option');
                option.value = '';
                option.textContent = 'No ports detected';
                option.disabled = true;
                this.arduinoPortSelect.appendChild(option);
            }
        } catch (error) {
            console.error('Failed to fetch Arduino ports:', error);
            this.arduinoPortSelect.innerHTML = '';
            const option = document.createElement('option');
            option.value = '';
            option.textContent = 'Error loading ports';
            option.disabled = true;
            this.arduinoPortSelect.appendChild(option);
        }
    }

    async setArduinoPort() {
        const port = this.arduinoPortSelect.value;
        const res = await fetch('/set_arduino_port', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ port }),
            cache: 'no-store'
        });
        const data = await res.json();
        if (data.success) {
            this.showSuccess(data.message);
        } else {
            this.showError(data.message);
        }
    }

    async fetchCommandLog() {
        const res = await fetch('/command_log', {
            cache: 'no-store'
        });
        const data = await res.json();
        this.commandLogTerminal.innerHTML = data.log.map(line => `<div>${line}</div>`).join('');
        this.commandLogTerminal.scrollTop = this.commandLogTerminal.scrollHeight;
    }

    async setLaserAutoControl() {
        const auto = this.laserAutoToggle.checked;
        const res = await fetch('/set_laser_auto_control', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ auto }),
            cache: 'no-store'
        });
        const data = await res.json();
        this.laserAutoToggleLabel.textContent = auto ? 'ON' : 'OFF';
    }
}

// Register service worker for PWA functionality
if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
        navigator.serviceWorker.register('/static/sw.js')
            .then((registration) => {
                console.log('SW registered: ', registration);
            })
            .catch((registrationError) => {
                console.log('SW registration failed: ', registrationError);
            });
    });
}

// Initialize the application when the page loads
window.addEventListener('DOMContentLoaded', () => {
    window.track4focusApp = new Track4FocusApp();
    
    // Load Arduino ports immediately
    window.track4focusApp.fetchArduinoPorts();
    
    // Update status immediately and then every 2 seconds
    window.track4focusApp.updateStatus();
    setInterval(() => window.track4focusApp.updateStatus(), 2000);
});

// Handle video feed errors
document.getElementById('videoFeed').addEventListener('error', function() {
    this.style.display = 'none';
    const container = this.parentElement;
    if (!container.querySelector('.video-error')) {
        const errorDiv = document.createElement('div');
        errorDiv.className = 'video-error';
        errorDiv.innerHTML = `
            <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; color: #666;">
                <i class="fas fa-video-slash" style="font-size: 3em; margin-bottom: 20px;"></i>
                <p>Camera feed unavailable</p>
                <p style="font-size: 0.9em; margin-top: 10px;">Start tracking to begin video stream</p>
            </div>
        `;
        container.appendChild(errorDiv);
    }
});

// Handle successful video feed load
document.getElementById('videoFeed').addEventListener('load', function() {
    this.style.display = 'block';
    const errorDiv = this.parentElement.querySelector('.video-error');
    if (errorDiv) {
        errorDiv.remove();
    }
});
