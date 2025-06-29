import math

class InverseKinematics:
    def __init__(self, L1=10.0, L2=10.0):
        """
        Initialize inverse kinematics solver for 2-DOF planar robotic arm
        
        Args:
            L1: Length of first link (shoulder to elbow) in cm
            L2: Length of second link (elbow to wrist) in cm
        """
        self.L1 = L1  # Shoulder to elbow length
        self.L2 = L2  # Elbow to wrist length
        self.max_reach = L1 + L2
        self.min_reach = abs(L1 - L2)
    
    def solve(self, x, y):
        """
        Solve inverse kinematics for target position (x, y)
        
        Args:
            x: X coordinate in cm (relative to robot base)
            y: Y coordinate in cm (relative to robot base)
            
        Returns:
            tuple: (shoulder_angle, elbow_angle, wrist_angle) in degrees or None if unreachable
        """
        # Calculate distance to target
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if distance > self.max_reach or distance < self.min_reach:
            return None
        
        # Calculate elbow angle using law of cosines
        D = (x*x + y*y - self.L1*self.L1 - self.L2*self.L2) / (2 * self.L1 * self.L2)
        
        # Clamp D to valid range to avoid numerical errors
        D = max(-1.0, min(1.0, D))
        
        # Calculate theta2 (elbow angle) - using positive solution for "elbow up" configuration
        theta2 = math.atan2(math.sqrt(1 - D*D), D)
        
        # Calculate theta1 (shoulder angle)
        theta1 = math.atan2(y, x) - math.atan2(self.L2 * math.sin(theta2), 
                                                self.L1 + self.L2 * math.cos(theta2))
        
        # Convert to degrees
        shoulder_angle = math.degrees(theta1)
        elbow_angle = math.degrees(theta2)
        
        # Calculate wrist angle to maintain end-effector orientation
        # Simple approach: keep end-effector parallel to ground
        wrist_angle = 180 - elbow_angle
        
        return shoulder_angle, elbow_angle, wrist_angle
    
    def forward_kinematics(self, shoulder_angle, elbow_angle):
        """
        Calculate end-effector position from joint angles
        
        Args:
            shoulder_angle: Shoulder joint angle in degrees
            elbow_angle: Elbow joint angle in degrees
            
        Returns:
            tuple: (x, y) position of end-effector in cm
        """
        theta1 = math.radians(shoulder_angle)
        theta2 = math.radians(elbow_angle)
        
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        
        return x, y
    
    def get_workspace_bounds(self):
        """
        Get workspace boundaries
        
        Returns:
            dict: Dictionary containing workspace limits
        """
        return {
            'max_reach': self.max_reach,
            'min_reach': self.min_reach,
            'L1': self.L1,
            'L2': self.L2
        }
    
    def update_link_lengths(self, L1, L2):
        """
        Update link lengths
        
        Args:
            L1: New length of first link in cm
            L2: New length of second link in cm
        """
        self.L1 = L1
        self.L2 = L2
        self.max_reach = L1 + L2
        self.min_reach = abs(L1 - L2)
