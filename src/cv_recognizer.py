import math
from utils import GestureType

class CVHandRecognizer:
    def __init__(self):
        self.last_grip = None
        
        # Smoothing factors (Simple Low-pass filter)
        self.prev_x = 0.5
        self.prev_y = 0.5
        self.prev_z = 0
        self.prev_roll = 90
        
    def analyze_hand(self, landmarks):
        """
        Returns: (GripState, BaseCmd, ShoulderAngle, ElbowAngle, WristAngle, DebugStr)
        """
        # 1. Extract Key Points
        wrist = landmarks[0]
        index_mcp = landmarks[5]
        pinky_mcp = landmarks[17]
        
        # --- A. BASE (X-Axis) ---
        # Logic: Zones. Left < 30%, Right > 70%
        x = wrist.x
        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        if x < 0.3: base_cmd = GestureType.MOVE_LEFT
        elif x > 0.7: base_cmd = GestureType.MOVE_RIGHT
        
        # --- B. SHOULDER (Y-Axis) ---
        # Logic: Map Screen Y (0.0 top to 1.0 bottom) to Angle (0 to 180)
        # Inverted: Screen Top (0) -> Robot Up (180)
        y = wrist.y
        shoulder_angle = int((1.0 - y) * 180)
        shoulder_angle = max(45, min(135, shoulder_angle)) # Limit range for safety
        
        # --- C. ELBOW (Depth/Z-Axis) ---
        # Logic: Estimate depth by hand size (distance between wrist and middle finger tip)
        # Closer (Big Hand) = Extend. Farther (Small Hand) = Retract.
        # Note: This needs calibration based on your camera distance!
        middle_tip = landmarks[12]
        hand_size = math.sqrt((middle_tip.x - wrist.x)**2 + (middle_tip.y - wrist.y)**2)
        
        # Map size 0.1 (Far) to 0.4 (Close) -> Angle 45 (Retract) to 160 (Extend)
        # You might need to tweak these 0.1 and 0.4 values
        norm_z = (hand_size - 0.1) / (0.3) # Normalize 0 to 1
        elbow_angle = int(norm_z * 115 + 45)
        elbow_angle = max(45, min(160, elbow_angle))

        # --- D. WRIST (Roll) ---
        # Logic: Angle between Index MCP and Pinky MCP
        dx = pinky_mcp.x - index_mcp.x
        dy = pinky_mcp.y - index_mcp.y
        roll_rad = math.atan2(dy, dx)
        roll_deg = math.degrees(roll_rad)
        # Map -45 to 45 degrees tilt -> 0 to 180 servo
        wrist_angle = int(roll_deg + 90) 
        wrist_angle = max(0, min(180, wrist_angle))

        # --- E. GRIPPER (Fist) ---
        fingers_folded = 0
        if landmarks[8].y > landmarks[5].y: fingers_folded += 1
        if landmarks[12].y > landmarks[9].y: fingers_folded += 1
        if landmarks[16].y > landmarks[13].y: fingers_folded += 1
        if landmarks[20].y > landmarks[17].y: fingers_folded += 1
        
        is_closed = fingers_folded >= 3
        grip_cmd = GestureType.CLOSED_HAND if is_closed else GestureType.OPEN_HAND

        # Debug Info
        debug = f"S:{shoulder_angle} E:{elbow_angle} W:{wrist_angle}"
        
        return grip_cmd, base_cmd, shoulder_angle, elbow_angle, wrist_angle, debug