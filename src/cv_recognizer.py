import math
from utils import GestureType

class CVHandRecognizer:
    def __init__(self):
        self.last_grip = None
        
    def analyze_hand(self, landmarks):
        """
        Returns: (GripState, BaseCmd, ShoulderAngle, ElbowAngle, WristAngle, DebugStr)
        """
        wrist = landmarks[0]
        
        # --- 1. GRIPPER (Fist vs Open) ---
        fingers_folded = 0
        if landmarks[8].y > landmarks[5].y: fingers_folded += 1 # Index
        if landmarks[12].y > landmarks[9].y: fingers_folded += 1 # Middle
        if landmarks[16].y > landmarks[13].y: fingers_folded += 1 # Ring
        if landmarks[20].y > landmarks[17].y: fingers_folded += 1 # Pinky
        
        is_fist = fingers_folded >= 3
        grip_cmd = GestureType.CLOSED_HAND if is_fist else GestureType.OPEN_HAND

        # --- 2. BASE (Left/Right) ---
        x = wrist.x
        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        debug_gesture = "CENTER"
        
        # Like Sign Detection (Stop)
        thumb_tip = landmarks[4]
        thumb_ip = landmarks[3]
        index_mcp = landmarks[5]
        
        is_thumb_high = (thumb_tip.y < index_mcp.y)
        is_thumb_up = (thumb_tip.y < thumb_ip.y)
        
        if is_fist and is_thumb_high and is_thumb_up:
            base_cmd = GestureType.STOP_MOVE_HORIZONTAL
            debug_gesture = "👍 LIKE (STOP)"
        else:
            if x < 0.40: 
                base_cmd = GestureType.MOVE_LEFT
                debug_gesture = "⬅️ LEFT"
            elif x > 0.60: 
                base_cmd = GestureType.MOVE_RIGHT
                debug_gesture = "➡️ RIGHT"

        # --- 3. SHOULDER (Height / Y-Axis) ---
        # Move Hand UP -> Shoulder UP (Angle 180)
        # Move Hand DOWN -> Shoulder DOWN (Angle 0)
        y = wrist.y
        shoulder_angle = int((1.0 - y) * 180)
        shoulder_angle = max(0, min(180, shoulder_angle))

        # --- 4. ELBOW (Depth / Z-Axis) ---
        # Logic: Use "Hand Size" to estimate depth.
        # Hand Close to Camera (Big) -> Extend Elbow (Reach out)
        # Hand Far from Camera (Small) -> Retract Elbow (Pull back)
        
        # Calculate distance between Wrist (0) and Middle Finger Knuckle (9)
        # (Using knuckle is more stable than finger tip)
        middle_mcp = landmarks[9]
        hand_size = math.sqrt((middle_mcp.x - wrist.x)**2 + (middle_mcp.y - wrist.y)**2)
        
        # Calibration Thresholds (Adjust these if needed!)
        SIZE_FAR = 0.10   # Hand is far away (Small size)
        SIZE_CLOSE = 0.25 # Hand is close (Big size)
        
        # Normalize size to 0.0 - 1.0 range
        depth_ratio = (hand_size - SIZE_FAR) / (SIZE_CLOSE - SIZE_FAR)
        depth_ratio = max(0.0, min(1.0, depth_ratio))
        
        # Map to Angle: 45 (Retracted) to 160 (Extended)
        elbow_angle = int(45 + (depth_ratio * 115))

        # --- 5. WRIST (Fixed) ---
        wrist_angle = 90

        debug = f"{debug_gesture} | S:{shoulder_angle} E:{elbow_angle}"
        
        return grip_cmd, base_cmd, shoulder_angle, elbow_angle, wrist_angle, debug