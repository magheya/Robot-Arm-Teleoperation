import math
from utils import GestureType

class CVHandRecognizer:
    def __init__(self):
        # Smoothing buffers (Simple Low-Pass Filter)
        self.smooth_shoulder = 150
        self.smooth_elbow = 90
        self.alpha = 0.2  # Smoothing factor (Lower = Smoother but slower)

    def analyze_bimanual(self, right_hand, left_hand):
        """
        BIMANUAL MODE:
        Right Hand -> Aiming (Base Rotation + Shoulder Height)
        Left Hand  -> Action (Elbow Extension + Gripper Trigger)
        """
        # --- RIGHT HAND (The Turret) ---
        # 1. Base (X-Axis Rate Control)
        # Deadzone in the middle (0.4 to 0.6) prevents drift
        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        if right_hand[0].x < 0.4: base_cmd = GestureType.MOVE_LEFT
        elif right_hand[0].x > 0.6: base_cmd = GestureType.MOVE_RIGHT

        # 2. Shoulder (Y-Axis Position)
        # Map Screen Y (1.0 bottom to 0.0 top) to Angles (0 to 150)
        # We limit max height to 150 to keep it safe
        target_shoulder = int((1.0 - right_hand[0].y) * 160)
        target_shoulder = max(40, min(160, target_shoulder)) # Safe Limits
        
        # --- LEFT HAND (The Trigger) ---
        # 3. Elbow (Left Hand Height controls Reach)
        # Hand Low = Retracted (45), Hand High = Extended (140)
        target_elbow = int((1.0 - left_hand[0].y) * 140)
        target_elbow = max(45, min(140, target_elbow))

        # 4. Gripper (Left Hand Fist)
        is_fist = self.is_fist(left_hand)
        grip_cmd = GestureType.CLOSED_HAND if is_fist else GestureType.OPEN_HAND

        # --- SMOOTHING ---
        self.smooth_shoulder = int(self.smooth_shoulder * (1 - self.alpha) + target_shoulder * self.alpha)
        self.smooth_elbow = int(self.smooth_elbow * (1 - self.alpha) + target_elbow * self.alpha)

        # Wrist is fixed for stability in bimanual
        wrist_angle = 180 

        debug = f"BI | Base:{base_cmd.name} Sh:{self.smooth_shoulder} El:{self.smooth_elbow} Grip:{is_fist}"
        return grip_cmd, base_cmd, self.smooth_shoulder, self.smooth_elbow, wrist_angle, debug

    def analyze_unimanual(self, landmarks):
        """
        UNIMANUAL MODE (Revised):
        - X/Y: Base & Shoulder (Same as before)
        - Z (Reach): Hand Size (Push towards camera to reach)
        - Grip: Fist
        """
        wrist = landmarks[0]
        # Use knuckles for stable size measurement (fingertips move too much)
        index_mcp = landmarks[5]  
        pinky_mcp = landmarks[17]
        
        # 1. Base (X-Axis) - Rate Control
        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        if wrist.x < 0.4: base_cmd = GestureType.MOVE_LEFT
        elif wrist.x > 0.6: base_cmd = GestureType.MOVE_RIGHT

        # 2. Shoulder (Y-Axis) - Position Control
        # Map Y (1.0=Bottom, 0.0=Top) to Angle (40-160)
        target_shoulder = int((1.0 - wrist.y) * 160)
        target_shoulder = max(40, min(160, target_shoulder))

        # 3. Elbow (Z-Axis) - DEPTH / HAND SIZE MAPPING
        # Calculate width of palm (Index Knuckle to Pinky Knuckle)
        # This is more stable than wrist-to-tip
        hand_width = math.hypot(index_mcp.x - pinky_mcp.x, index_mcp.y - pinky_mcp.y)
        
        # Calibration (Adjust these numbers based on your distance from camera!)
        # Check the 'Debug' text on screen to see your current 'Size'
        SIZE_FAR = 0.10   # Pulling back (Small hand)
        SIZE_CLOSE = 0.20 # Pushing forward (Big hand)
        
        # Map Size to Elbow Angle (45=Retracted, 120=Extended)
        # Normalize: (Value - Min) / (Max - Min)
        depth_ratio = (hand_width - SIZE_FAR) / (SIZE_CLOSE - SIZE_FAR)
        depth_ratio = max(0.0, min(1.0, depth_ratio)) # Clamp between 0 and 1
        
        target_elbow = int(45 + (depth_ratio * 75)) # 45 + range of 75 = 120 max

        # 4. Grip
        is_fist = self.is_fist(landmarks)
        grip_cmd = GestureType.CLOSED_HAND if is_fist else GestureType.OPEN_HAND

        # --- SMOOTHING ---
        self.smooth_shoulder = int(self.smooth_shoulder * (1 - self.alpha) + target_shoulder * self.alpha)
        self.smooth_elbow = int(self.smooth_elbow * (1 - self.alpha) + target_elbow * self.alpha)

        # Safety: Tuck wrist if spinning base
        wrist_angle = 90
        if base_cmd != GestureType.STOP_MOVE_HORIZONTAL:
            wrist_angle = 180

        # Updated Debug String to help you calibrate
        debug = f"UNI | Size:{hand_width:.2f} Sh:{self.smooth_shoulder} El:{self.smooth_elbow}"
        return grip_cmd, base_cmd, self.smooth_shoulder, self.smooth_elbow, wrist_angle, debug
    
    def is_fist(self, landmarks):
        # Check if fingers are folded
        # Compare fingertip (TIP) Y vs knuckle (PIP) Y
        # If tip is below knuckle (in screen coords where Y increases downwards), it's folded.
        fingers_folded = 0
        
        # Index Finger (Tip 8 vs PIP 6)
        if landmarks[8].y > landmarks[6].y: fingers_folded += 1
        # Middle Finger (Tip 12 vs PIP 10)
        if landmarks[12].y > landmarks[10].y: fingers_folded += 1
        # Ring Finger (Tip 16 vs PIP 14)
        if landmarks[16].y > landmarks[14].y: fingers_folded += 1
        # Pinky Finger (Tip 20 vs PIP 18)
        if landmarks[20].y > landmarks[18].y: fingers_folded += 1
        
        # If 3 or more fingers are folded, count as a fist
        return fingers_folded >= 3
    
    # Add this method inside your CVHandRecognizer class in cv_recognizer.py

    def is_peace_sign(self, landmarks):
        """
        Detects a 'Peace' or 'Victory' sign (Index & Middle UP, others DOWN).
        Used to toggle the Motor Lock.
        """
        # Finger Tips vs PIP (Knuckles)
        # Tip < PIP means "Up" (because Y increases downwards)
        
        index_up = landmarks[8].y < landmarks[6].y
        middle_up = landmarks[12].y < landmarks[10].y
        ring_down = landmarks[16].y > landmarks[14].y
        pinky_down = landmarks[20].y > landmarks[18].y
        
        # Strict check: Index/Middle UP, Ring/Pinky DOWN
        return index_up and middle_up and ring_down and pinky_down