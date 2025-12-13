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
        UNIMANUAL MODE:
        One hand does everything. Uses Tilt for Elbow control.
        """
        wrist = landmarks[0]
        middle_finger_tip = landmarks[12]

        # 1. Base (X-Axis)
        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        if wrist.x < 0.4: base_cmd = GestureType.MOVE_LEFT
        elif wrist.x > 0.6: base_cmd = GestureType.MOVE_RIGHT

        # 2. Shoulder (Y-Axis)
        target_shoulder = int((1.0 - wrist.y) * 160)
        target_shoulder = max(40, min(160, target_shoulder))

        # 3. Elbow (Hand Pitch / Tilt)
        # Measure vertical distance between Wrist and Middle Finger Tip
        # If Tip is ABOVE Wrist (High Y diff) -> Hand is Upright -> Retract
        # If Tip is LEVEL with Wrist (Low Y diff) -> Hand is Flat -> Extend
        tilt_diff = wrist.y - middle_finger_tip.y # Positive if fingers point up
        
        if tilt_diff > 0.15: 
            # Fingers pointing UP -> Pull Back
            target_elbow = 45 
        elif tilt_diff < 0.05:
            # Fingers pointing Forward/Down -> Reach Out
            target_elbow = 120
        else:
            # Neutral -> Hold current (or middle)
            target_elbow = self.smooth_elbow

        # 4. Grip
        is_fist = self.is_fist(landmarks)
        grip_cmd = GestureType.CLOSED_HAND if is_fist else GestureType.OPEN_HAND

        # --- SMOOTHING ---
        self.smooth_shoulder = int(self.smooth_shoulder * (1 - self.alpha) + target_shoulder * self.alpha)
        self.smooth_elbow = int(self.smooth_elbow * (1 - self.alpha) + target_elbow * self.alpha)

        # Auto-Tuck Wrist if moving base (Safety Feature)
        wrist_angle = 90
        if base_cmd != GestureType.STOP_MOVE_HORIZONTAL:
            wrist_angle = 180 # Tuck in while spinning

        debug = f"UNI | Tilt:{tilt_diff:.2f} Sh:{self.smooth_shoulder} El:{self.smooth_elbow}"
        return grip_cmd, base_cmd, self.smooth_shoulder, self.smooth_elbow, wrist_angle, debug

    def is_fist(self, landmarks):
        # Simple check: Are finger tips below finger PIP joints?
        fingers_folded = 0
        # Index (8 vs 6), Middle (12 vs 10), Ring (16 vs 14), Pinky (20 vs 18)
        if landmarks[8].y > landmarks[6].y: fingers_folded += 1
        if landmarks[12].y > landmarks[10].y: fingers_folded += 1
        if landmarks[16].y > landmarks[14].y: fingers_folded += 1
        if landmarks[20].y > landmarks[18].y: fingers_folded += 1
        return fingers_folded >= 3