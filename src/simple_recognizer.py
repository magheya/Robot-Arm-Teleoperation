from utils import GestureType

class SimpleHandRecognizer:
    def __init__(self):
        self.last_grip_gesture = None
        self.last_horizontal_move_gesture = None
        self.last_vertical_move_gesture = None
        
        self.GRAB_THRESHOLD = 0.3
        
        # Movement settings (measured in millimeters from center)
        self.CENTER_DEADZONE = 30.0  # Hand must move 30mm away from center to trigger
        self.VERT_CENTER_DEADZONE = 20.0  # Hand must move 20mm away from vertical center
        self.VERT_CENTER = 40.0
    
    def analyze_hand(self, hand):
        """
        Returns a tuple: (GripCommand, HorizontalMoveCommand, VerticalMoveCommand, DebugInfo)
        """
        debug_info = self._get_debug_info(hand)
        
        # --- 1. Analyze Gripper (Open/Close) ---
        current_grip = self._detect_grip(hand)
        grip_command = None

        # Only report if grip state changes
        if current_grip != self.last_grip_gesture:
            self.last_grip_gesture = current_grip
            grip_command = current_grip

        # --- 2. Analyze Horizontal Movement (Left/Right) ---
        current_horizontal_move = self._detect_horizontal_movement(hand)
        horizontal_move_command = None
        
        # --- 3. Analyze Vertical Movement (Up/Down) ---
        current_vertical_move = self._detect_vertical_movement(hand)
        vertical_move_command = None        
        
        # Only report if movement direction changes
        if current_horizontal_move != self.last_horizontal_move_gesture:
            self.last_horizontal_move_gesture = current_horizontal_move
            horizontal_move_command = current_horizontal_move

        if current_vertical_move != self.last_vertical_move_gesture:
            self.last_vertical_move_gesture = current_vertical_move
            vertical_move_command = current_vertical_move        

        return grip_command, horizontal_move_command, vertical_move_command, debug_info
    
    def _detect_grip(self, hand):
        # This logic seems to be based on an older Leap SDK.
        # You might need to adjust based on your Leap SDK version.
        # Assuming `hand.index` etc. are valid finger objects.
        fingers_extended = [
            f.is_extended for f in hand.fingers
        ]
        extended_count = sum(fingers_extended)
        
        if hand.grab_strength > (1.0 - self.GRAB_THRESHOLD) and extended_count <= 1:
            return GestureType.CLOSED_HAND
        elif hand.grab_strength < 0.3 and extended_count >= 4:
            return GestureType.OPEN_HAND
        return None 

    def _detect_horizontal_movement(self, hand):
        # Negative X is Left, Positive X is Right
        x_pos = hand.palm.position.x
        
        if x_pos < -self.CENTER_DEADZONE:
            return GestureType.MOVE_LEFT
        elif x_pos > self.CENTER_DEADZONE:
            return GestureType.MOVE_RIGHT
        else:
            return GestureType.STOP_MOVE_HORIZONTAL
        
    def _detect_vertical_movement(self, hand):
        # Using Z-axis for forward/backward, which we map to Up/Down
        z_pos = hand.palm.position.z

        if z_pos > self.VERT_CENTER and abs(self.VERT_CENTER - z_pos) > self.VERT_CENTER_DEADZONE:
            return GestureType.MOVE_UP # Moving hand away from screen
        elif z_pos < self.VERT_CENTER and abs(self.VERT_CENTER - z_pos) > self.VERT_CENTER_DEADZONE:
            return GestureType.MOVE_DOWN # Moving hand towards screen
        else:
            return GestureType.STOP_MOVE_VERTICAL     
        
    def _get_position(self, hand):
        return (hand.palm.position.x, hand.palm.position.y, hand.palm.position.z)
    
    def _get_debug_info(self, hand):
        x, y, z = self._get_position(hand)
        extended_count = sum([f.is_extended for f in hand.fingers])
        
        return f"Pos:({x:.1f}, {y:.1f}, {z:.1f}), Grab:{hand.grab_strength:.2f}, Fingers:{extended_count}"