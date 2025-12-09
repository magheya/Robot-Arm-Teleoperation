from utils import GestureType

class SimpleHandRecognizer:
    def __init__(self):
        self.last_grip_gesture = None
        self.last_horizontal_move_gesture = None
        self.last_vertical_move_gesture = None
        
        self.GRAB_THRESHOLD = 0.3
        
        # Movement settings (measured in millimeters from center)
        self.CENTER_DEADZONE = 30.0  # Hand must move 60mm away from center to trigger
        self.VERT_CENTER_DEADZONE = 20.0  # Hand must move 60mm away from center to trigger
        self.VERT_CENTER = 40.0
    
    def analyze_hand(self, hand):
        """
        Returns a tuple: (GripCommand, MoveCommand, DebugInfo)
        """
        debug_info = self._get_debug_info(hand)
        
        # --- 1. Analyze Gripper (Open/Close) ---
        current_grip = self._detect_grip(hand)
        grip_command = None

        # Only report if grip state changes
        if current_grip != self.last_grip_gesture:
            self.last_grip_gesture = current_grip
            grip_command = current_grip

        # --- 2. Analyze Movement (Left/Right) ---
        current_horizontal_move = self._detect_horizontal_movement(hand)
        horizontal_move_command = None

        # --- 2. Analyze Movement (Left/Right) ---
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
        fingers_extended = [
            hand.thumb.is_extended, hand.index.is_extended,
            hand.middle.is_extended, hand.ring.is_extended,
            hand.pinky.is_extended
        ]
        extended_count = sum(fingers_extended)
        
        if hand.grab_strength > self.GRAB_THRESHOLD and extended_count <= 1:
            return GestureType.CLOSED_HAND
        elif hand.grab_strength < 0.3 and extended_count >= 4:
            return GestureType.OPEN_HAND
        return None 

    def _detect_horizontal_movement(self, hand):
        # hand.palm.position.x is usually in millimeters
        # Negative X is Left, Positive X is Right
        x_pos = hand.palm.position.x
        
        if x_pos < -self.CENTER_DEADZONE:
            return GestureType.MOVE_LEFT
        elif x_pos > self.CENTER_DEADZONE:
            return GestureType.MOVE_RIGHT
        else:
            return GestureType.STOP_MOVE_HORIZONTAL
        
    def _detect_vertical_movement(self, hand):
        # hand.palm.position.x is usually in millimeters
        # Negative X is Left, Positive X is Right
        z_pos = hand.palm.position.z



        if z_pos > self.VERT_CENTER and abs(self.VERT_CENTER - z_pos) > self.VERT_CENTER_DEADZONE:
            return GestureType.MOVE_UP
        elif z_pos < self.VERT_CENTER and abs(self.VERT_CENTER - z_pos) > self.VERT_CENTER_DEADZONE:
            return GestureType.MOVE_DOWN
        else:
            return GestureType.STOP_MOVE_VERTICAL     
        
    def _get_position(self, hand):
        return (hand.palm.position.x, hand.palm.position.y)
    
    def _get_debug_info(self, hand):
        fingers_extended = [
            hand.thumb.is_extended, hand.index.is_extended,
            hand.middle.is_extended, hand.ring.is_extended,
            hand.pinky.is_extended
        ]
        return {
            'grab_strength': hand.grab_strength,
            'extended_count': sum(fingers_extended),
            'x_pos': hand.palm.position.x  # Added X position for debugging
        }
