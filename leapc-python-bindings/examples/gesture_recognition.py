"""
Gesture Recognition System for Leap Motion
Detects various hand gestures including:
- Open/Closed hands
- Directional movements (left, right, up, down)
- Pinch gestures
- Peace sign, thumbs up, pointing
- Swipe gestures
"""

import leap
import time
import math
from collections import deque
from enum import Enum


class GestureType(Enum):
    UNKNOWN = "unknown"
    OPEN_HAND = "open_hand"
    CLOSED_FIST = "closed_fist"
    PINCH = "pinch"
    POINTING = "pointing"
    PEACE_SIGN = "peace_sign"
    THUMBS_UP = "thumbs_up"
    SWIPE_LEFT = "swipe_left"
    SWIPE_RIGHT = "swipe_right"
    SWIPE_UP = "swipe_up"
    SWIPE_DOWN = "swipe_down"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    MOVE_UP = "move_up"
    MOVE_DOWN = "move_down"


class GestureRecognizer:
    def __init__(self, history_size=10):
        self.history_size = history_size
        self.position_history = {}  # hand_id -> deque of positions
        self.gesture_callbacks = {}  # gesture_type -> callback function
        self.last_gestures = {}  # hand_id -> last detected gesture
        
        # Thresholds for gesture detection
        self.GRAB_THRESHOLD = 0.7  # For closed fist
        self.PINCH_THRESHOLD = 0.6  # For pinch gesture
        self.MOVEMENT_THRESHOLD = 20  # mm for movement detection
        self.SWIPE_VELOCITY_THRESHOLD = 100  # mm/s for swipe detection
        self.EXTENDED_FINGER_THRESHOLD = 0.5  # For finger extension
    
    def register_callback(self, gesture_type, callback):
        """Register a callback function for a specific gesture"""
        self.position_history[gesture_type] = callback
    
    def analyze_hand(self, hand):
        """Analyze a single hand and detect gestures"""
        hand_id = hand.id
        current_position = (hand.palm.position.x, hand.palm.position.y, hand.palm.position.z)
        
        # Update position history
        if hand_id not in self.position_history:
            self.position_history[hand_id] = deque(maxlen=self.history_size)
        self.position_history[hand_id].append(current_position)
        
        # Detect static gestures
        static_gesture = self._detect_static_gesture(hand)
        
        # Detect movement gestures
        movement_gesture = self._detect_movement_gesture(hand_id)
        
        # Return the most confident gesture
        detected_gesture = static_gesture if static_gesture != GestureType.UNKNOWN else movement_gesture
        
        # Only report if gesture changed
        if hand_id not in self.last_gestures or self.last_gestures[hand_id] != detected_gesture:
            self.last_gestures[hand_id] = detected_gesture
            return detected_gesture
        
        return None
    
    def _detect_static_gesture(self, hand):
        """Detect static hand poses"""
        
        # Get finger extension states
        fingers_extended = [
            hand.thumb.is_extended,
            hand.index.is_extended,
            hand.middle.is_extended,
            hand.ring.is_extended,
            hand.pinky.is_extended
        ]
        
        extended_count = sum(fingers_extended)
        
        # Closed fist: high grab strength, no extended fingers
        if hand.grab_strength > self.GRAB_THRESHOLD and extended_count <= 1:
            return GestureType.CLOSED_FIST
        
        # Open hand: low grab strength, most fingers extended
        if hand.grab_strength < 0.3 and extended_count >= 4:
            return GestureType.OPEN_HAND
        
        # Pinch: high pinch strength
        if hand.pinch_strength > self.PINCH_THRESHOLD:
            return GestureType.PINCH
        
        # Pointing: only index finger extended
        if fingers_extended == [False, True, False, False, False]:
            return GestureType.POINTING
        
        # Peace sign: index and middle fingers extended
        if fingers_extended == [False, True, True, False, False]:
            return GestureType.PEACE_SIGN
        
        # Thumbs up: only thumb extended, hand rotated appropriately
        if fingers_extended == [True, False, False, False, False]:
            # Check if palm is facing sideways (thumbs up orientation)
            palm_normal = hand.palm.normal
            if abs(palm_normal.y) > 0.7:  # Palm facing up/down
                return GestureType.THUMBS_UP
        
        return GestureType.UNKNOWN
    
    def _detect_movement_gesture(self, hand_id):
        """Detect movement-based gestures"""
        if hand_id not in self.position_history or len(self.position_history[hand_id]) < 3:
            return GestureType.UNKNOWN
        
        positions = list(self.position_history[hand_id])
        
        # Calculate movement vector from oldest to newest position
        start_pos = positions[0]
        end_pos = positions[-1]
        
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        dz = end_pos[2] - start_pos[2]
        
        # Calculate movement magnitude and direction
        horizontal_movement = abs(dx)
        vertical_movement = abs(dy)
        depth_movement = abs(dz)
        
        # Calculate velocity (approximate)
        time_span = len(positions) * 0.1  # Assuming ~10 FPS
        velocity_x = dx / time_span if time_span > 0 else 0
        velocity_y = dy / time_span if time_span > 0 else 0
        
        # Detect swipes (fast movements)
        if abs(velocity_x) > self.SWIPE_VELOCITY_THRESHOLD:
            return GestureType.SWIPE_RIGHT if velocity_x > 0 else GestureType.SWIPE_LEFT
        
        if abs(velocity_y) > self.SWIPE_VELOCITY_THRESHOLD:
            return GestureType.SWIPE_UP if velocity_y > 0 else GestureType.SWIPE_DOWN
        
        # Detect slower directional movements
        if horizontal_movement > self.MOVEMENT_THRESHOLD and horizontal_movement > vertical_movement:
            return GestureType.MOVE_RIGHT if dx > 0 else GestureType.MOVE_LEFT
        
        if vertical_movement > self.MOVEMENT_THRESHOLD and vertical_movement > horizontal_movement:
            return GestureType.MOVE_UP if dy > 0 else GestureType.MOVE_DOWN
        
        return GestureType.UNKNOWN
    
    def get_hand_info(self, hand):
        """Get detailed information about a hand for debugging"""
        fingers_extended = [
            hand.thumb.is_extended,
            hand.index.is_extended,
            hand.middle.is_extended,
            hand.ring.is_extended,
            hand.pinky.is_extended
        ]
        
        return {
            'id': hand.id,
            'type': 'left' if str(hand.type) == "HandType.Left" else 'right',
            'position': (hand.palm.position.x, hand.palm.position.y, hand.palm.position.z),
            'grab_strength': hand.grab_strength,
            'pinch_strength': hand.pinch_strength,
            'fingers_extended': fingers_extended,
            'extended_count': sum(fingers_extended),
            'confidence': hand.confidence
        }


class GestureListener(leap.Listener):
    def __init__(self):
        self.gesture_recognizer = GestureRecognizer()
        self.frame_count = 0
        
    def on_connection_event(self, event):
        print("✓ Connected to Leap Motion device")
        
    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()
        print(f"✓ Found device: {info.serial}")
        
    def on_tracking_event(self, event):
        self.frame_count += 1
        
        # Process every few frames to avoid too much output
        if self.frame_count % 5 != 0:
            return
            
        print(f"\n--- Frame {event.tracking_frame_id} with {len(event.hands)} hands ---")
        
        for hand in event.hands:
            # Get hand information
            hand_info = self.gesture_recognizer.get_hand_info(hand)
            
            # Detect gesture
            detected_gesture = self.gesture_recognizer.analyze_hand(hand)
            
            # Print hand info
            print(f"Hand {hand_info['id']} ({hand_info['type']}):")
            print(f"  Position: ({hand_info['position'][0]:.1f}, {hand_info['position'][1]:.1f}, {hand_info['position'][2]:.1f})")
            print(f"  Grab: {hand_info['grab_strength']:.2f}, Pinch: {hand_info['pinch_strength']:.2f}")
            print(f"  Fingers extended: {hand_info['extended_count']}/5 {hand_info['fingers_extended']}")
            
            # Print detected gesture
            if detected_gesture:
                print(f"  🎯 GESTURE DETECTED: {detected_gesture.value.upper()}")
                self.handle_gesture(detected_gesture, hand_info)
            else:
                current_gesture = self.gesture_recognizer.last_gestures.get(hand.id, GestureType.UNKNOWN)
                print(f"  Current gesture: {current_gesture.value}")
    
    def handle_gesture(self, gesture, hand_info):
        """Handle detected gestures - customize this for your application"""
        hand_type = hand_info['type']
        
        if gesture == GestureType.OPEN_HAND:
            print(f"    → {hand_type} hand is open - could trigger 'release' action")
            
        elif gesture == GestureType.CLOSED_FIST:
            print(f"    → {hand_type} hand is closed - could trigger 'grab' action")
            
        elif gesture == GestureType.PINCH:
            print(f"    → {hand_type} hand is pinching - could trigger 'select' action")
            
        elif gesture == GestureType.POINTING:
            print(f"    → {hand_type} hand is pointing - could trigger 'click' action")
            
        elif gesture == GestureType.PEACE_SIGN:
            print(f"    → {hand_type} hand shows peace sign - could trigger 'pause' action")
            
        elif gesture == GestureType.THUMBS_UP:
            print(f"    → {hand_type} hand thumbs up - could trigger 'like/approve' action")
            
        elif gesture in [GestureType.SWIPE_LEFT, GestureType.SWIPE_RIGHT, 
                        GestureType.SWIPE_UP, GestureType.SWIPE_DOWN]:
            direction = gesture.value.split('_')[1]
            print(f"    → {hand_type} hand swiped {direction} - could trigger navigation")
            
        elif gesture in [GestureType.MOVE_LEFT, GestureType.MOVE_RIGHT,
                        GestureType.MOVE_UP, GestureType.MOVE_DOWN]:
            direction = gesture.value.split('_')[1]
            print(f"    → {hand_type} hand moving {direction} - could control cursor/object")


def main():
    print("🚀 Starting Gesture Recognition System...")
    print("Available gestures:")
    print("  - Open hand / Closed fist")
    print("  - Pinch")
    print("  - Pointing (index finger)")
    print("  - Peace sign (index + middle)")
    print("  - Thumbs up")
    print("  - Directional movements and swipes")
    print("\nMove your hands in front of the Leap Motion sensor...")
    print("Press Ctrl+C to exit\n")
    
    gesture_listener = GestureListener()
    connection = leap.Connection()
    connection.add_listener(gesture_listener)
    
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        try:
            while True:
                time.sleep(0.1)  # 10 FPS processing
        except KeyboardInterrupt:
            print("\n👋 Gesture recognition stopped.")


if __name__ == "__main__":
    main()