import leap
import time
import serial
from enum import Enum

# --- GESTURE DEFINITIONS ---
class GestureType(Enum):
    UNKNOWN = "unknown"
    OPEN_HAND = "open_hand"
    CLOSED_HAND = "closed_hand"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    STOP_MOVE = "stop_move"

# --- LOGIC CLASS (Unchanged) ---
class SimpleHandRecognizer:
    def __init__(self):
        self.last_grip_gesture = None
        self.last_move_gesture = None
        self.GRAB_THRESHOLD = 0.6
        # The Zone: -60 to +60 is the "Stop" zone
        self.CENTER_DEADZONE = 60.0 
    
    def analyze_hand(self, hand):
        debug_info = self._get_debug_info(hand)
        
        # 1. Gripper Logic
        current_grip = self._detect_grip(hand)
        grip_command = None
        if current_grip != self.last_grip_gesture:
            self.last_grip_gesture = current_grip
            grip_command = current_grip

        # 2. Movement Logic
        current_move = self._detect_movement(hand)
        move_command = None
        if current_move != self.last_move_gesture:
            self.last_move_gesture = current_move
            move_command = current_move

        return grip_command, move_command, debug_info
    
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

    def _detect_movement(self, hand):
        x_pos = hand.palm.position.x
        if x_pos < -self.CENTER_DEADZONE:
            return GestureType.MOVE_LEFT
        elif x_pos > self.CENTER_DEADZONE:
            return GestureType.MOVE_RIGHT
        else:
            return GestureType.STOP_MOVE
    
    def _get_debug_info(self, hand):
        return {'x_pos': hand.palm.position.x}

# --- REAL HARDWARE CONTROLLER ---
class RobotArmController:
    def __init__(self, port=None):
        self.arduino = None
        try:
            # Simple connection attempt
            self.arduino = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)
            print(f"✓ Connected to Arduino on {port}")
        except Exception as e:
            print(f"✗ Connection failed: {e}")

    def send_command(self, gesture):
        if not self.arduino: return
        try:
            if gesture == GestureType.OPEN_HAND: self.arduino.write(b'O')
            elif gesture == GestureType.CLOSED_HAND: self.arduino.write(b'C')
            elif gesture == GestureType.MOVE_LEFT: self.arduino.write(b'L')
            elif gesture == GestureType.MOVE_RIGHT: self.arduino.write(b'R')
            elif gesture == GestureType.STOP_MOVE: self.arduino.write(b'X')
        except: pass
            
    def close(self):
        if self.arduino: self.arduino.close()

# --- NEW: MOCK CONTROLLER (For Testing) ---
class MockRobotController:
    def __init__(self):
        print("⚡ SIMULATION MODE: No Arduino connected.")
        print("   Watching for virtual commands...")

    def send_command(self, gesture):
        # This simulates what the robot WOULD do
        if gesture == GestureType.MOVE_LEFT:
            print("⬅️  MOTOR: Rotating LEFT  (Sent 'L')")
        elif gesture == GestureType.MOVE_RIGHT:
            print("➡️  MOTOR: Rotating RIGHT (Sent 'R')")
        elif gesture == GestureType.STOP_MOVE:
            print("🛑 MOTOR: Stopped        (Sent 'X')")
        elif gesture == GestureType.OPEN_HAND:
            print("✋ GRIPPER: Released      (Sent 'O')")
        elif gesture == GestureType.CLOSED_HAND:
            print("✊ GRIPPER: Grabbed       (Sent 'C')")

    def close(self):
        print("End simulation.")

# --- LISTENER ---
class HandGestureListener(leap.Listener):
    def __init__(self, arduino_port=None, use_simulation=False):
        self.recognizer = SimpleHandRecognizer()
        
        # DECIDE WHICH CONTROLLER TO USE
        if use_simulation:
            self.controller = MockRobotController()
        else:
            self.controller = RobotArmController(port=arduino_port)
            
        self.frame_count = 0
        
    def on_tracking_event(self, event):
        self.frame_count += 1
        if self.frame_count % 3 != 0: return 
            
        if not event.hands: return
        hand = event.hands[0]
        
        grip_cmd, move_cmd, debug = self.recognizer.analyze_hand(hand)
        
        if grip_cmd: self.controller.send_command(grip_cmd)
        if move_cmd: self.controller.send_command(move_cmd)

        # Visual Dashboard for position
        self._print_position_bar(debug['x_pos'])

    def _print_position_bar(self, x_pos):
        # Visualizing the Deadzone
        # Zone:  Left <--- -60 --- [DEADZONE] --- +60 ---> Right
        
        status = "|| DEADZONE ||"
        if x_pos < -60: status = "<< LEFT <<    "
        elif x_pos > 60: status = "    >> RIGHT >>"
        
        # Print overwriting the same line (creates a dashboard effect)
        print(f"\rPosition X: {x_pos:6.1f} | Status: {status}", end="", flush=True)

    def cleanup(self):
        self.controller.close()

# --- MAIN ---
def main():
    print("=== Robot Arm Control ===")
    
    # *** SET THIS TO TRUE TO TEST WITHOUT ARDUINO ***
    SIMULATION_MODE = True  
    
    arduino_port = '/dev/cu.usbmodem2101' 
    
    listener = HandGestureListener(arduino_port, use_simulation=SIMULATION_MODE)
    connection = leap.Connection()
    connection.add_listener(listener)
    
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        try:
            while True: time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nExiting...")
            listener.cleanup()

if __name__ == "__main__":
    main()