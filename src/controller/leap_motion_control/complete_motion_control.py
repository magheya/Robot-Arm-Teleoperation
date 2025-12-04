import leap
import time
import serial
from enum import Enum

class GestureType(Enum):
    UNKNOWN = "unknown"
    OPEN_HAND = "open_hand"
    CLOSED_HAND = "closed_hand"
    # New States for Stepper
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    STOP_MOVE = "stop_move"

class SimpleHandRecognizer:
    def __init__(self):
        self.last_grip_gesture = None
        self.last_move_gesture = None
        
        self.GRAB_THRESHOLD = 0.6
        
        # Movement settings (measured in millimeters from center)
        self.CENTER_DEADZONE = 60.0  # Hand must move 60mm away from center to trigger
    
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
        current_move = self._detect_movement(hand)
        move_command = None
        
        # Only report if movement direction changes
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
        # hand.palm.position.x is usually in millimeters
        # Negative X is Left, Positive X is Right
        x_pos = hand.palm.position.x
        
        if x_pos < -self.CENTER_DEADZONE:
            return GestureType.MOVE_LEFT
        elif x_pos > self.CENTER_DEADZONE:
            return GestureType.MOVE_RIGHT
        else:
            return GestureType.STOP_MOVE
    
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

class RobotArmController:
    def __init__(self, port=None, baud_rate=9600):
        self.arduino = None
        self.port = port
        self.baud_rate = baud_rate
        if port is None:
            self.port = self._find_arduino_port()
        self._connect_to_arduino()
    
    def _find_arduino_port(self):
        import glob, platform
        if platform.system() == "Darwin": ports = glob.glob('/dev/cu.usbmodem*')
        elif platform.system() == "Linux": ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        elif platform.system() == "Windows": ports = ['COM%s' % (i + 1) for i in range(256)]
        else: ports = []
        
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                return port
            except: pass
        return None
    
    def _connect_to_arduino(self):
        if not self.port: return
        try:
            self.arduino = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)
            print(f"✓ Connected to Arduino on {self.port}")
        except Exception as e:
            print(f"✗ Connection failed: {e}")

    def send_command(self, gesture):
        if not self.arduino: return
        try:
            # Gripper Commands
            if gesture == GestureType.OPEN_HAND:
                self.arduino.write(b'O')
                print(">> Sent: Open Gripper")
            elif gesture == GestureType.CLOSED_HAND:
                self.arduino.write(b'C')
                print(">> Sent: Close Gripper")
            
            # Stepper Commands
            elif gesture == GestureType.MOVE_LEFT:
                self.arduino.write(b'L')
                print("<< Sent: Rotate LEFT")
            elif gesture == GestureType.MOVE_RIGHT:
                self.arduino.write(b'R')
                print(">> Sent: Rotate RIGHT")
            elif gesture == GestureType.STOP_MOVE:
                self.arduino.write(b'X') # 'X' for Stop
                print("|| Sent: STOP Rotation")
                
        except Exception as e:
            print(f"Serial Error: {e}")
            
    def close(self):
        if self.arduino: self.arduino.close()

class HandGestureListener(leap.Listener):
    def __init__(self, arduino_port=None):
        self.recognizer = SimpleHandRecognizer()
        self.controller = RobotArmController(port=arduino_port)
        self.frame_count = 0
        
    def on_tracking_event(self, event):
        self.frame_count += 1
        if self.frame_count % 3 != 0: return # Limit processing rate
            
        if not event.hands: return
            
        hand = event.hands[0]
        
        # Get both gripper and movement commands
        grip_cmd, move_cmd, debug = self.recognizer.analyze_hand(hand)
        
        # If there is a change in grip, send it
        if grip_cmd:
            self.controller.send_command(grip_cmd)
            
        # If there is a change in movement direction, send it
        if move_cmd:
            self.controller.send_command(move_cmd)
            print(f"   Position X: {debug['x_pos']:.1f} (Deadzone: +/- 60)")

    def cleanup(self):
        self.controller.close()

def main():
    print("=== Robot Arm: Grip & Rotate Control ===")
    
    # UPDATE THIS PORT
    arduino_port = '/dev/cu.usbmodem2101' 
    
    listener = HandGestureListener(arduino_port)
    connection = leap.Connection()
    connection.add_listener(listener)
    
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        try:
            while True: time.sleep(0.1)
        except KeyboardInterrupt:
            listener.cleanup()

if __name__ == "__main__":
    main()