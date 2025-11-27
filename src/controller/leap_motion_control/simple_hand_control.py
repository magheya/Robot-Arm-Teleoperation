"""
Simple Hand Gesture Recognition - Open/Closed Hand Only
Sends commands to Arduino to control robot arm
"""

import leap
import time
import serial
from enum import Enum


class GestureType(Enum):
    UNKNOWN = "unknown"
    OPEN_HAND = "open_hand"
    CLOSED_HAND = "closed_hand"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"


class SimpleHandRecognizer:
    def __init__(self):
        self.last_gestures = {}
        self.GRAB_THRESHOLD = 0.6  # Threshold for detecting closed hand
    
    def analyze_hand(self, hand):
        hand_id = hand.id
        
        # Detect open or closed hand
        detected_gesture = self._detect_hand_gesture(hand)
        
        # Only report gesture changes
        if hand_id not in self.last_gestures or self.last_gestures[hand_id] != detected_gesture:
            self.last_gestures[hand_id] = detected_gesture
            return detected_gesture, self._get_debug_info(hand)
        
        return None, None
    
    def _detect_hand_gesture(self, hand):
        """Detect if hand is open or closed"""
        fingers_extended = [
            hand.thumb.is_extended,
            hand.index.is_extended,
            hand.middle.is_extended,
            hand.ring.is_extended,
            hand.pinky.is_extended
        ]
        
        extended_count = sum(fingers_extended)
        
        # Closed hand: high grab strength and few extended fingers
        if hand.grab_strength > self.GRAB_THRESHOLD and extended_count <= 1:
            return GestureType.CLOSED_HAND
        
        # Open hand: low grab strength and most fingers extended
        elif hand.grab_strength < 0.3 and extended_count >= 4:
            return GestureType.OPEN_HAND
        
        return GestureType.UNKNOWN
    
    def _get_debug_info(self, hand):
        """Get debug information"""
        fingers_extended = [
            hand.thumb.is_extended,
            hand.index.is_extended,
            hand.middle.is_extended,
            hand.ring.is_extended,
            hand.pinky.is_extended
        ]
        
        return {
            'grab_strength': hand.grab_strength,
            'extended_count': sum(fingers_extended),
            'confidence': hand.confidence
        }


class RobotArmController:
    def __init__(self, port=None, baud_rate=9600):
        """Initialize Arduino connection with automatic port detection"""
        self.arduino = None
        self.port = port
        self.baud_rate = baud_rate
        
        if port is None:
            self.port = self._find_arduino_port()
        
        self._connect_to_arduino()
    
    def _find_arduino_port(self):
        """Automatically find Arduino port"""
        import glob
        import platform
        
        if platform.system() == "Darwin":  # macOS
            ports = glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/cu.usbserial*')
        elif platform.system() == "Linux":
            ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        elif platform.system() == "Windows":
            ports = ['COM%s' % (i + 1) for i in range(256)]
        else:
            ports = []
        
        # Filter existing ports
        available_ports = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                available_ports.append(port)
            except (OSError, serial.SerialException):
                pass
        
        if available_ports:
            print(f"📋 Available Arduino ports: {available_ports}")
            return available_ports[0]  # Use first available port
        else:
            print("❌ No Arduino ports found")
            return None
    
    def _connect_to_arduino(self):
        """Connect to Arduino with retry logic"""
        if not self.port:
            print("❌ No Arduino port specified")
            return
        
        max_retries = 3
        for attempt in range(max_retries):
            try:
                print(f"🔌 Connecting to Arduino on {self.port} (attempt {attempt + 1}/{max_retries})...")
                self.arduino = serial.Serial(self.port, self.baud_rate, timeout=2)
                time.sleep(3)  # Give Arduino more time to initialize
                
                # Test connection by sending a status command
                self.arduino.write(b'S')
                time.sleep(0.5)
                
                # Try to read response
                if self.arduino.in_waiting > 0:
                    response = self.arduino.read(self.arduino.in_waiting)
                    print(f"✅ Arduino connected! Response: {len(response)} bytes")
                else:
                    print(f"✅ Arduino connected on {self.port} at {self.baud_rate} baud")
                
                return
                
            except serial.SerialException as e:
                print(f"❌ Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_retries - 1:
                    print("   Retrying in 2 seconds...")
                    time.sleep(2)
                else:
                    print("❌ Failed to connect after all attempts")
                    self.arduino = None
            except Exception as e:
                print(f"❌ Unexpected error: {e}")
                self.arduino = None
                break
    
    def send_command(self, gesture):
        """Send gesture command to Arduino"""
        if not self.arduino:
            print("❌ No Arduino connection")
            return
        
        try:
            if gesture == GestureType.OPEN_HAND:
                self.arduino.write(b'O')  # Send 'O' for open
                print("📤 Sent: 'O' (Open hand)")
                
            elif gesture == GestureType.CLOSED_HAND:
                self.arduino.write(b'C')  # Send 'C' for closed
                print("📤 Sent: 'C' (Closed hand)")
                
        except Exception as e:
            print(f"❌ Error sending command: {e}")
    
    def close(self):
        """Close Arduino connection"""
        if self.arduino:
            self.arduino.close()


class HandGestureListener(leap.Listener):
    def __init__(self, arduino_port=None, arduino_baud=9600):
        self.gesture_recognizer = SimpleHandRecognizer()
        self.robot_controller = RobotArmController(port=arduino_port, baud_rate=arduino_baud)
        self.frame_count = 0
        
    def on_connection_event(self, event):
        print("✓ Connected to Leap Motion device")
        
    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()
        print(f"✓ Found Leap Motion device: {info.serial}")
        
    def on_tracking_event(self, event):
        self.frame_count += 1
        
        # Process every few frames to avoid spam
        if self.frame_count % 5 != 0:
            return
            
        if not event.hands:
            if self.frame_count % 50 == 0:
                print("👋 Place hand above Leap Motion sensor...")
            return
            
        # Process only the first hand detected
        hand = event.hands[0]
        hand_type = 'LEFT' if str(hand.type) == "HandType.Left" else 'RIGHT'
        
        detected_gesture, debug_info = self.gesture_recognizer.analyze_hand(hand)
        
        if detected_gesture and detected_gesture != GestureType.UNKNOWN:
            print(f"\n🎯 {hand_type} HAND: {detected_gesture.value.upper().replace('_', ' ')}")
            
            if debug_info:
                print(f"   ✋ Fingers extended: {debug_info['extended_count']}/5")
                print(f"   ✊ Grab strength: {debug_info['grab_strength']:.2f}")
                print(f"   📊 Confidence: {debug_info['confidence']:.2f}")
            
            # Send command to robot arm
            self.robot_controller.send_command(detected_gesture)
            self.handle_gesture(detected_gesture)
    
    def handle_gesture(self, gesture):
        """Handle detected gestures"""
        if gesture == GestureType.OPEN_HAND:
            print("   🤖 Robot Action: OPEN GRIPPER / RELEASE")
            
        elif gesture == GestureType.CLOSED_HAND:
            print("   🤖 Robot Action: CLOSE GRIPPER / GRAB")
    
    def cleanup(self):
        """Clean up connections"""
        self.robot_controller.close()


def main():
    print("🤖 Simple Robot Arm Gesture Control")
    print("=" * 50)
    
    # Specify Arduino connection explicitly
    arduino_port = '/dev/cu.usbmodem2101'  # Your specific Arduino port
    arduino_baud = 9600  # Match your Arduino code baud rate
    
    print(f"🔌 Attempting to connect to Arduino on {arduino_port}...")
    
    print("Supported gestures:")
    print("  ✋ OPEN HAND   → Open gripper / Release")
    print("  ✊ CLOSED HAND → Close gripper / Grab")
    print("\nInstructions:")
    print("1. Make sure Arduino is connected and running robot arm code")
    print("2. Close Arduino IDE Serial Monitor if open")
    print("3. Place your hand above the Leap Motion sensor")
    print("4. Open or close your hand to control the robot arm")
    print("\nCommands sent to Arduino:")
    print("  'O' = Open hand detected")
    print("  'C' = Closed hand detected")
    print("\nPress Ctrl+C to exit\n")
    
    # Create gesture listener with specific Arduino port
    gesture_listener = HandGestureListener(arduino_port, arduino_baud)
    connection = leap.Connection()
    connection.add_listener(gesture_listener)
    
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n👋 Stopping gesture control...")
            gesture_listener.cleanup()
            print("✓ Disconnected from devices")


if __name__ == "__main__":
    main()