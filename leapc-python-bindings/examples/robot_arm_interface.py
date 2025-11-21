"""
Simple 2-Servo Gripper Controller Interface for Leap Motion Gesture Recognition
Connects the gesture recognition system to Arduino-controlled gripper
"""

import serial
import time
import threading
from queue import Queue
import leap
from gesture_recognition import GestureRecognizer, GestureType


class GripperController:
    def __init__(self, port='/dev/tty.usbmodem1101', baudrate=9600):
        """
        Initialize gripper controller
        
        Args:
            port: Arduino serial port
            baudrate: Serial communication speed
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.connected = False
        self.last_gesture_time = {}
        self.gesture_cooldown = 1.0  # seconds between same gesture
        
        # Connect to Arduino
        self.connect()
        
    def connect(self):
        """Connect to Arduino via serial"""
        try:
            self.serial_connection = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=1,
                write_timeout=1
            )
            time.sleep(2)  # Wait for Arduino to initialize
            self.connected = True
            print(f"✓ Connected to gripper on {self.port}")
            
            # Read initial Arduino messages
            for _ in range(15):
                if self.serial_connection.in_waiting:
                    response = self.serial_connection.readline().decode().strip()
                    print(f"Arduino: {response}")
                time.sleep(0.1)
                
        except serial.SerialException as e:
            print(f"❌ Failed to connect to gripper: {e}")
            print("Make sure:")
            print("1. Arduino is connected")
            print("2. Correct port is specified")
            print("3. Arduino IDE Serial Monitor is closed")
            self.connected = False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        if self.connected and self.serial_connection:
            self.serial_connection.close()
            self.connected = False
            print("Disconnected from gripper")
    
    def send_command(self, command):
        """Send command to Arduino"""
        if not self.connected:
            print(f"Cannot send command - not connected: {command}")
            return False
            
        try:
            full_command = command + '\n'
            self.serial_connection.write(full_command.encode())
            print(f"Sent: {command}")
            
            # Read response with timeout
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if self.serial_connection.in_waiting:
                    response = self.serial_connection.readline().decode().strip()
                    if response:
                        print(f"Arduino: {response}")
                        break
                time.sleep(0.01)
            return True
            
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def send_gesture(self, gesture_type):
        """Send gesture command to gripper"""
        current_time = time.time()
        gesture_key = str(gesture_type)
        
        # Check cooldown
        if gesture_key in self.last_gesture_time:
            if current_time - self.last_gesture_time[gesture_key] < self.gesture_cooldown:
                return False
        
        self.last_gesture_time[gesture_key] = current_time
        
        # Map gesture to gripper command
        command = f"GESTURE:{gesture_type.value}"
        return self.send_command(command)
    
    def open_gripper(self):
        """Open gripper"""
        return self.send_command("OPEN")
    
    def close_gripper(self):
        """Close gripper"""
        return self.send_command("CLOSE")
    
    def half_gripper(self):
        """Move gripper to half position"""
        return self.send_command("HALF")
    
    def get_status(self):
        """Get gripper status"""
        return self.send_command("STATUS")
    
    # Also support original single character commands
    def send_char(self, char):
        """Send single character command (o/c/h)"""
        if char not in ['o', 'c', 'h']:
            print(f"Invalid character command: {char}")
            return False
        
        try:
            self.serial_connection.write(char.encode())
            print(f"Sent char: {char}")
            return True
        except Exception as e:
            print(f"Error sending char: {e}")
            return False


class GripperGestureListener(leap.Listener):
    def __init__(self, gripper_controller, control_mode='gestures'):
        """
        Initialize gesture listener for gripper control
        
        Args:
            gripper_controller: GripperController instance
            control_mode: 'gestures' or 'grab_strength'
        """
        self.gripper = gripper_controller
        self.gesture_recognizer = GestureRecognizer()
        self.control_mode = control_mode
        self.frame_count = 0
        self.last_grab_state = None
        
        print(f"🤖 Gripper Gesture Control Active")
        print(f"Control Mode: {control_mode}")
        
        if control_mode == 'gestures':
            print("Gesture Controls:")
            print("  ✊ Closed Fist → Close gripper")
            print("  ✋ Open Hand → Open gripper")
            print("  🤏 Pinch → Half position")
            print("  (Other gestures acknowledged but no action)")
        else:
            print("Grab Strength Control:")
            print("  Low grab strength → Open gripper")
            print("  Medium grab strength → Half position")
            print("  High grab strength → Close gripper")
        
    def on_connection_event(self, event):
        print("✓ Leap Motion connected")
        
    def on_device_event(self, event):
        print("✓ Leap Motion device ready")
        
    def on_tracking_event(self, event):
        self.frame_count += 1
        
        # Process every few frames
        if self.frame_count % 5 != 0:
            return
            
        if not self.gripper.connected:
            return
            
        for hand in event.hands:
            hand_type = 'left' if str(hand.type) == "HandType.Left" else 'right'
            
            if self.control_mode == 'gestures':
                # Gesture-based control
                detected_gesture = self.gesture_recognizer.analyze_hand(hand)
                if detected_gesture:
                    print(f"\n🎯 {hand_type.upper()} HAND: {detected_gesture.value.upper()}")
                    success = self.gripper.send_gesture(detected_gesture)
                    if success:
                        print(f"   ✓ Gripper executing gesture")
                    else:
                        print(f"   ⏸ Gesture on cooldown")
                        
            elif self.control_mode == 'grab_strength':
                # Grab strength control (use right hand primarily)
                if hand_type == 'right':
                    grab = hand.grab_strength
                    
                    # Determine gripper state based on grab strength
                    if grab < 0.2:
                        new_state = 'open'
                    elif grab < 0.6:
                        new_state = 'half'
                    else:
                        new_state = 'closed'
                    
                    # Only send command if state changed
                    if self.last_grab_state != new_state:
                        print(f"Grab strength: {grab:.2f} → {new_state}")
                        
                        if new_state == 'open':
                            self.gripper.open_gripper()
                        elif new_state == 'half':
                            self.gripper.half_gripper()
                        else:
                            self.gripper.close_gripper()
                        
                        self.last_grab_state = new_state


def find_arduino_port():
    """Helper function to find Arduino port automatically"""
    import serial.tools.list_ports
    
    print("Scanning for Arduino ports...")
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        # Look for common Arduino identifiers
        if any(keyword in port.description.lower() for keyword in 
               ['arduino', 'ch340', 'cp210', 'ftdi', 'usb']):
            print(f"Found potential Arduino: {port.device} - {port.description}")
            return port.device
    
    # Fallback - show all available ports
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")
    
    return None


def main():
    print("🤏 Gripper Gesture Control System")
    print("=================================")
    
    # Find Arduino port
    arduino_port = find_arduino_port()
    if not arduino_port:
        arduino_port = input("Enter Arduino port (e.g., /dev/tty.usbmodem1101): ").strip()
    
    # Choose control mode
    print("\nControl Modes:")
    print("1. Gesture Control - Specific gestures control gripper")
    print("2. Grab Strength - Hand grip strength controls gripper")
    print("3. Manual Test - Test gripper with keyboard commands")
    mode_choice = input("Choose mode (1-3): ").strip()
    
    if mode_choice == '3':
        # Manual test mode
        gripper = GripperController(port=arduino_port)
        if not gripper.connected:
            return
        
        print("\nManual Test Mode")
        print("Commands: o=open, c=close, h=half, s=status, q=quit")
        
        try:
            while True:
                cmd = input("Enter command: ").strip().lower()
                if cmd == 'q':
                    break
                elif cmd == 'o':
                    gripper.send_char('o')
                elif cmd == 'c':
                    gripper.send_char('c')
                elif cmd == 'h':
                    gripper.send_char('h')
                elif cmd == 's':
                    gripper.get_status()
                else:
                    print("Invalid command")
        except KeyboardInterrupt:
            pass
        
        gripper.disconnect()
        return
    
    control_mode = 'gestures' if mode_choice == '1' else 'grab_strength'
    
    # Initialize gripper controller
    gripper = GripperController(port=arduino_port)
    
    if not gripper.connected:
        print("❌ Cannot start - gripper not connected")
        return
    
    # Test gripper movement
    print("\nTesting gripper...")
    gripper.half_gripper()
    time.sleep(1)
    
    # Start gesture recognition
    listener = GripperGestureListener(gripper, control_mode)
    connection = leap.Connection()
    connection.add_listener(listener)
    
    try:
        with connection.open():
            connection.set_tracking_mode(leap.TrackingMode.Desktop)
            print(f"\n🎮 Gesture control active! Move your hands above the Leap Motion sensor.")
            print("Press Ctrl+C to stop")
            
            while True:
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\n🛑 Stopping gripper control...")
        gripper.disconnect()
        print("👋 Gripper control stopped")


if __name__ == "__main__":
    main()