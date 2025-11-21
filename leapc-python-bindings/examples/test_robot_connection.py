"""
Simple test script for 2-Servo Gripper Controller
Tests basic Arduino communication and gripper movements
"""

import serial
import time
import serial.tools.list_ports


def find_arduino_ports():
    """Find all potential Arduino ports"""
    ports = serial.tools.list_ports.comports()
    arduino_ports = []
    
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device} - {port.description}")
        if any(keyword in port.description.lower() for keyword in 
               ['arduino', 'ch340', 'cp210', 'ftdi', 'usb']):
            arduino_ports.append(port.device)
    
    return arduino_ports


def test_gripper_connection(port):
    """Test basic connection to gripper"""
    try:
        print(f"Connecting to {port}...")
        arduino = serial.Serial(port, 9600, timeout=2)
        time.sleep(3)  # Wait for Arduino to initialize
        
        # Read initial messages
        print("Arduino startup messages:")
        for _ in range(15):
            if arduino.in_waiting:
                msg = arduino.readline().decode().strip()
                if msg:
                    print(f"  {msg}")
            time.sleep(0.1)
        
        return arduino
        
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None


def send_test_command(arduino, command):
    """Send a test command and read response"""
    print(f"\nSending: {command}")
    arduino.write((command + '\n').encode())
    
    # Wait for response
    start_time = time.time()
    while time.time() - start_time < 2:
        if arduino.in_waiting:
            response = arduino.readline().decode().strip()
            if response:
                print(f"Response: {response}")
        time.sleep(0.1)


def send_char_command(arduino, char):
    """Send single character command"""
    print(f"\nSending char: {char}")
    arduino.write(char.encode())
    
    # Wait for response
    time.sleep(0.5)
    while arduino.in_waiting:
        response = arduino.readline().decode().strip()
        if response:
            print(f"Response: {response}")


def run_basic_gripper_test(arduino):
    """Run basic gripper movement tests"""
    print("\n🤏 Starting Basic Gripper Tests")
    print("=" * 40)
    
    # Test original single character commands
    print("Testing single character commands (your original method):")
    char_tests = [
        ('h', "Half position"),
        ('o', "Open gripper"),
        ('c', "Close gripper"),
        ('h', "Back to half position"),
    ]
    
    for char, description in char_tests:
        print(f"\n{description}:")
        send_char_command(arduino, char)
        time.sleep(2)
    
    print("\n" + "=" * 40)
    print("Testing new command format:")
    
    # Test new command format
    command_tests = [
        ("HALF", "Half position"),
        ("OPEN", "Open gripper"),
        ("CLOSE", "Close gripper"),
        ("STATUS", "Get status"),
    ]
    
    for command, description in command_tests:
        print(f"\n{description}:")
        send_test_command(arduino, command)
        time.sleep(2)
    
    print("\n✅ Basic gripper tests completed!")


def run_gesture_test(arduino):
    """Test gesture commands"""
    print("\n🎯 Testing Gesture Commands")
    print("=" * 40)
    
    gesture_tests = [
        ("GESTURE:pinch", "Pinch gesture → Half position"),
        ("GESTURE:open_hand", "Open hand → Open gripper"),
        ("GESTURE:closed_fist", "Closed fist → Close gripper"),
        ("GESTURE:pointing", "Pointing → Acknowledged (no action)"),
        ("GESTURE:peace_sign", "Peace sign → Acknowledged (no action)"),
    ]
    
    for command, description in gesture_tests:
        print(f"\n{description}:")
        send_test_command(arduino, command)
        time.sleep(3)
    
    print("\n✅ Gesture tests completed!")


def interactive_test(arduino):
    """Interactive testing mode"""
    print("\n🎮 Interactive Test Mode")
    print("=" * 40)
    print("Commands:")
    print("  Single chars: o=open, c=close, h=half")
    print("  Full commands: OPEN, CLOSE, HALF, STATUS")
    print("  Gestures: GESTURE:open_hand, GESTURE:closed_fist, GESTURE:pinch")
    print("  Type 'quit' to exit")
    
    while True:
        try:
            cmd = input("\nEnter command: ").strip()
            
            if cmd.lower() in ['quit', 'exit', 'q']:
                break
            elif len(cmd) == 1 and cmd.lower() in ['o', 'c', 'h']:
                send_char_command(arduino, cmd.lower())
            elif cmd:
                send_test_command(arduino, cmd)
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    print("🔧 Gripper Connection Test")
    print("=" * 30)
    
    # Find Arduino ports
    arduino_ports = find_arduino_ports()
    
    if not arduino_ports:
        port = input("\nNo Arduino detected. Enter port manually: ").strip()
        arduino_ports = [port]
    
    # Try connecting to Arduino
    arduino = None
    for port in arduino_ports:
        arduino = test_gripper_connection(port)
        if arduino:
            break
    
    if not arduino:
        print("❌ Could not connect to Arduino")
        print("\nTroubleshooting:")
        print("1. Make sure Arduino is connected via USB")
        print("2. Upload the robot_arm_controller.ino sketch to Arduino")
        print("3. Close Arduino IDE Serial Monitor")
        print("4. Check the port name")
        print("5. Make sure servos are connected to pins 9 and 10")
        return
    
    print("✅ Arduino connected successfully!")
    
    try:
        while True:
            print("\n🎯 Test Options:")
            print("1. Basic gripper movement test")
            print("2. Gesture command test")
            print("3. Interactive test mode")
            print("4. Get gripper status")
            print("5. Quit")
            
            choice = input("Choose test (1-5): ").strip()
            
            if choice == '1':
                run_basic_gripper_test(arduino)
            elif choice == '2':
                run_gesture_test(arduino)
            elif choice == '3':
                interactive_test(arduino)
            elif choice == '4':
                send_test_command(arduino, "STATUS")
            elif choice == '5':
                break
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n\n🛑 Test stopped")
    
    finally:
        arduino.close()
        print("👋 Disconnected from gripper")


if __name__ == "__main__":
    main()