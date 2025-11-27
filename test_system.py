#!/usr/bin/env python3
"""
Test script for keyboard control system
This script helps verify that the Arduino communication is working
"""

import serial
import time
import sys

def test_arduino_connection():
    """Test if Arduino is connected and responding"""
    try:
        # Try to connect to Arduino
        arduino = serial.Serial('/dev/cu.usbmodem2101', 9600, timeout=5)
        time.sleep(2)  # Wait for Arduino to reset
        
        print("Arduino connected successfully!")
        
        # Send test commands
        print("\n Testing commands...")
        test_commands = [('L', 'Left'), ('R', 'Right')]
        
        for cmd, name in test_commands:
            print(f"Sending {name} command: '{cmd}'")
            arduino.write(cmd.encode())
            time.sleep(1)
            
            # Read response if available
            if arduino.in_waiting > 0:
                response = arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore')
                print(f"Arduino response: {response.strip()}")
        
        arduino.close()
        print("\nAll tests passed! Your Arduino is ready.")
        return True
        
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
        print(" Make sure:")
        print("   - Arduino is connected via USB")
        print("   - Arduino sketch is uploaded")
        print("   - Correct port in keyboard_control.py")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False

def list_available_ports():
    """List all available serial ports"""
    import glob
    ports = glob.glob('/dev/cu.*')
    print("\n📋 Available serial ports:")
    for port in ports:
        print(f"   {port}")

if __name__ == "__main__":
    print(" Arduino Keyboard Control Test")
    print("=" * 40)
    
    # List available ports
    list_available_ports()
    
    # Test connection
    print("\n Testing Arduino connection...")
    if test_arduino_connection():
        print("\n Ready to run keyboard_control.py!")
        print("Run: python src/controller/keyboard_control.py")
    else:
        print("\n Fix the connection issues above, then try again.")