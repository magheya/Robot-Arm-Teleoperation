#!/usr/bin/env python3
"""
Direct servo test - manually send L/R commands to Arduino
"""
import serial
import time

def test_servo_direct():
    try:
        arduino = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=2)
        time.sleep(2)
        
        print("🔧 Direct Servo Test")
        print("Sending commands directly to Arduino...")
        
        commands = [
            ('R', 'Right'), ('R', 'Right'), ('R', 'Right'),
            ('L', 'Left'), ('L', 'Left'), ('L', 'Left'),
            ('R', 'Right')
        ]
        
        for cmd, name in commands:
            print(f"Sending {name} command: {cmd}")
            arduino.write(cmd.encode())
            time.sleep(1)
            
            # Read any response
            while arduino.in_waiting > 0:
                response = arduino.readline().decode().strip()
                if response:
                    print(f"Arduino: {response}")
        
        arduino.close()
        print("\nTest complete. Did you see the servo move?")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_servo_direct()