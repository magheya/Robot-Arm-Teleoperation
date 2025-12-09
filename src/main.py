import leap
import time
import serial
from enum import Enum
import commands
from simple_recognizer import SimpleHandRecognizer
from base_recognizer import BaseHandRecognizer
from utils import GestureType

class RobotArmController:
    def __init__(self, port=None, baud_rate=115200):
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
                commands.send_open_grip(self.arduino)
                print(">> Sent: Open Gripper")
            elif gesture == GestureType.CLOSED_HAND:
                commands.send_close_grip(self.arduino)
                print(">> Sent: Close Gripper")

            elif gesture == GestureType.MOVE_UP:
                commands.send_move_up(self.arduino)
                print("^^ Sent: UP")  
            elif gesture == GestureType.MOVE_DOWN:
                print("-- Sent: DOWN")
                commands.send_move_down(self.arduino) 
            elif gesture == GestureType.STOP_MOVE_VERTICAL:
                print("= Sent: STOP VERITICAL")   
                commands.send_stop_vertical(self.arduino)                     
            
            # Stepper Commands
            elif gesture == GestureType.MOVE_LEFT:
                commands.send_rotate_left(self.arduino)
                print("<< Sent: Rotate LEFT")
            elif gesture == GestureType.MOVE_RIGHT:
                commands.send_rotate_right(self.arduino)
                print(">> Sent: Rotate RIGHT")
            elif gesture == GestureType.STOP_MOVE_HORIZONTAL:
                commands.send_rotate_stop(self.arduino)
                print("|| Sent: STOP Rotation")                                   
                
        except Exception as e:
            print(f"Serial Error: {e}")
            
    def close(self):
        if self.arduino: self.arduino.close()

class HandGestureListener(leap.Listener):
    def __init__(self, arduino_port=None):
        self.recognizer = SimpleHandRecognizer() # BaseHandRecognizer()
        self.controller = RobotArmController(port=arduino_port)
        self.frame_count = 0
        
    def on_tracking_event(self, event):
        self.frame_count += 1
        if self.frame_count % 3 != 0: return # Limit processing rate
            
        if not event.hands: return
            
        hand = event.hands[0]
        
        # Get both gripper and movement commands
        grip_cmd, horiontal_move_cmd, vertical_move_cmd, debug = self.recognizer.analyze_hand(hand)
        
        # If there is a change in grip, send it
        if grip_cmd:
            self.controller.send_command(grip_cmd)
            
        # If there is a change in movement direction, send it
        if horiontal_move_cmd:
            self.controller.send_command(horiontal_move_cmd)
        if vertical_move_cmd:
            self.controller.send_command(vertical_move_cmd)    

    def cleanup(self):
        self.controller.close()

def main():
    print("=== Robot Arm: Grip & Rotate Control ===")
    
    # UPDATE THIS PORT
    arduino_port = '/dev/cu.usbmodem21301' 
    
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