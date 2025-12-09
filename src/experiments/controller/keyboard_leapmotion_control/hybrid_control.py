"""
Enhanced Hybrid Control System - Gesture & Key Switching
- Peace sign gesture switches from Leap Motion to Keyboard
- TAB key switches from Keyboard to Leap Motion
- No Arduino connection conflicts
"""

import subprocess
import threading
import time
from enum import Enum
from pynput import keyboard
import os
import leap

class ControlMode(Enum):
    LEAP_MOTION = "leap_motion"
    KEYBOARD = "keyboard"

class HybridController:
    def __init__(self):
        self.current_mode = ControlMode.LEAP_MOTION
        self.leap_process = None
        self.keyboard_process = None
        self.running = True
        
        # Leap Motion components for gesture switching ONLY (no Arduino connection)
        self.leap_connection = None
        self.gesture_listener = None
        self.leap_thread = None
        
        print("🎮 Enhanced Hybrid Controller - Gesture & Key Switching")
        self._print_instructions()
        
        # Start with Leap Motion mode
        self.switch_to_leap_motion()
        
        # Start key monitoring for TAB detection
        self._start_key_monitor()
    
    def _print_instructions(self):
        print("\n🎮 ENHANCED HYBRID CONTROL INSTRUCTIONS")
        print("=" * 50)
        print("📱 LEAP MOTION MODE:")
        print("  → Runs your existing simple_hand_control.py")
        print("  ✋ Open/Close hand gestures work as before")
        print("  ✌️  Peace sign gesture → SWITCH TO KEYBOARD MODE")
        print("\n⌨️  KEYBOARD MODE:")
        print("  → Runs your existing keyboard_control_bse.py") 
        print("  ← → ↑ ↓ arrows work as before")
        print("  TAB key → SWITCH TO LEAP MOTION MODE")
        print("\nPress 'Q' to quit")
        print("-" * 50)
    
    def switch_to_leap_motion(self):
        """Stop keyboard control and start leap motion control"""
        print("\n🔄 Switching to LEAP MOTION mode...")
        
        # Stop keyboard process if running
        if self.keyboard_process and self.keyboard_process.poll() is None:
            self.keyboard_process.terminate()
            print("⌨️  Stopped keyboard control")
            
            # Wait for process to fully terminate
            try:
                self.keyboard_process.wait(timeout=5)
                print("✅ Keyboard process terminated cleanly")
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing keyboard process...")
                self.keyboard_process.kill()
                self.keyboard_process.wait()
            
            # Wait for serial port to be released
            time.sleep(3)
            print("⏳ Waiting for Arduino port to be released...")
        
        # Start leap motion process
        leap_script = "src/controller/leap_motion_control/simple_hand_control.py"
        try:
            self.leap_process = subprocess.Popen(
                ["python", leap_script],
                cwd="/Users/mbp/Documents/ensimag/HCI-Project"
            )
            print("👋 Started Leap Motion control (simple_hand_control.py)")
            self.current_mode = ControlMode.LEAP_MOTION
            
            # Start gesture monitoring for peace sign (separate from the control process)
            time.sleep(2)  # Give main process time to start
            self._start_gesture_monitor()
            
        except Exception as e:
            print(f"❌ Failed to start Leap Motion control: {e}")
    
    def switch_to_keyboard(self):
        """Stop leap motion and start keyboard control"""
        print("\n🔄 Switching to KEYBOARD mode...")
        
        # Stop gesture monitoring first
        self._stop_gesture_monitor()
        
        # Stop leap motion process if running
        if self.leap_process and self.leap_process.poll() is None:
            self.leap_process.terminate()
            print("👋 Stopped Leap Motion control")
            
            # Wait for process to fully terminate
            try:
                self.leap_process.wait(timeout=5)
                print("✅ Leap Motion process terminated cleanly")
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing Leap Motion process...")
                self.leap_process.kill()
                self.leap_process.wait()
            
            # Wait for serial port to be released
            time.sleep(3)
            print("⏳ Waiting for Arduino port to be released...")
        
        # Start keyboard process  
        keyboard_script = "src/controller/keyboard_control/keyboard_control_bse.py"
        try:
            self.keyboard_process = subprocess.Popen(
                ["python", keyboard_script],
                cwd="/Users/mbp/Documents/ensimag/HCI-Project"
            )
            print("⌨️  Started keyboard control (keyboard_control_bse.py)")
            self.current_mode = ControlMode.KEYBOARD
        except Exception as e:
            print(f"❌ Failed to start keyboard control: {e}")
    
    def _start_gesture_monitor(self):
        """Start monitoring for peace sign gesture - NO Arduino connection"""
        if self.leap_connection is not None:
            return  # Already running
            
        class PeaceSignListener(leap.Listener):
            def __init__(self, controller):
                self.controller = controller
                self.frame_count = 0
                self.last_gestures = {}
                
            def on_tracking_event(self, event):
                # Only monitor gestures when in Leap Motion mode
                if self.controller.current_mode != ControlMode.LEAP_MOTION:
                    return
                    
                self.frame_count += 1
                if self.frame_count % 15 != 0:  # Check less frequently
                    return
                    
                if not event.hands:
                    return
                    
                hand = event.hands[0]
                if self._detect_peace_sign(hand):
                    print("✌️  Peace sign detected - switching to keyboard!")
                    self.controller.switch_to_keyboard()
            
            def _detect_peace_sign(self, hand):
                """Detect peace sign gesture"""
                hand_id = hand.id
                fingers_extended = [
                    hand.thumb.is_extended,
                    hand.index.is_extended,
                    hand.middle.is_extended,
                    hand.ring.is_extended,
                    hand.pinky.is_extended
                ]
                
                # Peace sign: only index and middle fingers extended
                is_peace_sign = (fingers_extended[1] and fingers_extended[2] and 
                               not fingers_extended[0] and not fingers_extended[3] and not fingers_extended[4])
                
                # Alternative: exactly 2 fingers extended and they are index + middle
                alt_peace_sign = (sum(fingers_extended) == 2 and fingers_extended[1] and fingers_extended[2])
                
                detected = is_peace_sign or alt_peace_sign
                
                # Only trigger if gesture changed
                if (hand_id not in self.last_gestures or 
                    self.last_gestures[hand_id] != detected):
                    self.last_gestures[hand_id] = detected
                    return detected
                
                return False
        
        try:
            self.gesture_listener = PeaceSignListener(self)
            self.leap_connection = leap.Connection()
            self.leap_connection.add_listener(self.gesture_listener)
            
            # Run in separate thread
            def start_leap_monitoring():
                try:
                    with self.leap_connection.open():
                        self.leap_connection.set_tracking_mode(leap.TrackingMode.Desktop)
                        while self.running and self.leap_connection:
                            time.sleep(0.1)
                except Exception as e:
                    print(f"🔍 Gesture monitor error: {e}")
            
            self.leap_thread = threading.Thread(target=start_leap_monitoring, daemon=True)
            self.leap_thread.start()
            print("🔍 Started peace sign monitoring (no Arduino conflict)")
            
        except Exception as e:
            print(f"⚠️  Could not start gesture monitoring: {e}")
    
    def _stop_gesture_monitor(self):
        """Stop gesture monitoring"""
        try:
            if self.leap_connection:
                self.leap_connection.close()
                self.leap_connection = None
                print("🔍 Stopped peace sign monitoring")
                time.sleep(1)
        except Exception as e:
            print(f"⚠️  Error stopping gesture monitor: {e}")

    def _start_key_monitor(self):
        """Start monitoring for TAB key and Q key"""
        def on_key_press(key):
            try:
                if key == keyboard.Key.tab:
                    # Only switch if in keyboard mode
                    if self.current_mode == ControlMode.KEYBOARD:
                        print("⌨️  TAB key detected - switching to Leap Motion!")
                        self.switch_to_leap_motion()
                
                elif hasattr(key, 'char') and (key.char == 'q' or key.char == 'Q'):
                    print("\n👋 Q key pressed - quitting hybrid controller...")
                    self.shutdown()
                    return False
                        
            except AttributeError:
                pass
        
        self.key_listener = keyboard.Listener(on_press=on_key_press)
        self.key_listener.start()
    
    def shutdown(self):
        """Clean shutdown of all processes"""
        self.running = False
        
        # Stop gesture monitoring first
        self._stop_gesture_monitor()
        
        # Stop leap motion process
        if self.leap_process and self.leap_process.poll() is None:
            self.leap_process.terminate()
            print("👋 Stopped Leap Motion control")
        
        # Stop keyboard process
        if self.keyboard_process and self.keyboard_process.poll() is None:
            self.keyboard_process.terminate()
            print("⌨️  Stopped keyboard control")
        
        # Stop key listener
        if hasattr(self, 'key_listener'):
            self.key_listener.stop()
        
        print("✅ All processes and monitors stopped")
    
    def run(self):
        """Main loop with better error handling"""
        try:
            while self.running:
                time.sleep(1)  # Slower polling
                
        except KeyboardInterrupt:
            print("\n👋 Keyboard interrupt received")
            
        finally:
            self.shutdown()


def main():
    print("🚀 Starting Enhanced Hybrid Control System...")
    print("✌️  Peace sign gesture switches from Leap Motion → Keyboard")
    print("⌨️  TAB key switches from Keyboard → Leap Motion")
    print("📍 No Arduino conflicts - gesture monitoring is separate")
    
    # Create and run hybrid controller
    controller = HybridController()
    controller.run()


if __name__ == "__main__":
    main()