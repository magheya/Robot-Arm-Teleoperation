import cv2
import mediapipe as mp
import serial
import time
import commands
from utils import GestureType
from cv_recognizer import CVHandRecognizer

class RobotArmController:
    def __init__(self):
        self.arduino = None
        self._connect()
        
    def _connect(self):
        port = '/dev/cu.usbmodem101' # Update if needed
        try:
            self.arduino = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to Arduino on {port}")
        except Exception as e:
            print(f"⚠️  SIMULATION MODE (No Robot Connected)")
            self.arduino = None

    def update(self, grip, base, shoulder, elbow, wrist):
        # --- SIMULATION PRINTS (Continuous) ---
        status_msg = ""
        
        # 1. Base Status
        if base == GestureType.MOVE_LEFT: status_msg += "⬅️ BASE: LEFT   "
        elif base == GestureType.MOVE_RIGHT: status_msg += "➡️ BASE: RIGHT  "
        else: status_msg += "⏹ BASE: STOP   "
        
        # 2. Gripper Status
        if grip == GestureType.CLOSED_HAND: status_msg += "| ✊ GRIP: CLOSE "
        else: status_msg += "| 🖐 GRIP: OPEN  "
        
        # 3. Arm Height Status
        status_msg += f"| ⬆️ ARM HEIGHT: {shoulder}°"

        # 4. Elbow Status
        status_msg += f"| � elbow: {elbow}°"

        # Print continuously (using \r to overwrite line for cleaner look, or normal print)
        print(status_msg)

        # --- SEND TO ARDUINO (If connected) ---
        if self.arduino:
            try:
                if base == GestureType.MOVE_LEFT: commands.send_rotate_left(self.arduino)
                elif base == GestureType.MOVE_RIGHT: commands.send_rotate_right(self.arduino)
                else: commands.send_rotate_stop(self.arduino)

                commands.send_shoulder(self.arduino, shoulder)
                commands.send_elbow(self.arduino, elbow)
                commands.send_wrist(self.arduino, wrist)
                
                is_closed = (grip == GestureType.CLOSED_HAND)
                commands.send_grip(self.arduino, is_closed)
            except Exception as e:
                print(f"Serial Error: {e}")

def main():
    cap = cv2.VideoCapture(0)
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(max_num_hands=1)
    mp_draw = mp.solutions.drawing_utils
    
    controller = RobotArmController()
    recognizer = CVHandRecognizer()
    
    print("📷 System Ready. Press 'q' to quit.")
    
    while cap.isOpened():
        success, image = cap.read()
        if not success: continue
        
        image = cv2.flip(image, 1)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Analyze
                grip, base, shoulder, elbow, wrist, debug = recognizer.analyze_hand(hand_landmarks.landmark)
                
                # Control & Print
                controller.update(grip, base, shoulder, elbow, wrist)
                
                # Display Debug on Screen
                cv2.putText(image, debug, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Robot Arm Control', image)
        if cv2.waitKey(5) & 0xFF == ord('q'): break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()