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
        port = '/dev/cu.usbmodem101' # CHECK THIS!
        try:
            self.arduino = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to Arduino on {port}")
        except Exception as e:
            print(f"⚠️ SIMULATION MODE: {e}")
            self.arduino = None

    def update(self, grip, base, shoulder, elbow, wrist):
        if not self.arduino:
            # print(f"SIM: Base={base} Shldr={shoulder} Elbw={elbow} Wrst={wrist} Grip={grip}")
            return

        try:
            # 1. Base (Stepper)
            if base == GestureType.MOVE_LEFT: commands.send_rotate_left(self.arduino)
            elif base == GestureType.MOVE_RIGHT: commands.send_rotate_right(self.arduino)
            else: commands.send_rotate_stop(self.arduino)

            # 2. Servos (Absolute Positioning)
            # We send these every frame, which is fine because Arduino handles it fast
            commands.send_shoulder(self.arduino, shoulder)
            commands.send_elbow(self.arduino, elbow)
            commands.send_wrist(self.arduino, wrist)
            
            # 3. Gripper
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
        
        image = cv2.flip(image, 1) # Mirror view
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image_rgb)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Analyze
                grip, base, shoulder, elbow, wrist, debug = recognizer.analyze_hand(hand_landmarks.landmark)
                
                # Control
                controller.update(grip, base, shoulder, elbow, wrist)
                
                # Display Data
                cv2.putText(image, debug, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(image, f"Base: {base.name}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow('Robot Arm Control', image)
        if cv2.waitKey(5) & 0xFF == ord('q'): break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()