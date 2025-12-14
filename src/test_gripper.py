import cv2
import mediapipe as mp
import serial
import time
from cv_recognizer import CVHandRecognizer
from utils import GestureType
import commands 

# --- CONFIG ---
SERIAL_PORT = '/dev/tty.usbmodem1201' # <--- CHECK YOUR PORT

# --- SETUP SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(2) 
    print(f"✅ Serial Connected on {SERIAL_PORT}")
except:
    print("⚠️ Serial NOT Connected (Running in Debug Mode)")
    ser = None

# --- SETUP MEDIAPIPE ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, max_num_hands=1)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

recognizer = CVHandRecognizer()

print("--- GRIPPER TEST MODE ---")
print("1. OPEN HAND -> Servos go to 180")
print("2. FIST      -> Servos go to 0")
print("Press 'q' to Quit")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    grip_cmd = GestureType.OPEN_HAND
    debug_str = "Waiting for Hand..."
    
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        # Use existing logic to detect Fist vs Open
        # We ignore base/shoulder/elbow return values here
        grip_cmd, _, _, _, _, _ = recognizer.analyze_unimanual(hand_landmarks.landmark)
        
        # Override debug string for this specific test
        if grip_cmd == GestureType.CLOSED_HAND:
            debug_str = "FIST DETECTED -> Sending 0"
        else:
            debug_str = "OPEN DETECTED -> Sending 180"

    # --- SEND ONLY GRIP COMMANDS ---
    if ser:
        is_closed = (grip_cmd == GestureType.CLOSED_HAND)
        commands.send_grip(ser, is_closed)

    # --- UI ---
    cv2.rectangle(frame, (0,0), (640, 60), (0,0,0), -1)
    cv2.putText(frame, "TEST: GRIPPER ONLY", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # Status Color
    c_status = (0, 0, 255) if grip_cmd == GestureType.CLOSED_HAND else (0, 255, 0)
    status_txt = "CLOSED (0)" if grip_cmd == GestureType.CLOSED_HAND else "OPEN (180)"
    
    cv2.putText(frame, status_txt, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, c_status, 2)
    cv2.imshow("Gripper Test", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()