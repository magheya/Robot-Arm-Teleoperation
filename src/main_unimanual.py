import cv2
import mediapipe as mp
import serial
import time
from cv_recognizer import CVHandRecognizer
from utils import GestureType
import commands 

# --- CONFIG ---
SERIAL_PORT = 'COM3' # CHECK YOUR PORT

# --- SETUP SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(2) 
    print(f"✅ Serial Connected on {SERIAL_PORT}")
except:
    print("⚠️ Serial NOT Connected (Running in Debug Mode)")
    ser = None

# --- SETUP MEDIAPIPE (1 HAND) ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    min_detection_confidence=0.7, 
    min_tracking_confidence=0.5,
    max_num_hands=1 
)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

recognizer = CVHandRecognizer()

print("--- UNIMANUAL MODE STARTED ---")
print("Press 'q' to Quit")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    # Defaults
    grip_cmd = GestureType.OPEN_HAND
    base_cmd = GestureType.STOP_MOVE_HORIZONTAL
    sh_angle, el_angle, wr_angle = 150, 90, 180
    debug_str = "Waiting for Hand..."
    
    # Text colors (BGR)
    color_grip = (0, 255, 0) # Green
    color_base = (0, 255, 0) # Green

    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str = \
            recognizer.analyze_unimanual(hand_landmarks.landmark)

    # --- SEND TO ROBOT ---
    if ser:
        if base_cmd == GestureType.MOVE_LEFT: commands.send_rotate_left(ser)
        elif base_cmd == GestureType.MOVE_RIGHT: commands.send_rotate_right(ser)
        else: commands.send_rotate_stop(ser)
        
        commands.send_shoulder(ser, sh_angle)
        commands.send_elbow(ser, el_angle)
        commands.send_wrist(ser, wr_angle)
        commands.send_grip(ser, grip_cmd == GestureType.CLOSED_HAND)

    # --- UI DISPLAY (THE NEW PART) ---
    
    # 1. Determine Status Strings
    if grip_cmd == GestureType.CLOSED_HAND:
        grip_txt = "GRIP: CLOSED (FIST)"
        color_grip = (0, 0, 255) # Red for Closed
    else:
        grip_txt = "GRIP: OPEN"
        color_grip = (0, 255, 0) # Green for Open
        
    if base_cmd == GestureType.MOVE_LEFT:
        base_txt = "<< MOVING LEFT"
        color_base = (0, 255, 255) # Yellow
    elif base_cmd == GestureType.MOVE_RIGHT:
        base_txt = "MOVING RIGHT >>"
        color_base = (0, 255, 255) # Yellow
    else:
        base_txt = "BASE: STOPPED"
        color_base = (200, 200, 200) # Gray

    # 2. Draw Top Bar Background
    cv2.rectangle(frame, (0,0), (640, 80), (0,0,0), -1)
    
    # 3. Print Text
    # Mode Title
    cv2.putText(frame, "MODE: UNIMANUAL", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Grip Status (Left Side)
    cv2.putText(frame, grip_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_grip, 2)
    
    # Base Status (Right Side)
    cv2.putText(frame, base_txt, (350, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_base, 2)
    
    # Debug Info (Small, below)
    cv2.putText(frame, debug_str, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Deadzone lines (Visual Aid for Left/Right)
    cv2.line(frame, (int(640*0.4), 0), (int(640*0.4), 480), (100,100,100), 1)
    cv2.line(frame, (int(640*0.6), 0), (int(640*0.6), 480), (100,100,100), 1)

    cv2.imshow("Unimanual Controller", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()