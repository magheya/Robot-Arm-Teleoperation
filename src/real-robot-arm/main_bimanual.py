import cv2
import mediapipe as mp
import serial
import time
from cv_recognizer import CVHandRecognizer
from utils import GestureType
import commands

# --- CONFIG ---
SERIAL_PORT = '/dev/tty.usbmodem1201' 

# --- SETUP SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(2)
    print(f"✅ Serial Connected on {SERIAL_PORT}")
except:
    print("⚠️ Serial NOT Connected (Running in Debug Mode)")
    ser = None

# --- SETUP MEDIAPIPE (2 HANDS) ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    min_detection_confidence=0.7, 
    min_tracking_confidence=0.5,
    max_num_hands=2  # <--- FORCE 2 HANDS for Bimanual
)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

recognizer = CVHandRecognizer()

print("--- BIMANUAL MODE STARTED ---")
print("RIGHT HAND: Aiming (Base L/R + Shoulder U/D)")
print("LEFT HAND:  Action (Height = Reach In/Out, Fist = Grab)")
print("Press 'q' to Quit")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    right_hand = None
    left_hand = None

    # Identify Hands
    if results.multi_hand_landmarks:
        for landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            label = handedness.classification[0].label 
            
            # Note: MediaPipe labels are often mirrored relative to the user
            # Adjust this if your Left/Right feels swapped
            if label == "Right": right_hand = landmarks.landmark
            else: left_hand = landmarks.landmark
            
            mp_draw.draw_landmarks(frame, landmarks, mp_hands.HAND_CONNECTIONS)

    # Defaults
    grip_cmd = GestureType.OPEN_HAND
    base_cmd = GestureType.STOP_MOVE_HORIZONTAL
    sh_angle, el_angle, wr_angle = 150, 90, 180
    debug_str = "Waiting for Both Hands..."

    # Run Bimanual Logic ONLY if both hands are visible
    if right_hand and left_hand:
        grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str = \
            recognizer.analyze_bimanual(right_hand, left_hand)
    elif right_hand:
        debug_str = "Need Left Hand..."
    elif left_hand:
        debug_str = "Need Right Hand..."

    # --- SEND TO ROBOT ---
    if ser:
        if base_cmd == GestureType.MOVE_LEFT: commands.send_rotate_left(ser)
        elif base_cmd == GestureType.MOVE_RIGHT: commands.send_rotate_right(ser)
        else: commands.send_rotate_stop(ser)
        
        commands.send_shoulder(ser, sh_angle)
        commands.send_elbow(ser, el_angle)
        commands.send_wrist(ser, wr_angle)
        commands.send_grip(ser, grip_cmd == GestureType.CLOSED_HAND)

    # --- UI ---
    cv2.rectangle(frame, (0,0), (640, 60), (0,0,0), -1)
    cv2.putText(frame, "MODE: BIMANUAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, debug_str, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    cv2.line(frame, (int(640*0.4), 0), (int(640*0.4), 480), (100,100,100), 1)
    cv2.line(frame, (int(640*0.6), 0), (int(640*0.6), 480), (100,100,100), 1)

    cv2.imshow("Bimanual Controller", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()