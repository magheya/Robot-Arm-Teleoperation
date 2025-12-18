import cv2
import mediapipe as mp
import serial
import time
from cv_recognizer import CVHandRecognizer
from utils import GestureType
import commands # Your existing commands.py

# --- SETUP SERIAL ---
# CHANGE 'COM3' TO YOUR PORT
try:
    ser = serial.Serial('/dev/tty.usbmodem1201', 115200, timeout=1)
    time.sleep(2) # Wait for Arduino restart
    print("✅ Serial Connected")
except:
    print("⚠️ Serial NOT Connected (Debug Mode)")
    ser = None

# --- SETUP MEDIAPIPE ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    min_detection_confidence=0.7, 
    min_tracking_confidence=0.5,
    max_num_hands=2 # ENABLE 2 HANDS
)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

recognizer = CVHandRecognizer()
mode = "BIMANUAL" # Default mode

print("--- CONTROLLER STARTED ---")
print("Press 'u' for Unimanual Mode")
print("Press 'b' for Bimanual Mode")
print("Press 'q' to Quit")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    # 1. Process Frame
    frame = cv2.flip(frame, 1) # Mirror effect
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    right_hand = None
    left_hand = None

    if results.multi_hand_landmarks:
        for landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            label = handedness.classification[0].label # "Left" or "Right"
            
            # Map MediaPipe Labels (which are mirrored)
            # If flipped: 'Right' label usually means user's Left hand visually
            # Let's trust label but verify with X coord if needed
            if label == "Right": right_hand = landmarks.landmark
            else: left_hand = landmarks.landmark
            
            mp_draw.draw_landmarks(frame, landmarks, mp_hands.HAND_CONNECTIONS)

    # 2. Analyze Gestures
    grip_cmd = GestureType.OPEN_HAND
    base_cmd = GestureType.STOP_MOVE_HORIZONTAL
    sh_angle, el_angle, wr_angle = 150, 90, 180
    debug_str = "No Hands"

    if mode == "BIMANUAL" and right_hand and left_hand:
        # Require BOTH hands to act
        grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str = \
            recognizer.analyze_bimanual(right_hand, left_hand)
            
    elif mode == "UNIMANUAL" and right_hand:
        # Only uses Right hand
        grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str = \
            recognizer.analyze_unimanual(right_hand)
            
    elif right_hand: # Fallback if only one hand in Bimanual
        debug_str = "Waiting for Left Hand..."

    # 3. Send to Robot (Only if Serial is connected)
    if ser:
        # Base
        if base_cmd == GestureType.MOVE_LEFT: commands.send_rotate_left(ser)
        elif base_cmd == GestureType.MOVE_RIGHT: commands.send_rotate_right(ser)
        else: commands.send_rotate_stop(ser)
        
        # Arms
        commands.send_shoulder(ser, sh_angle)
        commands.send_elbow(ser, el_angle)
        commands.send_wrist(ser, wr_angle)
        
        # Grip
        is_closed = (grip_cmd == GestureType.CLOSED_HAND)
        commands.send_grip(ser, is_closed)

    # 4. Display UI
    cv2.rectangle(frame, (0,0), (640, 60), (0,0,0), -1) # Top Bar
    cv2.putText(frame, f"MODE: {mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, debug_str, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    # Draw "Deadzone" lines for Base Control
    cv2.line(frame, (int(640*0.4), 0), (int(640*0.4), 480), (100,100,100), 1)
    cv2.line(frame, (int(640*0.6), 0), (int(640*0.6), 480), (100,100,100), 1)

    cv2.imshow("Robot Controller", frame)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'): break
    if key & 0xFF == ord('u'): mode = "UNIMANUAL"
    if key & 0xFF == ord('b'): mode = "BIMANUAL"

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()