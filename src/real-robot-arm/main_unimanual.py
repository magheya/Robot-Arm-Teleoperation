import cv2
import mediapipe as mp
import serial
import time
from cv_recognizer import CVHandRecognizer
from utils import GestureType
import commands 

# --- CONFIG ---
SERIAL_PORT = '/dev/tty.usbmodem1201' # CHECK YOUR PORT

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

# --- STATE VARIABLES FOR LOCK ---
motor_locked = False       # Starts UNLOCKED (Active)
last_toggle_time = 0       # For cooldown (debounce)
TOGGLE_COOLDOWN = 1.0      # Seconds to wait between toggles

print("--- UNIMANUAL MODE STARTED ---")
print("✌️ PEACE SIGN to Lock/Unlock Motors")
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
    
    # Colors
    color_grip = (0, 255, 0)
    color_base = (0, 255, 0)

    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        # 1. Analyze Gestures
        grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str = \
            recognizer.analyze_unimanual(hand_landmarks.landmark)
            
        # 2. CHECK FOR LOCK TOGGLE (PEACE SIGN)
        if recognizer.is_peace_sign(hand_landmarks.landmark):
            current_time = time.time()
            if current_time - last_toggle_time > TOGGLE_COOLDOWN:
                motor_locked = not motor_locked # Toggle State
                last_toggle_time = current_time
                print(f"🔄 TOGGLE! Motor Locked: {motor_locked}")

    # 3. OVERRIDE IF LOCKED
    # If motors are locked, FORCE stop, regardless of hand position
    if motor_locked:
        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        debug_str = "⛔ MOTORS LOCKED (Peace Sign to Unlock)"

    # --- SEND TO ROBOT ---
    if ser:
        if base_cmd == GestureType.MOVE_LEFT: commands.send_rotate_left(ser)
        elif base_cmd == GestureType.MOVE_RIGHT: commands.send_rotate_right(ser)
        else: commands.send_rotate_stop(ser)
        
        commands.send_shoulder(ser, sh_angle)
        commands.send_elbow(ser, el_angle)
        commands.send_wrist(ser, wr_angle)
        commands.send_grip(ser, grip_cmd == GestureType.CLOSED_HAND)

    # --- UI DISPLAY ---
    
    # Grip Status
    if grip_cmd == GestureType.CLOSED_HAND:
        grip_txt, color_grip = "GRIP: CLOSED", (0, 0, 255)
    else:
        grip_txt, color_grip = "GRIP: OPEN", (0, 255, 0)
        
    # Base Status (Handle Lock UI)
    if motor_locked:
        base_txt = "⛔ LOCKED"
        color_base = (0, 0, 255) # Red
    elif base_cmd == GestureType.MOVE_LEFT:
        base_txt = "<< LEFT"
        color_base = (0, 255, 255) # Yellow
    elif base_cmd == GestureType.MOVE_RIGHT:
        base_txt = "RIGHT >>"
        color_base = (0, 255, 255) # Yellow
    else:
        base_txt = "ACTIVE (STOP)"
        color_base = (200, 200, 200) # Gray

    # Draw UI
    cv2.rectangle(frame, (0,0), (640, 80), (20,20,20), -1)
    
    # Title
    cv2.putText(frame, "MODE: UNIMANUAL (DEPTH)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    # Status Text
    cv2.putText(frame, grip_txt, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_grip, 2)
    cv2.putText(frame, base_txt, (450, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_base, 2)
    
    # Instructions/Debug
    cv2.putText(frame, debug_str, (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Deadzone lines (Visual Aid) - Change color if locked
    line_color = (50, 50, 255) if motor_locked else (100, 100, 100)
    cv2.line(frame, (int(640*0.4), 0), (int(640*0.4), 480), line_color, 1)
    cv2.line(frame, (int(640*0.6), 0), (int(640*0.6), 480), line_color, 1)

    cv2.imshow("Unimanual Controller", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()