import cv2
import mediapipe as mp
import serial
import time
import commands 

# --- CONFIG ---
SERIAL_PORT = '/dev/tty.usbmodem21201' # <--- CHECK YOUR PORT
SENSITIVITY = 2.0 
SHOULDER_FIXED = 140 # Safe Fixed Angle

# --- SETUP SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(2) 
    print(f"✅ Serial Connected on {SERIAL_PORT}")
except:
    print("⚠️ Serial NOT Connected")
    ser = None

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, max_num_hands=1)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

print("--- 4-GESTURE CONTROL ---")
print("✊ FIST   -> CLOSE Gripper (Stop Move)")
print("✋ PALM   -> OPEN Gripper  (Stop Move)")
print("☝️ INDEX  -> Move BASE (Keep Grip)")
print("✌️ PEACE  -> Move ARM  (Keep Grip)")

# State Memory
current_elbow = 90
is_grip_closed = False # "Sticky" Variable
alpha = 0.2

# --- GESTURE DEFINITIONS ---
def count_fingers_up(lm):
    count = 0
    # Tips: 8, 12, 16, 20. PIPs: 6, 10, 14, 18
    if lm[8].y < lm[6].y: count += 1  # Index
    if lm[12].y < lm[10].y: count += 1 # Middle
    if lm[16].y < lm[14].y: count += 1 # Ring
    if lm[20].y < lm[18].y: count += 1 # Pinky
    # Thumb is tricky, let's ignore it for robust 1 vs 2 vs 4 distinction
    return count

def map_sens(val):
    centered = val - 0.5
    scaled = centered * SENSITIVITY
    return max(0.0, min(1.0, scaled + 0.5))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    # UI Defaults
    mode_text = "WAITING"
    status_color = (200, 200, 200)
    
    if results.multi_hand_landmarks:
        lm = results.multi_hand_landmarks[0].landmark
        mp_draw.draw_landmarks(frame, results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)
        
        # 1. IDENTIFY GESTURE
        fingers = count_fingers_up(lm)
        
        # 2. STATE MACHINE
        if fingers == 0: 
            # --- FIST (0 fingers) = GRAB ---
            mode_text = "ACTION: GRAB (FIST)"
            status_color = (0, 0, 255) # Red
            is_grip_closed = True
            
            # Stop moving motors while changing grip
            if ser: commands.send_rotate_stop(ser)
            
        elif fingers >= 4:
            # --- PALM (4+ fingers) = RELEASE ---
            mode_text = "ACTION: RELEASE (PALM)"
            status_color = (0, 255, 0) # Green
            is_grip_closed = False
            
            # Stop moving motors while changing grip
            if ser: commands.send_rotate_stop(ser)
            
        elif fingers == 1:
            # --- INDEX (1 finger) = BASE MOVE ---
            mode_text = "MODE: BASE (Left/Right)"
            status_color = (255, 255, 0) # Cyan
            
            # Map X for Base
            val_x = map_sens(lm[0].x)
            if val_x < 0.4:
                if ser: commands.send_rotate_left(ser)
                mode_text += " <<L"
            elif val_x > 0.6:
                if ser: commands.send_rotate_right(ser)
                mode_text += " R>>"
            else:
                if ser: commands.send_rotate_stop(ser)
            
            # Freeze Elbow
            if ser: commands.send_elbow(ser, current_elbow)

        elif fingers == 2:
            # --- PEACE (2 fingers) = ARM MOVE ---
            mode_text = "MODE: ARM (Up/Down)"
            status_color = (255, 0, 255) # Magenta
            
            # Stop Base
            if ser: commands.send_rotate_stop(ser)
            
            # Map Y for Elbow
            # Top(0) -> Lift(45), Bottom(1) -> Reach(160)
            val_y = map_sens(lm[0].y)
            target_elbow = int(45 + (val_y * 115))
            
            # Smooth it
            current_elbow = int(current_elbow * (1 - alpha) + target_elbow * alpha)
            if ser: commands.send_elbow(ser, current_elbow)

        # 3. ALWAYS UPDATE SHARED STATE
        # Ensure Shoulder is fixed and Grip remembers its state
        if ser:
            commands.send_shoulder(ser, SHOULDER_FIXED)
            commands.send_wrist(ser, 90)
            commands.send_grip(ser, is_grip_closed)

    else:
        # Safety Stop
        if ser: commands.send_rotate_stop(ser)

    # --- UI DISPLAY ---
    cv2.rectangle(frame, (0,0), (640, 80), (20,20,20), -1)
    
    # Mode Text
    cv2.putText(frame, mode_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
    
    # Grip Status Indicator
    grip_str = "CLOSED" if is_grip_closed else "OPEN"
    grip_col = (0, 0, 255) if is_grip_closed else (0, 255, 0)
    cv2.putText(frame, f"Grip State: {grip_str}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, grip_col, 2)
    
    # Visual Guide
    cv2.line(frame, (320, 0), (320, 480), (50, 50, 50), 1)
    cv2.line(frame, (0, 240), (640, 240), (50, 50, 50), 1)

    cv2.imshow("4-Gesture Control", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()