import cv2
import mediapipe as mp
import serial
import time
import commands 

# --- CONFIG ---
SERIAL_PORT = '/dev/tty.usbmodem1201' # <--- CHECK PORT

# --- CUSTOMIZE YOUR PHYSICS HERE ---
SHOULDER_MIN = 90   # Don't go below this (prevents stalling with heavy gripper)
SHOULDER_MAX = 240  # Don't go above this (prevents falling backward)
SENSITIVITY  = 2.5  # 2.5x Speed (Small hand move = Big robot move)

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

print("--- SCALED SHOULDER TEST ---")
print(f"Range Restricted: {SHOULDER_MIN}° to {SHOULDER_MAX}°")
print(f"Sensitivity: {SENSITIVITY}x")

# Smoothing
smooth_angle = 150
alpha = 0.2

def map_value(value, in_min, in_max, out_min, out_max):
    # Standard mapping function
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    # Visual "Active Zone" box
    h, w, _ = frame.shape
    box_h = int(h / SENSITIVITY)
    y_start = int((h - box_h) / 2)
    y_end = y_start + box_h
    cv2.rectangle(frame, (100, y_start), (540, y_end), (0, 255, 0), 2)
    
    target_angle = smooth_angle # Default to last known

    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        # 1. Get raw Y (0.0 Top to 1.0 Bottom)
        raw_y = hand_landmarks.landmark[0].y
        
        # 2. Apply Sensitivity (Focus on the middle of the screen)
        # We want 0.5 to be the center.
        # If Sensitivity is 2.0, we only care about Y from 0.25 to 0.75
        center_y = 0.5
        range_half = 0.5 / SENSITIVITY
        y_min_limit = center_y - range_half
        y_max_limit = center_y + range_half
        
        # Clamp input to the box
        clamped_y = max(y_min_limit, min(y_max_limit, raw_y))
        
        # 3. Map Clamped Y to Servo Angle
        # Top of Box (y_min_limit) -> SHOULDER_MAX (Up)
        # Bottom of Box (y_max_limit) -> SHOULDER_MIN (Down)
        target_angle = map_value(clamped_y, y_min_limit, y_max_limit, SHOULDER_MAX, SHOULDER_MIN)
        
        # --- SMOOTHING ---
        smooth_angle = int(smooth_angle * (1 - alpha) + target_angle * alpha)

    # --- SEND TO ROBOT ---
    if ser:
        commands.send_shoulder(ser, smooth_angle)

    # --- UI ---
    cv2.rectangle(frame, (0,0), (640, 80), (20,20,20), -1)
    cv2.putText(frame, f"RANGE: {SHOULDER_MIN}-{SHOULDER_MAX}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(frame, f"Angle: {smooth_angle}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Draw Slider showing where the robot is within its ALLOWED range
    norm_pos = (smooth_angle - SHOULDER_MIN) / (SHOULDER_MAX - SHOULDER_MIN)
    bar_h = int(norm_pos * 480)
    cv2.rectangle(frame, (600, 480 - bar_h), (630, 480), (0, 255, 0), -1)
    cv2.rectangle(frame, (600, 0), (630, 480), (100, 100, 100), 2)

    cv2.imshow("Scaled Shoulder", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()