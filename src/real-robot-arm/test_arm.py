import cv2
import mediapipe as mp
import serial
import time
import commands 

# --- CONFIG ---
SERIAL_PORT = '/dev/tty.usbmodem1201' # <--- CHECK PORT
SENSITIVITY = 2.5  # <--- ADJUST THIS! (Higher = Faster/Less Hand Movement)

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

print("--- HIGH SENSITIVITY TEST ---")
print(f"Sensitivity: {SENSITIVITY}x")
print("Small Hand Movement -> BIG Robot Movement")

# Smoothing variables (Low alpha = very smooth, High alpha = very responsive)
smooth_wrist = 90
smooth_elbow = 90
alpha = 0.2 

def map_with_sensitivity(value, sensitivity):
    """
    Maps an input (0.0 to 1.0) to a scaled range centered at 0.5.
    Returns a value between 0.0 and 1.0.
    """
    # 1. Center the value (-0.5 to 0.5)
    centered = value - 0.5
    
    # 2. Scale it (Amplify movement)
    scaled = centered * sensitivity
    
    # 3. Un-center and Clamp (0.0 to 1.0)
    final_val = scaled + 0.5
    return max(0.0, min(1.0, final_val))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    is_grip = False
    
    # Draw the "Active Zone" Box (Visual Guide)
    # If your hand is inside this box, you have control. 
    # Outside means you hit the max limit.
    zone_width = int(640 / SENSITIVITY)
    zone_height = int(480 / SENSITIVITY)
    x_start = int((640 - zone_width) / 2)
    y_start = int((480 - zone_height) / 2)
    cv2.rectangle(frame, (x_start, y_start), (x_start+zone_width, y_start+zone_height), (0, 255, 0), 2)

    if results.multi_hand_landmarks:
        landmarks = results.multi_hand_landmarks[0].landmark
        mp_draw.draw_landmarks(frame, results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)
        
        # --- 1. RAW INPUTS ---
        raw_x = landmarks[0].x
        raw_y = landmarks[0].y
        
        # --- 2. APPLY SENSITIVITY SCALING ---
        scaled_x = map_with_sensitivity(raw_x, SENSITIVITY)
        scaled_y = map_with_sensitivity(raw_y, SENSITIVITY)
        
        # --- 3. CONVERT TO ANGLES ---
        # Wrist: Left(0) to Right(180)
        target_wrist = int(scaled_x * 180)
        
        # Elbow: Top(0) is Extend(160), Bottom(1) is Retract(45)
        # Note: We use scaled_y now!
        target_elbow = int(160 - (scaled_y * 115)) 
        
        # --- 4. GRIP ---
        fingers_folded = 0
        if landmarks[8].y > landmarks[6].y: fingers_folded += 1
        if landmarks[12].y > landmarks[10].y: fingers_folded += 1
        if landmarks[16].y > landmarks[14].y: fingers_folded += 1
        if landmarks[20].y > landmarks[18].y: fingers_folded += 1
        is_grip = (fingers_folded >= 3)

        # --- SMOOTHING ---
        smooth_wrist = int(smooth_wrist * (1 - alpha) + target_wrist * alpha)
        smooth_elbow = int(smooth_elbow * (1 - alpha) + target_elbow * alpha)

    # --- SEND TO ROBOT ---
    if ser:
        commands.send_shoulder(ser, 150) # Locked
        commands.send_wrist(ser, smooth_wrist)
        commands.send_elbow(ser, smooth_elbow)
        commands.send_grip(ser, is_grip)

    # --- UI ---
    cv2.rectangle(frame, (0,0), (640, 80), (20,20,20), -1)
    
    grip_txt = "CLOSED" if is_grip else "OPEN"
    c_grip = (0, 0, 255) if is_grip else (0, 255, 0)
    
    cv2.putText(frame, f"SENSITIVITY: {SENSITIVITY}x", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(frame, f"Grip: {grip_txt}", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c_grip, 2)
    
    # Draw Crosshair
    cx = int(x_start + (smooth_wrist/180 * zone_width))
    # Invert mapping for visual Y
    cy_norm = (160 - smooth_elbow) / 115
    cy = int(y_start + (cy_norm * zone_height))
    
    # Clamp visuals to screen
    cx = max(0, min(640, cx))
    cy = max(0, min(480, cy))
    
    cv2.circle(frame, (cx, cy), 10, (0, 255, 255), 2)

    cv2.imshow("High Speed Control", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()