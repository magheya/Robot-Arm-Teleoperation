import cv2
import mediapipe as mp
import serial
import time
import commands 

# --- CONFIG ---
SERIAL_PORT = '/dev/tty.usbmodem1201' # <--- CHECK YOUR PORT

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

print("--- ELBOW TEST MODE ---")
print("Move Hand UP   -> Angle 180")
print("Move Hand DOWN -> Angle 0")
print("⚠️ WARNING: Move SLOWLY to avoid hitting the table!")
print("Press 'q' to Quit")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    elbow_angle = 90 # Default Center
    
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        # 1. Get Y Position (0.0 Top to 1.0 Bottom)
        y_pos = hand_landmarks.landmark[0].y
        
        # 2. Map Y directly to Angle
        # Top (0.0) -> 180
        # Bottom (1.0) -> 0
        elbow_angle = int((1.0 - y_pos) * 180)
        
        # Clamp to hardware limits
        elbow_angle = max(0, min(180, elbow_angle))

    # --- SEND TO ROBOT ---
    if ser:
        commands.send_elbow(ser, elbow_angle)

    # --- UI ---
    cv2.rectangle(frame, (0,0), (640, 60), (0,0,0), -1)
    cv2.putText(frame, "TEST: ELBOW ONLY", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # Visual Bar
    cv2.putText(frame, f"Angle: {elbow_angle}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Vertical Slider (Visual Aid)
    slider_height = int(480 * (elbow_angle / 180))
    cv2.rectangle(frame, (600, 480 - slider_height), (630, 480), (0, 255, 0), -1)
    cv2.rectangle(frame, (600, 0), (630, 480), (100, 100, 100), 2)

    cv2.imshow("Elbow Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()