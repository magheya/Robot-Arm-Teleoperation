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

print("--- WRIST TEST MODE ---")
print("Move Hand LEFT  -> Wrist 0")
print("Move Hand RIGHT -> Wrist 180")
print("Press 'q' to Quit")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    wrist_angle = 90 # Default Center
    
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        # 1. Get X Position (0.0 to 1.0)
        x_pos = hand_landmarks.landmark[0].x
        
        # 2. Map X directly to Angle (0 to 180)
        # 0.0 (Left) -> 0 deg
        # 1.0 (Right) -> 180 deg
        wrist_angle = int(x_pos * 180)
        
        # Clamp just to be safe (Python logic side)
        wrist_angle = max(0, min(180, wrist_angle))

    # --- SEND TO ROBOT ---
    if ser:
        commands.send_wrist(ser, wrist_angle)

    # --- UI ---
    cv2.rectangle(frame, (0,0), (640, 60), (0,0,0), -1)
    cv2.putText(frame, "TEST: WRIST ONLY", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    # Visual Bar
    cv2.putText(frame, f"Angle: {wrist_angle}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Draw a visual slider at the bottom
    cv2.rectangle(frame, (50, 450), (590, 470), (100, 100, 100), 2)
    slider_x = int(50 + (wrist_angle/180 * 540))
    cv2.circle(frame, (slider_x, 460), 10, (0, 255, 0), -1)

    cv2.imshow("Wrist Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
if ser: ser.close()
cv2.destroyAllWindows()