import cv2
import mediapipe as mp
import time
from cv_recognizer import CVHandRecognizer
from utils import GestureType
from simulator import RobotSimulator

# --- SETUP SIMULATION ---
URDF_PATH = "src/sim_files/robot_arm.urdf" 
sim = RobotSimulator(URDF_PATH)

# --- SETUP MEDIAPIPE ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    min_detection_confidence=0.7, 
    min_tracking_confidence=0.5,
    max_num_hands=2
)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

recognizer = CVHandRecognizer()
mode = "UNIMANUAL"

# --- State for Base Rotation ---
base_angle = 0.0
sh_angle = 0.0
BASE_ROTATION_SPEED = 1.0

print("--- SIMULATION CONTROLLER STARTED ---")
print("Press 'u' for Unimanual Mode")
print("Press 'b' for Bimanual Mode")
print("Press 'r' to Reset Simulation")
print("Press 'q' to Quit")

while cap.isOpened() and sim.physicsClient is not None:
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    frame_height, frame_width, _ = frame.shape
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)
    
    right_hand = None
    left_hand = None

    if results.multi_hand_landmarks:
        for landmarks in results.multi_hand_landmarks:
            wrist_x = landmarks.landmark[mp_hands.HandLandmark.WRIST].x
            if wrist_x > 0.5:
                right_hand = landmarks.landmark
            else:
                left_hand = landmarks.landmark
            
            mp_draw.draw_landmarks(frame, landmarks, mp_hands.HAND_CONNECTIONS)

    # 2. Analyze Gestures
    grip_cmd = GestureType.OPEN_HAND
    base_cmd = GestureType.STOP_MOVE_HORIZONTAL
    sh_cmd = GestureType.STOP_MOVE_VERTICAL
    el_angle, wr_angle =  45, 45 
    debug_str = "No Hands"

    if mode == "BIMANUAL" and right_hand and left_hand:
        grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str = \
            recognizer.analyze_bimanual(right_hand, left_hand)
            
    elif mode == "UNIMANUAL" and right_hand:
        grip_cmd, base_cmd, sh_cmd, el_angle, wr_angle, debug_str = \
            recognizer.analyze_unimanual(right_hand)
            
    elif right_hand: debug_str = "BIMANUAL: Waiting for Left Hand"
    elif left_hand: debug_str = "BIMANUAL: Waiting for Right Hand"

    # 3. Send to Simulator
    if base_cmd == GestureType.MOVE_LEFT: base_angle += BASE_ROTATION_SPEED
    elif base_cmd == GestureType.MOVE_RIGHT: base_angle -= BASE_ROTATION_SPEED

    print('command: ', sh_cmd, sh_angle)

    if sh_cmd == GestureType.MOVE_UP: sh_angle += BASE_ROTATION_SPEED
    elif sh_cmd == GestureType.MOVE_DOWN: sh_angle -= BASE_ROTATION_SPEED
    
    base_angle = max(-150, min(150, base_angle))
    sim.set_joint_angle("base_joint", base_angle)
    

    sh_angle = max(-150, min(150, sh_angle))
    sim.set_joint_angle("shoulder_joint", sh_angle)

    # ignore other joints for now
    # sim.set_joint_angle("elbow_joint", el_angle)
    # sim.set_joint_angle("wrist_joint", wr_angle)
    
    # --- Gripper Logic ---
    # 1. Animate the gripper fingers for visual effect
    # grip_target_angle = 40 if (grip_cmd == GestureType.CLOSED_HAND) else 0
    # sim.set_joint_angle("left_gripper_joint", grip_target_angle)
    # sim.set_joint_angle("right_gripper_joint", grip_target_angle)

    # 2. Handle the picking/placing physics
    # grip_command_str = "CLOSE" if grip_cmd == GestureType.CLOSED_HAND else "OPEN"
    # sim.control_gripper(grip_command_str)

    sim.step()

    # --- Visual Markers ---
    cv2.line(frame, (frame_width // 2, 0), (frame_width // 2, frame_height), (0, 255, 255), 2)
    cv2.line(frame, (0, int(frame_height * 0.2)), (frame_width, int(frame_height * 0.2)), (255, 0, 255), 1)
    cv2.line(frame, (0, int(frame_height * 0.8)), (frame_width, int(frame_height * 0.8)), (255, 0, 255), 1)
    cv2.line(frame, (int(frame_width * 0.7), 0), (int(frame_width * 0.7), frame_height), (255, 100, 100), 1)
    cv2.line(frame, (int(frame_width * 0.8), 0), (int(frame_width * 0.8), frame_height), (255, 100, 100), 1)

    # 4. Display UI
    cv2.rectangle(frame, (0,0), (frame_width, 60), (0,0,0), -1)
    cv2.putText(frame, f"MODE: {mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, debug_str, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    cv2.imshow("Robot Simulation Controller", frame)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'): break
    if key & 0xFF == ord('u'): mode = "UNIMANUAL"
    if key & 0xFF == ord('b'): mode = "BIMANUAL"
    # --- NEW: Handle reset key press ---
    if key & 0xFF == ord('r'):
        sim.reset()
        base_angle = 0.0 # Also reset the controller's internal state
        sh_angle = 0.0

cap.release()
sim.close()
cv2.destroyAllWindows()