import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import mediapipe as mp
import csv
import os
import math

# =============================
# PYBULLET SETUP
# =============================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)

p.setPhysicsEngineParameter(
    fixedTimeStep=1.0 / 240.0,
    numSolverIterations=200,
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.4,
    cameraYaw=90,
    cameraPitch=-25,
    cameraTargetPosition=[0.5, 0, 0.3]
)

p.loadURDF("plane.urdf")

# =============================
# LOAD PANDA
# =============================
robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

joint_name_to_id = {}
link_name_to_id = {}
HAND_LINK = None

for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    jname = info[1].decode()
    lname = info[12].decode()
    joint_name_to_id[jname] = i
    link_name_to_id[lname] = i
    if lname == "panda_link8":
        HAND_LINK = i

BASE_J     = joint_name_to_id["panda_joint1"]
SHOULDER_J = joint_name_to_id["panda_joint2"]
ELBOW_J    = joint_name_to_id["panda_joint4"]
WRIST_J    = joint_name_to_id["panda_joint6"]
FINGER_L_J = joint_name_to_id["panda_finger_joint1"]
FINGER_R_J = joint_name_to_id["panda_finger_joint2"]

LEFT_FINGER_LINK  = link_name_to_id.get("panda_leftfinger", None)
RIGHT_FINGER_LINK = link_name_to_id.get("panda_rightfinger", None)

if HAND_LINK is None:
    raise RuntimeError("Could not find panda_link8")

LOCKED_JOINTS = ["panda_joint3", "panda_joint5", "panda_joint7"]

# =============================
# NEUTRAL POSTURE
# =============================
neutral = {
    "panda_joint1": 0.0,
    "panda_joint2": -0.4,
    "panda_joint3": 0.0,
    "panda_joint4": -2.0,
    "panda_joint5": 0.0,
    "panda_joint6": 1.7,
    "panda_joint7": 0.8,
}
for j, v in neutral.items():
    p.resetJointState(robot, joint_name_to_id[j], v)

# =============================
# TABLES
# =============================
def create_table(x, y, top_z):
    half = [0.22, 0.22, 0.05]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[0.75, 0.75, 0.75, 1])
    p.createMultiBody(0, col, vis, [x, y, top_z - half[2]])

TABLE_X = 0.48
PICK_Y, PLACE_Y = -0.28, 0.28
TABLE_Z = 0.26

create_table(TABLE_X, PICK_Y, TABLE_Z)
create_table(TABLE_X, PLACE_Y, TABLE_Z)

# =============================
# CUBE + TARGET
# =============================
CUBE_SIZE = 0.04
cube_start = [TABLE_X, PICK_Y + 0.03, TABLE_Z + CUBE_SIZE / 2]
cube = p.loadURDF("cube_small.urdf", cube_start, globalScaling=0.8)

TARGET_POS = np.array([TABLE_X, PLACE_Y - 0.03, TABLE_Z + CUBE_SIZE / 2])
target_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.002, rgbaColor=[1, 0, 0, 0.7])
p.createMultiBody(0, baseVisualShapeIndex=target_vis, basePosition=TARGET_POS)

# =============================
# MEDIAPIPE & CONTROLS
# =============================
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7, # Increased for stability
    min_tracking_confidence=0.5
)
cap = cv2.VideoCapture(0)

JOINT_VEL = {"BASE": 1.4, "SHOULDER": 1.2, "WRIST": 1.5, "ELBOW": 1.3}
DEADZONE_TOP = 0.45
DEADZONE_BOTTOM = 0.55
ARM_FORCE = 90
GRIP_FORCE = 100

# --- IMPROVED GRIP SETTINGS ---
# Instead of raw distance, we use (Pinch Dist / Palm Size)
# < 0.30 means fingers are touching
# > 0.60 means fingers are well spread
PINCH_THRESHOLD_CLOSE = 0.35 
PINCH_THRESHOLD_OPEN  = 0.60 

# Smoothing factor (0.0 to 1.0). Higher = smoother but more lag.
SMOOTHING_ALPHA = 0.6 
current_pinch_ratio = 1.0 # Start open

# =============================
# ASSISTED GRASP SETTINGS
# =============================
GRASP_XY_THRESH = 0.065
GRASP_Z_THRESH  = 0.055

HALO_RADIUS = 0.075
halo_vis = p.createVisualShape(p.GEOM_SPHERE, radius=HALO_RADIUS, rgbaColor=[1, 0, 0, 0.18])
halo_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=halo_vis, basePosition=cube_start)

# =============================
# LOGGING & VARS
# =============================
PARTICIPANT_ID = "P01"
LOG_FILE = f"results_{PARTICIPANT_ID}_unimanual.csv"
SUCCESS_THRESH = 0.03

if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow(["participant", "trial", "time", "placement_error", "success"])

trial = 0
trial_start = time.time()
cube_attached = False
cid = None

def get_grasp_point():
    if LEFT_FINGER_LINK is not None and RIGHT_FINGER_LINK is not None:
        lf = np.array(p.getLinkState(robot, LEFT_FINGER_LINK)[0])
        rf = np.array(p.getLinkState(robot, RIGHT_FINGER_LINK)[0])
        return (lf + rf) * 0.5
    return np.array(p.getLinkState(robot, HAND_LINK)[0])

def calc_pixel_dist(p1, p2, w, h):
    """Calculates Euclidean distance in pixels."""
    x1, y1 = p1.x * w, p1.y * h
    x2, y2 = p2.x * w, p2.y * h
    return math.hypot(x2 - x1, y2 - y1)

# =============================
# MAIN LOOP
# =============================
grip_state = 0.04 # 0.04 = Open, 0.0 = Closed

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    # UI Lines
    cv2.rectangle(frame, (0, int(h*DEADZONE_TOP)), (w, int(h*DEADZONE_BOTTOM)), (60,60,60), -1)
    
    joint_vel = {BASE_J: 0.0, SHOULDER_J: 0.0, WRIST_J: 0.0, ELBOW_J: 0.0}
    active_region = "NONE"
    
    # Process Hands
    res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    if res.multi_hand_landmarks:
        hand_lms = res.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_lms, mp_hands.HAND_CONNECTIONS)
        lm = hand_lms.landmark

        # 1. Navigation Control
        cx = np.mean([pt.x for pt in lm])
        cy = np.mean([pt.y for pt in lm])

        if cx < 0.25:
            active_region = "BASE";     jid = BASE_J
        elif cx < 0.50:
            active_region = "SHOULDER"; jid = SHOULDER_J
        elif cx < 0.75:
            active_region = "WRIST";    jid = WRIST_J
        else:
            active_region = "ELBOW";    jid = ELBOW_J

        if cy < DEADZONE_TOP:
            joint_vel[jid] = +JOINT_VEL[active_region]
        elif cy > DEADZONE_BOTTOM:
            joint_vel[jid] = -JOINT_VEL[active_region]

        # ---------------------------------------------------------
        # IMPROVED GRIP DETECTION LOGIC
        # ---------------------------------------------------------
        
        # Points of interest
        thumb_tip = lm[mp_hands.HandLandmark.THUMB_TIP]
        index_tip = lm[mp_hands.HandLandmark.INDEX_FINGER_TIP]
        wrist     = lm[mp_hands.HandLandmark.WRIST]
        index_mcp = lm[mp_hands.HandLandmark.INDEX_FINGER_MCP] # Knuckle

        # A. Calculate Distances in Pixels (fixes aspect ratio issues)
        pinch_dist = calc_pixel_dist(thumb_tip, index_tip, w, h)
        
        # B. Calculate Hand Reference Scale (Wrist to Index Knuckle)
        # This distance changes proportionally when you move hand closer/further
        scale_ref = calc_pixel_dist(wrist, index_mcp, w, h)
        if scale_ref < 1.0: scale_ref = 1.0 # prevent div by zero

        # C. Calculate Ratio (Scale Invariant!)
        instant_ratio = pinch_dist / scale_ref

        # D. Smoothing (Exponential Moving Average)
        current_pinch_ratio = (SMOOTHING_ALPHA * current_pinch_ratio) + \
                              ((1 - SMOOTHING_ALPHA) * instant_ratio)

        # E. Hysteresis Logic (Latch)
        if current_pinch_ratio < PINCH_THRESHOLD_CLOSE:
            grip_state = 0.0 # Close
            color = (0, 255, 0) # Green for active grip
        elif current_pinch_ratio > PINCH_THRESHOLD_OPEN:
            grip_state = 0.04 # Open
            color = (0, 0, 255) # Red for open
        else:
            # In between state - keep previous state
            color = (0, 255, 255) # Yellow for hysteresis zone

        # Visual Debug for Pinch
        # Draw line between fingers
        pt1 = (int(thumb_tip.x * w), int(thumb_tip.y * h))
        pt2 = (int(index_tip.x * w), int(index_tip.y * h))
        cv2.line(frame, pt1, pt2, color, 3)
        cv2.putText(frame, f"R: {current_pinch_ratio:.2f}", pt1, 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # ---------- Apply Control ----------
    p.setJointMotorControl2(robot, BASE_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[BASE_J], force=ARM_FORCE)
    p.setJointMotorControl2(robot, SHOULDER_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[SHOULDER_J], force=ARM_FORCE)
    p.setJointMotorControl2(robot, WRIST_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[WRIST_J], force=ARM_FORCE)
    p.setJointMotorControl2(robot, ELBOW_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[ELBOW_J], force=ARM_FORCE)

    # Gripper Control
    p.setJointMotorControl2(robot, FINGER_L_J, p.POSITION_CONTROL, grip_state, force=GRIP_FORCE)
    p.setJointMotorControl2(robot, FINGER_R_J, p.POSITION_CONTROL, grip_state, force=GRIP_FORCE)

    # Keep locked joints steady
    for jn in LOCKED_JOINTS:
        p.setJointMotorControl2(robot, joint_name_to_id[jn], p.POSITION_CONTROL, neutral[jn], force=ARM_FORCE)

    # ---------- Assisted Snap Grasp ----------
    cube_pos, _ = p.getBasePositionAndOrientation(cube)
    grasp_point = get_grasp_point()

    # Distance calc
    xy_dist = float(np.linalg.norm(grasp_point[:2] - np.array(cube_pos)[:2]))
    z_dist  = float(abs(grasp_point[2] - cube_pos[2]))
    graspable = (xy_dist < GRASP_XY_THRESH) and (z_dist < GRASP_Z_THRESH)

    # Visual Halo Update
    halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
    p.changeVisualShape(halo_body, -1, rgbaColor=halo_color)
    p.resetBasePositionAndOrientation(halo_body, cube_pos, [0, 0, 0, 1])

    # Logic: If Gripper Closed (0.0) AND Graspable AND Not yet attached -> Attach
    if grip_state < 0.01 and not cube_attached and graspable:
        p.resetBasePositionAndOrientation(cube, grasp_point.tolist(), [0, 0, 0, 1])
        cid = p.createConstraint(robot, HAND_LINK, cube, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
        cube_attached = True

# Logic: If Gripper Open (0.04) AND Attached -> Release
    if grip_state > 0.02 and cube_attached:
        p.removeConstraint(cid)
        cube_attached = False
        
        # --- NEW: Let the object fall and settle for 1 second ---
        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240)
        # --------------------------------------------------------

        # Log Result (now that it has settled)
        final_pos = np.array(p.getBasePositionAndOrientation(cube)[0])
        err = np.linalg.norm(final_pos - TARGET_POS)
        
        duration = time.time() - trial_start
        with open(LOG_FILE, "a", newline="") as f:
            csv.writer(f).writerow([PARTICIPANT_ID, trial, round(duration, 3), round(err, 4), int(err < SUCCESS_THRESH)])
        
        # Reset for next trial
        trial += 1
        trial_start = time.time()
        p.resetBasePositionAndOrientation(cube, cube_start, [0, 0, 0, 1])

    # HUD
    cv2.putText(frame, f"ACTIVE: {active_region}", (10, h - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Control", frame)
    
    if cv2.waitKey(1) & 0xFF == 27:
        break

    p.stepSimulation()
    time.sleep(1 / 240)

cap.release()
cv2.destroyAllWindows()
p.disconnect()