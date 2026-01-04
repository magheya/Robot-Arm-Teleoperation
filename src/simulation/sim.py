import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import mediapipe as mp
import csv
import os

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

# These are LINKS (not joints) for fingertips — better grasp point
LEFT_FINGER_LINK  = link_name_to_id.get("panda_leftfinger", None)
RIGHT_FINGER_LINK = link_name_to_id.get("panda_rightfinger", None)

if HAND_LINK is None:
    raise RuntimeError("Could not find panda_link8")

if LEFT_FINGER_LINK is None or RIGHT_FINGER_LINK is None:
    # fallback: use hand link (still works), but less accurate
    print("[WARN] Could not find panda_leftfinger / panda_rightfinger links. Falling back to panda_link8 for grasp point.")

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
# TABLES (SAME HEIGHT) — Layout B (closer + less lateral)
# =============================
def create_table(x, y, top_z):
    half = [0.22, 0.22, 0.05]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=half,
        rgbaColor=[0.75, 0.75, 0.75, 1]
    )
    p.createMultiBody(0, col, vis, [x, y, top_z - half[2]])

# ---- Layout B (recommended) ----
TABLE_X = 0.48              # was 0.55 (too far)
PICK_Y, PLACE_Y = -0.28, 0.28  # was ±0.35 (too wide)
TABLE_Z = 0.26

create_table(TABLE_X, PICK_Y, TABLE_Z)
create_table(TABLE_X, PLACE_Y, TABLE_Z)

# =============================
# CUBE + TARGET
# =============================
CUBE_SIZE = 0.04

# place cube slightly toward table center (optional but helps)
cube_start = [TABLE_X, PICK_Y + 0.03, TABLE_Z + CUBE_SIZE / 2]
cube = p.loadURDF("cube_small.urdf", cube_start, globalScaling=0.8)

TARGET_POS = np.array([TABLE_X, PLACE_Y - 0.03, TABLE_Z + CUBE_SIZE / 2])
target_vis = p.createVisualShape(
    p.GEOM_CYLINDER, radius=0.05, length=0.002,
    rgbaColor=[1, 0, 0, 0.7]
)
p.createMultiBody(0, baseVisualShapeIndex=target_vis, basePosition=TARGET_POS)

# =============================
# MEDIAPIPE
# =============================
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1)
cap = cv2.VideoCapture(0)

# =============================
# VELOCITY CONTROL PARAMETERS
# =============================
JOINT_VEL = {
    "BASE":     1.4,
    "SHOULDER": 1.2,
    "WRIST":    1.5,
    "ELBOW":    1.3,
}

DEADZONE_TOP = 0.45
DEADZONE_BOTTOM = 0.55

PINCH_CLOSE = 0.04
PINCH_OPEN  = 0.07

ARM_FORCE = 90
GRIP_FORCE = 100

# =============================
# ASSISTED GRASP (OPTION A)
# XY + Z threshold (NOT a sphere)
# =============================
# Widened slightly for robustness with webcam control
GRASP_XY_THRESH = 0.065   # was 0.045
GRASP_Z_THRESH  = 0.055   # was 0.035

# Visual halo
HALO_RADIUS = 0.075
halo_vis = p.createVisualShape(
    p.GEOM_SPHERE,
    radius=HALO_RADIUS,
    rgbaColor=[1, 0, 0, 0.18]
)
halo_body = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=halo_vis,
    basePosition=cube_start
)

# =============================
# EXPERIMENT LOGGING
# =============================
PARTICIPANT_ID = "P01"
LOG_FILE = f"results_{PARTICIPANT_ID}_unimanual.csv"
SUCCESS_THRESH = 0.03

if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow(
            ["participant", "trial", "time", "placement_error", "success"]
        )

trial = 0
trial_start = time.time()
cube_attached = False
cid = None

def get_grasp_point():
    """
    Use midpoint of fingertip LINKS (best). If not available, fallback to hand link.
    """
    if LEFT_FINGER_LINK is not None and RIGHT_FINGER_LINK is not None:
        lf = np.array(p.getLinkState(robot, LEFT_FINGER_LINK)[0])
        rf = np.array(p.getLinkState(robot, RIGHT_FINGER_LINK)[0])
        return (lf + rf) * 0.5
    return np.array(p.getLinkState(robot, HAND_LINK)[0])

# =============================
# MAIN LOOP
# =============================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    # ---------- UI: regions ----------
    splits = [0.25, 0.50, 0.75]
    labels = ["BASE", "SHOULDER", "WRIST", "ELBOW"]
    for s in splits:
        cv2.line(frame, (int(w*s), 0), (int(w*s), h), (255,255,255), 2)
    for i,l in enumerate(labels):
        cv2.putText(frame, l,
            (int(w*(i*0.25+0.125))-45, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    # ---------- UI: deadzone ----------
    top_y = int(h * DEADZONE_TOP)
    bot_y = int(h * DEADZONE_BOTTOM)
    cv2.rectangle(frame, (0, top_y), (w, bot_y), (60,60,60), -1)
    cv2.line(frame, (0, top_y), (w, top_y), (0,255,0), 2)
    cv2.line(frame, (0, bot_y), (w, bot_y), (0,255,0), 2)

    # ---------- Default: zero velocity ----------
    joint_vel = {BASE_J: 0.0, SHOULDER_J: 0.0, WRIST_J: 0.0, ELBOW_J: 0.0}

    # ---------- Hand tracking ----------
    res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    active_region = "NONE"
    grip = 0.04
    pinch_val = None

    if res.multi_hand_landmarks:
        mp_draw.draw_landmarks(frame, res.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)

        lm = res.multi_hand_landmarks[0].landmark
        cx = np.mean([pt.x for pt in lm])
        cy = np.mean([pt.y for pt in lm])

        # Region selection
        if cx < 0.25:
            active_region = "BASE";     jid = BASE_J
        elif cx < 0.50:
            active_region = "SHOULDER"; jid = SHOULDER_J
        elif cx < 0.75:
            active_region = "WRIST";    jid = WRIST_J
        else:
            active_region = "ELBOW";    jid = ELBOW_J

        # Direction (UP/DOWN)
        if cy < DEADZONE_TOP:
            joint_vel[jid] = +JOINT_VEL[active_region]
        elif cy > DEADZONE_BOTTOM:
            joint_vel[jid] = -JOINT_VEL[active_region]

        # Pinch → gripper (works in ANY region)
        thumb = lm[mp_hands.HandLandmark.THUMB_TIP]
        index = lm[mp_hands.HandLandmark.INDEX_FINGER_TIP]
        pinch_val = float(np.linalg.norm(
            np.array([thumb.x, thumb.y]) - np.array([index.x, index.y])
        ))

        if pinch_val < PINCH_CLOSE:
            grip = 0.0
        elif pinch_val > PINCH_OPEN:
            grip = 0.04

    # ---------- Apply VELOCITY control ----------
    p.setJointMotorControl2(robot, BASE_J, p.VELOCITY_CONTROL,
                            targetVelocity=joint_vel[BASE_J], force=ARM_FORCE)
    p.setJointMotorControl2(robot, SHOULDER_J, p.VELOCITY_CONTROL,
                            targetVelocity=joint_vel[SHOULDER_J], force=ARM_FORCE)
    p.setJointMotorControl2(robot, WRIST_J, p.VELOCITY_CONTROL,
                            targetVelocity=joint_vel[WRIST_J], force=ARM_FORCE)
    p.setJointMotorControl2(robot, ELBOW_J, p.VELOCITY_CONTROL,
                            targetVelocity=joint_vel[ELBOW_J], force=ARM_FORCE)

    p.setJointMotorControl2(robot, FINGER_L_J, p.POSITION_CONTROL, grip, force=GRIP_FORCE)
    p.setJointMotorControl2(robot, FINGER_R_J, p.POSITION_CONTROL, grip, force=GRIP_FORCE)

    for jn in LOCKED_JOINTS:
        jid = joint_name_to_id[jn]
        p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, neutral[jn], force=ARM_FORCE)

    # ---------- Assisted snap grasp (XY + Z) using fingertip midpoint ----------
    cube_pos, _ = p.getBasePositionAndOrientation(cube)
    cube_pos_np = np.array(cube_pos)

    grasp_point = get_grasp_point()

    xy_dist = float(np.linalg.norm(grasp_point[:2] - cube_pos_np[:2]))
    z_dist  = float(abs(grasp_point[2] - cube_pos_np[2]))
    graspable = (xy_dist < GRASP_XY_THRESH) and (z_dist < GRASP_Z_THRESH)

    # Update halo color + position
    halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
    p.changeVisualShape(halo_body, -1, rgbaColor=halo_color)
    p.resetBasePositionAndOrientation(halo_body, cube_pos, [0, 0, 0, 1])

    # Snap grasp ONLY when user closes gripper AND graspable
    if grip < 0.01 and not cube_attached and graspable:
        # small “assist”: align cube to grasp point before fixing (optional but helps)
        p.resetBasePositionAndOrientation(cube, grasp_point.tolist(), [0, 0, 0, 1])

        cid = p.createConstraint(
            robot, HAND_LINK, cube, -1,
            p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0]
        )
        cube_attached = True

    # Release ends trial
    if grip > 0.02 and cube_attached:
        p.removeConstraint(cid)
        cube_attached = False

        err = np.linalg.norm(np.array(p.getBasePositionAndOrientation(cube)[0]) - TARGET_POS)
        duration = time.time() - trial_start

        with open(LOG_FILE, "a", newline="") as f:
            csv.writer(f).writerow(
                [PARTICIPANT_ID, trial, round(duration, 3), round(err, 4), int(err < SUCCESS_THRESH)]
            )

        trial += 1
        trial_start = time.time()
        p.resetBasePositionAndOrientation(cube, cube_start, [0, 0, 0, 1])

    # ---------- HUD ----------
    cv2.putText(frame, f"ACTIVE: {active_region}", (10, h - 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    if pinch_val is not None:
        cv2.putText(frame, f"PINCH: {pinch_val:.3f} | GRIP={'CLOSE' if grip < 0.01 else 'OPEN'}",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "PINCH: --", (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Unimanual Gesture Control – Velocity + Assisted Grasp", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

    p.stepSimulation()
    time.sleep(1 / 240)

cap.release()
cv2.destroyAllWindows()
p.disconnect()
