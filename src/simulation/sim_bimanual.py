import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import mediapipe as mp
import csv
import os

# ============================================================
# PYBULLET SETUP
# ============================================================
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

# ============================================================
# LOAD PANDA
# ============================================================
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

# Fingertip LINKS (better grasp point)
LEFT_FINGER_LINK  = link_name_to_id.get("panda_leftfinger", None)
RIGHT_FINGER_LINK = link_name_to_id.get("panda_rightfinger", None)

if HAND_LINK is None:
    raise RuntimeError("Could not find panda_link8")

if LEFT_FINGER_LINK is None or RIGHT_FINGER_LINK is None:
    print("[WARN] Could not find panda_leftfinger / panda_rightfinger links. Falling back to panda_link8 for grasp point.")

LOCKED_JOINTS = ["panda_joint3", "panda_joint5", "panda_joint7"]

# ============================================================
# NEUTRAL POSTURE
# ============================================================
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

# ============================================================
# TABLES (Layout B)  (UNCHANGED)
# ============================================================
def create_table(x, y, top_z):
    half = [0.22, 0.22, 0.05]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=half,
        rgbaColor=[0.75, 0.75, 0.75, 1]
    )
    p.createMultiBody(0, col, vis, [x, y, top_z - half[2]])

TABLE_X = 0.48
PICK_Y, PLACE_Y = -0.28, 0.28
TABLE_Z = 0.26

create_table(TABLE_X, PICK_Y, TABLE_Z)
create_table(TABLE_X, PLACE_Y, TABLE_Z)

# ============================================================
# CUBE + TARGET (UPDATED: keep IDs so we can move them per trial)
# ============================================================
CUBE_SIZE = 0.05

BASE_CUBE_POS = np.array([TABLE_X, PICK_Y + 0.03, TABLE_Z + CUBE_SIZE / 2], dtype=float)
BASE_TARGET_POS = np.array([TABLE_X, PLACE_Y - 0.03, TABLE_Z + CUBE_SIZE / 2], dtype=float)

cube = p.loadURDF("cube_small.urdf", BASE_CUBE_POS.tolist(), globalScaling=0.8)

target_vis = p.createVisualShape(
    p.GEOM_CYLINDER, radius=0.05, length=0.002,
    rgbaColor=[1, 0, 0, 0.7]
)
target_body = p.createMultiBody(0, baseVisualShapeIndex=target_vis, basePosition=BASE_TARGET_POS.tolist())

# ============================================================
# MEDIAPIPE (BIMANUAL)
# ============================================================
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

# ============================================================
# VELOCITY CONTROL PARAMETERS
# ============================================================
JOINT_VEL = {
    "BASE":     1.4,
    "SHOULDER": 1.2,
    "WRIST":    1.5,
    "ELBOW":    1.3,
}

# Bigger deadzone (transparent band)
DEADZONE_TOP = 0.42
DEADZONE_BOTTOM = 0.58

# Pinch thresholds
PINCH_CLOSE = 0.04
PINCH_OPEN  = 0.07

ARM_FORCE = 90
GRIP_FORCE = 100

# ============================================================
# ASSISTED GRASP
# ============================================================
GRASP_XY_THRESH = 0.065
GRASP_Z_THRESH  = 0.055

HALO_RADIUS = 0.075
halo_vis = p.createVisualShape(
    p.GEOM_SPHERE,
    radius=HALO_RADIUS,
    rgbaColor=[1, 0, 0, 0.18]
)
halo_body = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=halo_vis,
    basePosition=BASE_CUBE_POS.tolist()
)

# ============================================================
# EXPERIMENT LOGGING (FINAL: add factor + difficulty + D)
# ============================================================
PARTICIPANT_ID = "P01"
LOG_FILE = f"results_{PARTICIPANT_ID}_bimanual.csv"
SUCCESS_THRESH = 0.05

# ------------------------------------------------------------
# FACTOR A: Transport distance (D)
#   - cube fixed (easy grasp)
#   - target moves (easy/medium/hard)
# FACTOR B: Grasp difficulty
#   - target fixed
#   - cube moves (easy/medium/hard)
#
# 2 reps each difficulty => 6 trials per factor => 12 total trials
# ------------------------------------------------------------
N_REPS_PER_LEVEL = 2
LEVELS = ["easy", "medium", "hard"]

TARGET_Y_BY_LEVEL = {
    "easy":   float(BASE_TARGET_POS[1] - 0.10),
    "medium": float(BASE_TARGET_POS[1]),
    "hard":   float(BASE_TARGET_POS[1] + 0.10),
}

CUBE_POS_BY_LEVEL = {
    "easy":   np.array([TABLE_X - 0.06, PICK_Y + 0.02, BASE_CUBE_POS[2]], dtype=float),
    "medium": BASE_CUBE_POS.copy(),
    "hard":   np.array([TABLE_X + 0.06, PICK_Y - 0.06, BASE_CUBE_POS[2]], dtype=float),
}

TRIAL_PLAN = []
for _ in range(N_REPS_PER_LEVEL):
    for lvl in LEVELS:
        TRIAL_PLAN.append({"factor": "transport_distance", "difficulty": lvl})
for _ in range(N_REPS_PER_LEVEL):
    for lvl in LEVELS:
        TRIAL_PLAN.append({"factor": "grasp_difficulty", "difficulty": lvl})

N_TRIALS = len(TRIAL_PLAN)

# New metrics:
# - time_to_grasp: time from trial start to first attachment
# - grasp_attempts: number of pinch-close events during trial
# - drops: number of times cube was released while not at target (unintended drops)

if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow([
            "participant",
            "trial",
            "factor",
            "difficulty",
            "D",
            "time_total",
            "time_to_grasp",
            "placement_error",
            "success",
            "grasp_attempts",
            "drops",
            "cube_x", "cube_y", "cube_z",
            "target_x", "target_y", "target_z",
        ])

trial = 0
trial_start = time.time()
cube_attached = False
cid = None

# per-trial counters/state
grasp_attempts = 0
drops = 0
time_to_grasp = None

# current condition vars (updated per trial)
current_factor = TRIAL_PLAN[0]["factor"]
current_difficulty = TRIAL_PLAN[0]["difficulty"]
current_cube_pos = BASE_CUBE_POS.copy()
current_target_pos = BASE_TARGET_POS.copy()
current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

def reset_trial_state():
    global trial_start, cube_attached, cid, grasp_attempts, drops, time_to_grasp
    trial_start = time.time()
    cube_attached = False
    cid = None
    grasp_attempts = 0
    drops = 0
    time_to_grasp = None

def near_target(pos, target, thresh):
    return float(np.linalg.norm(np.array(pos) - np.array(target))) < float(thresh)

def get_grasp_point():
    """Midpoint of fingertip LINKS (best). If not available, fallback to hand link position."""
    if LEFT_FINGER_LINK is not None and RIGHT_FINGER_LINK is not None:
        lf = np.array(p.getLinkState(robot, LEFT_FINGER_LINK)[0])
        rf = np.array(p.getLinkState(robot, RIGHT_FINGER_LINK)[0])
        return (lf + rf) * 0.5
    return np.array(p.getLinkState(robot, HAND_LINK)[0])

def create_fixed_constraint_preserve_pose(parent_body, parent_link, child_body):
    """Fixed constraint preserving current relative pose (prevents cube snapping into wrist)."""
    ee_pos, ee_orn = p.getLinkState(parent_body, parent_link)[:2]
    cube_pos, cube_orn = p.getBasePositionAndOrientation(child_body)

    inv_pos, inv_orn = p.invertTransform(ee_pos, ee_orn)
    parent_frame_pos, parent_frame_orn = p.multiplyTransforms(inv_pos, inv_orn, cube_pos, cube_orn)

    return p.createConstraint(
        parentBodyUniqueId=parent_body,
        parentLinkIndex=parent_link,
        childBodyUniqueId=child_body,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=parent_frame_pos,
        childFramePosition=[0, 0, 0],
        parentFrameOrientation=parent_frame_orn,
        childFrameOrientation=[0, 0, 0, 1],
    )

def set_trial_condition(trial_idx: int):
    """
    Apply the trial condition:
      - transport_distance: cube fixed, target moves
      - grasp_difficulty: target fixed, cube moves
    """
    global current_factor, current_difficulty, current_cube_pos, current_target_pos, current_D

    cond = TRIAL_PLAN[trial_idx]
    current_factor = cond["factor"]
    current_difficulty = cond["difficulty"]

    current_cube_pos = BASE_CUBE_POS.copy()
    current_target_pos = BASE_TARGET_POS.copy()

    if current_factor == "transport_distance":
        current_target_pos = np.array(
            [BASE_TARGET_POS[0], TARGET_Y_BY_LEVEL[current_difficulty], BASE_TARGET_POS[2]],
            dtype=float
        )
    elif current_factor == "grasp_difficulty":
        current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty].copy()

    # Apply in sim
    p.resetBasePositionAndOrientation(cube, current_cube_pos.tolist(), [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(target_body, current_target_pos.tolist(), [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(halo_body, current_cube_pos.tolist(), [0, 0, 0, 1])

    current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

# ============================================================
# BIMANUAL RULE
# Left hand controls LEFT TWO regions: BASE + SHOULDER
# Right hand controls RIGHT TWO regions: WRIST + ELBOW
# (Same 4 splits on screen for fair comparison)
# ============================================================
pinch_state = "OPEN"      # OPEN / CLOSED
release_start_time = None
RELEASE_DELAY = 0.6

def hand_center(lms):
    xs = [pt.x for pt in lms]
    ys = [pt.y for pt in lms]
    return float(np.mean(xs)), float(np.mean(ys))

def pinch_value(lms):
    thumb = lms[mp_hands.HandLandmark.THUMB_TIP]
    index = lms[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    return float(np.linalg.norm(np.array([thumb.x, thumb.y]) - np.array([index.x, index.y])))

def region_from_cx(cx):
    if cx < 0.25:  return "BASE"
    if cx < 0.50:  return "SHOULDER"
    if cx < 0.75:  return "WRIST"
    return "ELBOW"

# Initialize first trial
set_trial_condition(0)
reset_trial_state()

# ============================================================
# MAIN LOOP
# ============================================================
try:
    while True:
        if trial >= N_TRIALS:
            print(f"[DONE] Completed {N_TRIALS} trials.")
            break

        if not p.isConnected():
            print("[ERROR] PyBullet disconnected (GUI closed/crashed). Exiting cleanly.")
            break

        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # ---------- UI: 4 regions ----------
        r1, r2, r3 = int(w * 0.25), int(w * 0.50), int(w * 0.75)
        cv2.line(frame, (r1, 0), (r1, h), (255, 255, 255), 2)
        cv2.line(frame, (r2, 0), (r2, h), (255, 255, 255), 2)
        cv2.line(frame, (r3, 0), (r3, h), (255, 255, 255), 2)

        labels = ["BASE", "SHOULDER", "WRIST", "ELBOW"]
        for i, label in enumerate(labels):
            cv2.putText(frame, label,
                        (int(w * (i * 0.25 + 0.125)) - 45, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)

        # ---------- UI: deadzone (transparent) ----------
        top_y = int(h * DEADZONE_TOP)
        bot_y = int(h * DEADZONE_BOTTOM)

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, top_y), (w, bot_y), (60, 60, 60), -1)
        frame = cv2.addWeighted(overlay, 0.22, frame, 0.78, 0)
        cv2.line(frame, (0, top_y), (w, top_y), (0, 255, 0), 2)
        cv2.line(frame, (0, bot_y), (w, bot_y), (0, 255, 0), 2)

        # ---------- Default velocities = 0 ----------
        joint_vel = {BASE_J: 0.0, SHOULDER_J: 0.0, WRIST_J: 0.0, ELBOW_J: 0.0}

        # ---------- MediaPipe processing ----------
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        active_left = "NONE"
        active_right = "NONE"
        pinch_val_best = None

        pinch_close_event = False
        pinch_open_event = False

        pinch_candidates = []

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_lm, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                mp_draw.draw_landmarks(frame, hand_lm, mp_hands.HAND_CONNECTIONS)

                label = handedness.classification[0].label  # "Left" or "Right"
                lms = hand_lm.landmark

                cx, cy = hand_center(lms)
                px, py = int(cx * w), int(cy * h)
                cv2.circle(frame, (px, py), 6, (0, 255, 0), -1)

                region = region_from_cx(cx)
                cv2.putText(frame, f"{label}: {region}",
                            (px - 55, py - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0), 2)

                # Direction based on deadzone
                direction = 0
                if cy < DEADZONE_TOP:
                    direction = +1
                elif cy > DEADZONE_BOTTOM:
                    direction = -1

                # Enforce mapping
                if label == "Left":
                    if region == "BASE":
                        joint_vel[BASE_J] = direction * JOINT_VEL["BASE"]
                        active_left = "BASE"
                    elif region == "SHOULDER":
                        joint_vel[SHOULDER_J] = direction * JOINT_VEL["SHOULDER"]
                        active_left = "SHOULDER"

                elif label == "Right":
                    if region == "WRIST":
                        joint_vel[WRIST_J] = direction * JOINT_VEL["WRIST"]
                        active_right = "WRIST"
                    elif region == "ELBOW":
                        joint_vel[ELBOW_J] = direction * JOINT_VEL["ELBOW"]
                        active_right = "ELBOW"

                # Pinch candidates (either hand)
                pinch_candidates.append(pinch_value(lms))

        # Use the most "closed" pinch (min distance)
        if pinch_candidates:
            pinch_val_best = float(min(pinch_candidates))

            # Close immediately
            if pinch_state == "OPEN" and pinch_val_best < PINCH_CLOSE:
                pinch_state = "CLOSED"
                pinch_close_event = True
                release_start_time = None

            # Open only if above threshold for RELEASE_DELAY
            elif pinch_state == "CLOSED":
                if pinch_val_best > PINCH_OPEN:
                    if release_start_time is None:
                        release_start_time = time.time()
                    elif (time.time() - release_start_time) > RELEASE_DELAY:
                        pinch_state = "OPEN"
                        pinch_open_event = True
                        release_start_time = None
                else:
                    release_start_time = None

        grip = 0.0 if pinch_state == "CLOSED" else 0.04

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

        # ---------- Assisted grasp detection ----------
        cube_pos, _ = p.getBasePositionAndOrientation(cube)
        cube_pos_np = np.array(cube_pos)

        grasp_point = get_grasp_point()
        xy_dist = float(np.linalg.norm(grasp_point[:2] - cube_pos_np[:2]))
        z_dist  = float(abs(grasp_point[2] - cube_pos_np[2]))
        graspable = (xy_dist < GRASP_XY_THRESH) and (z_dist < GRASP_Z_THRESH)

        halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
        p.changeVisualShape(halo_body, -1, rgbaColor=halo_color)
        p.resetBasePositionAndOrientation(halo_body, cube_pos, [0, 0, 0, 1])

        # attempts
        if pinch_close_event:
            grasp_attempts += 1

        # attach
        if pinch_close_event and (not cube_attached) and graspable:
            cid = create_fixed_constraint_preserve_pose(robot, HAND_LINK, cube)
            cube_attached = True
            if time_to_grasp is None:
                time_to_grasp = time.time() - trial_start

        # release
        if pinch_open_event and cube_attached:
            current_cube_pos_live = p.getBasePositionAndOrientation(cube)[0]
            is_near = near_target(current_cube_pos_live, current_target_pos, SUCCESS_THRESH)

            if not is_near:
                drops += 1

            p.removeConstraint(cid)
            cube_attached = False

            # end trial only if released near target
            if is_near:
                err = float(np.linalg.norm(np.array(current_cube_pos_live) - current_target_pos))
                time_total = time.time() - trial_start
                success = int(err < SUCCESS_THRESH)

                with open(LOG_FILE, "a", newline="") as f:
                    csv.writer(f).writerow([
                        PARTICIPANT_ID,
                        trial,
                        current_factor,
                        current_difficulty,
                        round(current_D, 4),
                        round(time_total, 3),
                        round(time_to_grasp if time_to_grasp is not None else time_total, 3),
                        round(err, 4),
                        success,
                        grasp_attempts,
                        drops,
                        round(current_cube_pos[0], 4), round(current_cube_pos[1], 4), round(current_cube_pos[2], 4),
                        round(current_target_pos[0], 4), round(current_target_pos[1], 4), round(current_target_pos[2], 4),
                    ])

                trial += 1
                if trial < N_TRIALS:
                    set_trial_condition(trial)
                reset_trial_state()

        # ---------- HUD ----------
        cv2.putText(frame,
                    f"TRIAL: {trial+1}/{N_TRIALS} | factor={current_factor} | diff={current_difficulty} | D={current_D:.2f}",
                    (10, h - 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.putText(frame, f"LEFT: {active_left} | RIGHT: {active_right}",
                    (10, h - 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        t2g_txt = "--" if time_to_grasp is None else f"{time_to_grasp:.2f}s"
        cv2.putText(frame, f"attempts={grasp_attempts} | drops={drops} | t_grasp={t2g_txt}",
                    (10, h - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        if pinch_val_best is not None:
            state_text = f"PINCH(min): {pinch_val_best:.3f} | STATE: {pinch_state}"
            if release_start_time is not None:
                countdown = RELEASE_DELAY - (time.time() - release_start_time)
                state_text += f" (DROP IN {countdown:.1f}s)"
                color = (0, 165, 255)
            else:
                color = (0, 255, 0)
            cv2.putText(frame, state_text,
                        (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        else:
            cv2.putText(frame, "PINCH: --",
                        (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Bimanual Gesture Control – Velocity + Assisted Grasp", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

        p.stepSimulation()
        time.sleep(1 / 240)

finally:
    cap.release()
    cv2.destroyAllWindows()
    if p.isConnected():
        p.disconnect()
