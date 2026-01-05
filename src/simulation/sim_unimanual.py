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

LEFT_FINGER_LINK  = link_name_to_id.get("panda_leftfinger", None)
RIGHT_FINGER_LINK = link_name_to_id.get("panda_rightfinger", None)

if HAND_LINK is None:
    raise RuntimeError("Could not find panda_link8")

if LEFT_FINGER_LINK is None or RIGHT_FINGER_LINK is None:
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
# TABLES (Layout B)
# =============================
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

# =============================
# CUBE + TARGET
# =============================
CUBE_SIZE = 0.05

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

# ✅ bigger deadzone
DEADZONE_TOP = 0.42
DEADZONE_BOTTOM = 0.58

PINCH_CLOSE = 0.04
PINCH_OPEN  = 0.07

ARM_FORCE = 90
GRIP_FORCE = 100

# =============================
# ASSISTED GRASP
# =============================
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
    basePosition=cube_start
)

# =============================
# EXPERIMENT LOGGING  (UPDATED)
# =============================
PARTICIPANT_ID = "P01"
LOG_FILE = f"results_{PARTICIPANT_ID}_unimanual.csv"
SUCCESS_THRESH = 0.05

N_TRIALS = 5  # ✅ run exactly 5 trials then stop

# New metrics:
# - time_to_grasp: time from trial start to first attachment
# - grasp_attempts: number of pinch-close events (attempts) during trial
# - drops: number of times cube was released while not at target (unintended drops)

if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow([
            "participant",
            "trial",
            "time_total",
            "time_to_grasp",
            "placement_error",
            "success",
            "grasp_attempts",
            "drops",
        ])

trial = 0
trial_start = time.time()
cube_attached = False
cid = None

# ✅ per-trial counters/state
grasp_attempts = 0
drops = 0
time_to_grasp = None  # set when first attached

def reset_trial_state():
    """Reset per-trial counters and state (keeps your environment the same)."""
    global trial_start, cube_attached, cid, grasp_attempts, drops, time_to_grasp
    trial_start = time.time()
    cube_attached = False
    cid = None
    grasp_attempts = 0
    drops = 0
    time_to_grasp = None

def get_grasp_point():
    """Midpoint of fingertip LINKS (best). If not available, fallback to hand link position."""
    if LEFT_FINGER_LINK is not None and RIGHT_FINGER_LINK is not None:
        lf = np.array(p.getLinkState(robot, LEFT_FINGER_LINK)[0])
        rf = np.array(p.getLinkState(robot, RIGHT_FINGER_LINK)[0])
        return (lf + rf) * 0.5
    return np.array(p.getLinkState(robot, HAND_LINK)[0])

def create_fixed_constraint_preserve_pose(parent_body, parent_link, child_body):
    """
    Create a fixed constraint that preserves the current relative pose between parent link and child body.
    This prevents the cube snapping into the wrist.
    """
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

def near_target(pos, target, thresh):
    return float(np.linalg.norm(np.array(pos) - np.array(target))) < float(thresh)

# pinch edge-trigger
pinch_state = "OPEN"  # OPEN or CLOSED
release_start_time = None  # Tracks when the release "began"
RELEASE_DELAY = 0.6  # Seconds to wait before actually releasing

# =============================
# MAIN LOOP
# =============================
try:
    while True:
        if trial >= N_TRIALS:
            print(f"[DONE] Completed {N_TRIALS} trials.")
            break

        if not p.isConnected():
            print("[ERROR] PyBullet disconnected (GUI closed or crashed). Exiting cleanly.")
            break

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
        for i, l in enumerate(labels):
            cv2.putText(frame, l,
                        (int(w*(i*0.25+0.125))-45, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # ---------- UI: deadzone (TRANSPARENT) ----------
        top_y = int(h * DEADZONE_TOP)
        bot_y = int(h * DEADZONE_BOTTOM)

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, top_y), (w, bot_y), (60,60,60), -1)
        frame = cv2.addWeighted(overlay, 0.22, frame, 0.78, 0)
        cv2.line(frame, (0, top_y), (w, top_y), (0,255,0), 2)
        cv2.line(frame, (0, bot_y), (w, bot_y), (0,255,0), 2)

        # ---------- Default velocities ----------
        joint_vel = {BASE_J: 0.0, SHOULDER_J: 0.0, WRIST_J: 0.0, ELBOW_J: 0.0}

        # ---------- Hand tracking ----------
        res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        active_region = "NONE"
        grip = 0.04
        pinch_val = None

        pinch_close_event = False
        pinch_open_event = False

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

            # Direction
            if cy < DEADZONE_TOP:
                joint_vel[jid] = +JOINT_VEL[active_region]
            elif cy > DEADZONE_BOTTOM:
                joint_vel[jid] = -JOINT_VEL[active_region]

            # Pinch value
            thumb = lm[mp_hands.HandLandmark.THUMB_TIP]
            index = lm[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            pinch_val = float(np.linalg.norm(
                np.array([thumb.x, thumb.y]) - np.array([index.x, index.y])
            ))

            # ----------------------------------------------------
            #  DELAYED RELEASE LOGIC
            # ----------------------------------------------------
            # 1. Detect CLOSE (Immediate)
            if pinch_state == "OPEN" and pinch_val < PINCH_CLOSE:
                pinch_state = "CLOSED"
                pinch_close_event = True
                release_start_time = None  # Reset timer immediately if we close

            # 2. Detect OPEN (Delayed)
            elif pinch_state == "CLOSED":
                if pinch_val > PINCH_OPEN:
                    # Pinch is open, start timer if not already started
                    if release_start_time is None:
                        release_start_time = time.time()

                    # Check if timer exceeded
                    elif (time.time() - release_start_time) > RELEASE_DELAY:
                        pinch_state = "OPEN"
                        pinch_open_event = True
                        release_start_time = None
                else:
                    # Pinch is tight again (or oscillating), cancel timer
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

        # Update halo
        halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
        p.changeVisualShape(halo_body, -1, rgbaColor=halo_color)
        p.resetBasePositionAndOrientation(halo_body, cube_pos, [0, 0, 0, 1])

        # ✅ grasp attempts = number of pinch-close events during the trial
        if pinch_close_event:
            grasp_attempts += 1

        # ✅ Attach on pinch CLOSE EVENT AND graspable
        if pinch_close_event and (not cube_attached) and graspable:
            cid = create_fixed_constraint_preserve_pose(robot, HAND_LINK, cube)
            cube_attached = True
            if time_to_grasp is None:
                time_to_grasp = time.time() - trial_start  # first grasp time

        # Release on pinch OPEN EVENT
        if pinch_open_event and cube_attached:
            # count a "drop" if released away from target (i.e., not a successful placement action)
            current_cube_pos = p.getBasePositionAndOrientation(cube)[0]
            is_near_target = near_target(current_cube_pos, TARGET_POS, SUCCESS_THRESH)
            if not is_near_target:
                drops += 1

            p.removeConstraint(cid)
            cube_attached = False

            # If user released near target => we end the trial + log
            if is_near_target:
                err = float(np.linalg.norm(np.array(current_cube_pos) - TARGET_POS))
                time_total = time.time() - trial_start
                success = int(err < SUCCESS_THRESH)

                with open(LOG_FILE, "a", newline="") as f:
                    csv.writer(f).writerow([
                        PARTICIPANT_ID,
                        trial,
                        round(time_total, 3),
                        round(time_to_grasp if time_to_grasp is not None else time_total, 3),
                        round(err, 4),
                        success,
                        grasp_attempts,
                        drops,
                    ])

                trial += 1

                # reset for next trial (same environment)
                p.resetBasePositionAndOrientation(cube, cube_start, [0, 0, 0, 1])
                reset_trial_state()

        # ---------- HUD ----------
        cv2.putText(frame, f"TRIAL: {trial+1}/{N_TRIALS} | ACTIVE: {active_region}",
                    (10, h - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if pinch_val is not None:
            # Show Timer if releasing
            state_text = f"STATE: {pinch_state}"
            if release_start_time is not None:
                countdown = RELEASE_DELAY - (time.time() - release_start_time)
                state_text += f" (DROP IN {countdown:.1f}s)"
                text_color = (0, 165, 255)  # Orange
            else:
                text_color = (0, 255, 0)    # Green

            cv2.putText(frame, f"PINCH: {pinch_val:.3f} | {state_text}",
                        (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
        else:
            cv2.putText(frame, "PINCH: --",
                        (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # show new counters live
        t2g_txt = "--" if time_to_grasp is None else f"{time_to_grasp:.2f}s"
        cv2.putText(frame, f"attempts={grasp_attempts} | drops={drops} | t_grasp={t2g_txt}",
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        cv2.imshow("Unimanual Gesture Control – Velocity + Assisted Grasp", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

        p.stepSimulation()
        time.sleep(1/240)
finally:
    cap.release()
    cv2.destroyAllWindows()
    if p.isConnected():
        p.disconnect()
