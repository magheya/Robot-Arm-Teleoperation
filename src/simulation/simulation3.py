import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import mediapipe as mp
import threading
import csv
import os

# =============================
# GLOBAL SHARED DATA
# =============================
class SharedState:
    def __init__(self):
        self.joint_vels = {"BASE": 0.0, "SHOULDER": 0.0, "WRIST": 0.0, "ELBOW": 0.0}
        self.grip_pos = 0.04
        self.pinch_event = None
        self.pinch_val = None
        self.frame = None
        self.is_running = True
        self.active_region = "NONE"
        self.pinch_state_text = "OPEN"
        self.release_countdown = None
        self.lock = threading.Lock()

shared = SharedState()

# Constants
DEADZONE_TOP = 0.42
DEADZONE_BOTTOM = 0.58
JOINT_SPEED = 2
MAX_FORCE = 250
RELEASE_DELAY = 0.6
SUCCESS_THRESH = 0.065

# Consider "fell off table" if cube is close to floor height
FLOOR_Z_THRESH = 0.06  # ~6 cm above ground is safe threshold

# =============================
# VISION WORKER THREAD
# =============================
def vision_worker(shared_state):
    cap = cv2.VideoCapture(0)
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)

    pinch_internal_state = "OPEN"
    release_start_time = None

    while shared_state.is_running:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb_frame)

        v_base, v_shoulder, v_wrist, v_elbow = 0.0, 0.0, 0.0, 0.0
        g_pos = 0.04
        event = None
        current_region = "NONE"
        p_val = None
        countdown = None

        # UI: Regions & Deadzone
        top_y, bot_y = int(h * DEADZONE_TOP), int(h * DEADZONE_BOTTOM)
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, top_y), (w, bot_y), (60, 60, 60), -1)
        frame = cv2.addWeighted(overlay, 0.22, frame, 0.78, 0)

        splits = [0.25, 0.50, 0.75]
        labels = ["BASE", "SHOULDER", "WRIST", "ELBOW"]
        for s in splits:
            cv2.line(frame, (int(w * s), 0), (int(w * s), h), (255, 255, 255), 1)
        for i, l in enumerate(labels):
            cv2.putText(frame, l, (int(w * (i * 0.25 + 0.125)) - 30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if res.multi_hand_landmarks:
            lm = res.multi_hand_landmarks[0].landmark
            cx, cy = np.mean([pt.x for pt in lm]), np.mean([pt.y for pt in lm])

            pixel_x, pixel_y = int(cx * w), int(cy * h)
            cv2.circle(frame, (pixel_x, pixel_y), 8, (0, 255, 0), -1)
            cv2.circle(frame, (pixel_x, pixel_y), 10, (255, 255, 255), 1)

            # Velocity Control Logic
            if cx < 0.25:
                current_region = "BASE"
                v_base = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0
            elif cx < 0.50:
                current_region = "SHOULDER"
                v_shoulder = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0
            elif cx < 0.75:
                current_region = "WRIST"
                v_wrist = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0
            else:
                current_region = "ELBOW"
                v_elbow = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0

            # Pinch Detection Logic
            p_val = float(np.linalg.norm([lm[4].x - lm[8].x, lm[4].y - lm[8].y]))

            if pinch_internal_state == "OPEN" and p_val < 0.04:
                pinch_internal_state = "CLOSED"
                event = "CLOSE"
                release_start_time = None

            elif pinch_internal_state == "CLOSED":
                if p_val > 0.07:
                    if release_start_time is None:
                        release_start_time = time.time()
                    countdown = RELEASE_DELAY - (time.time() - release_start_time)
                    if countdown <= 0:
                        pinch_internal_state = "OPEN"
                        event = "OPEN"
                        release_start_time = None
                        countdown = None
                else:
                    release_start_time = None

            g_pos = 0.0 if pinch_internal_state == "CLOSED" else 0.04

        with shared_state.lock:
            shared_state.joint_vels.update({"BASE": v_base, "SHOULDER": v_shoulder, "WRIST": v_wrist, "ELBOW": v_elbow})
            shared_state.grip_pos = g_pos
            shared_state.active_region = current_region
            shared_state.pinch_val = p_val
            shared_state.pinch_state_text = pinch_internal_state
            shared_state.release_countdown = countdown
            if event:
                shared_state.pinch_event = event
            shared_state.frame = frame

    cap.release()

# =============================
# MAIN THREAD (PYBULLET & LOGIC)
# =============================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(fixedTimeStep=1.0 / 240.0, numSolverIterations=50)

p.resetDebugVisualizerCamera(cameraDistance=1.4, cameraYaw=90, cameraPitch=-25,
                             cameraTargetPosition=[0.5, 0, 0.3])
plane_id = p.loadURDF("plane.urdf")
p.setCollisionFilterGroupMask(plane_id, -1, collisionFilterGroup=1, collisionFilterMask=-1)

robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# Joint/Link Mapping
joint_name_to_id = {}
link_name_to_id = {}
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    joint_name_to_id[info[1].decode()] = i
    link_name_to_id[info[12].decode()] = i

j_ids = {"BASE": 0, "SHOULDER": 1, "ELBOW": 3, "WRIST": 5, "F_L": 9, "F_R": 10, "HAND": 8}
HAND_LINK = link_name_to_id["panda_link8"]

# --- Neutral Posture & Tables ---
neutral = {
    "panda_joint1": 0.0, "panda_joint2": -0.4, "panda_joint3": 0.0,
    "panda_joint4": -2.0, "panda_joint5": 0.0, "panda_joint6": 1.7, "panda_joint7": 0.8
}
LOCKED_JOINTS = {"panda_joint3": 0.0, "panda_joint5": 0.0, "panda_joint7": 0.8}
locked_ids = {joint_name_to_id[name]: val for name, val in LOCKED_JOINTS.items()}
for j_name, val in neutral.items():
    p.resetJointState(robot, joint_name_to_id[j_name], val)

def create_table(x, y, top_z):
    half = [0.22, 0.22, 0.05]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[0.75, 0.75, 0.75, 1])
    t_id = p.createMultiBody(0, col, vis, [x, y, top_z - half[2]])
    p.setCollisionFilterGroupMask(t_id, -1, collisionFilterGroup=1, collisionFilterMask=-1)

TABLE_X, PICK_Y, PLACE_Y, TABLE_Z = 0.48, -0.28, 0.28, 0.26
create_table(TABLE_X, PICK_Y, TABLE_Z)
create_table(TABLE_X, PLACE_Y, TABLE_Z)

# --- Objects ---
CUBE_SIZE = 0.05
BASE_CUBE_POS = np.array([TABLE_X, PICK_Y + 0.03, TABLE_Z + CUBE_SIZE / 2])
BASE_TARGET_POS = np.array([TABLE_X, PLACE_Y - 0.03, TABLE_Z + CUBE_SIZE / 2])

cube = p.loadURDF("cube_small.urdf", BASE_CUBE_POS.tolist(), globalScaling=0.8)
p.setCollisionFilterGroupMask(cube, -1, collisionFilterGroup=2, collisionFilterMask=-1)

target_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.002, rgbaColor=[1, 0, 0, 0.7])
target_body = p.createMultiBody(0, baseVisualShapeIndex=target_vis, basePosition=BASE_TARGET_POS.tolist())

halo_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.075, rgbaColor=[1, 0, 0, 0.18])
halo_body = p.createMultiBody(0, baseVisualShapeIndex=halo_vis, basePosition=BASE_CUBE_POS.tolist())

shadow_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.025, length=0.001, rgbaColor=[0, 0, 0, 0.4])
shadow_body = p.createMultiBody(0, baseVisualShapeIndex=shadow_vis, basePosition=[0, 0, 0])
p.setCollisionFilterGroupMask(shadow_body, -1, 0, 0)

gripper_proj_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.03, length=0.001, rgbaColor=[1, 1, 1, 0.4])
gripper_proj_body = p.createMultiBody(0, baseVisualShapeIndex=gripper_proj_vis, basePosition=[0, 0, -1])

# =============================
# EXPERIMENT SETUP (ONLY GRASP DIFFICULTY)
# =============================
PARTICIPANT_ID = "P01"
LOG_FILE = f"results_{PARTICIPANT_ID}_threaded.csv"

# Added "outcome" column: success / missed_target / fell_off_table
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow([
            "participant", "trial", "difficulty", "D",
            "time_total", "time_to_grasp", "placement_error",
            "success", "outcome", "drops",
            "cube_x", "cube_y", "cube_z", "target_x", "target_y", "target_z"
        ])

LEVELS = ["easy", "medium", "hard"]

CUBE_POS_BY_LEVEL = {
    "easy":   np.array([TABLE_X - 0.06, PICK_Y + 0.02, BASE_CUBE_POS[2]]),
    "medium": BASE_CUBE_POS,
    "hard":   np.array([TABLE_X + 0.06, PICK_Y - 0.06, BASE_CUBE_POS[2]])
}
FIXED_TARGET_POS = BASE_TARGET_POS.copy()

TRIAL_PLAN = []
for _ in range(2):
    for lvl in LEVELS:
        TRIAL_PLAN.append({"difficulty": lvl})

trial_idx = 0
trial_start = time.time()
cube_attached, cid = False, None
drops, time_to_grasp = 0, None

def get_grasp_point():
    lf = np.array(p.getLinkState(robot, j_ids["F_L"])[0])
    rf = np.array(p.getLinkState(robot, j_ids["F_R"])[0])
    return (lf + rf) * 0.5

def create_fixed_constraint_preserve_pose(parent_robot, parent_link, child_body):
    ee_pos, ee_orn = p.getLinkState(parent_robot, parent_link)[:2]
    cube_pos, cube_orn = p.getBasePositionAndOrientation(child_body)
    inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
    local_pos, local_orn = p.multiplyTransforms(inv_ee_pos, inv_ee_orn, cube_pos, cube_orn)
    return p.createConstraint(parent_robot, parent_link, child_body, -1,
                              p.JOINT_FIXED, [0, 0, 0], local_pos, [0, 0, 0], local_orn)

def set_trial_condition(idx):
    global current_difficulty, current_cube_pos, current_target_pos, current_D
    current_difficulty = TRIAL_PLAN[idx]["difficulty"]
    current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty]
    current_target_pos = FIXED_TARGET_POS

    p.resetBasePositionAndOrientation(cube, current_cube_pos.tolist(), [0, 0, 0, 1])
    p.resetBasePositionAndOrientation(target_body, current_target_pos.tolist(), [0, 0, 0, 1])

    current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

def log_and_advance(outcome, placement_error, current_cube_pos):
    """
    outcome: "success" | "missed_target" | "fell_off_table"
    placement_error: float (2D error), or np.nan if not meaningful
    """
    global trial_idx, trial_start, drops, time_to_grasp

    time_total = time.time() - trial_start
    success = 1 if outcome == "success" else 0

    with open(LOG_FILE, "a", newline="") as f:
        csv.writer(f).writerow([
            PARTICIPANT_ID, trial_idx, current_difficulty,
            round(current_D, 4),
            round(time_total, 3),
            round(time_to_grasp if time_to_grasp is not None else time_total, 3),
            (round(float(placement_error), 4) if placement_error == placement_error else np.nan),  # keep NaN
            success, outcome, drops,
            *np.round(current_cube_pos, 4), *np.round(current_target_pos, 4)
        ])

    trial_idx += 1
    if trial_idx < len(TRIAL_PLAN):
        set_trial_condition(trial_idx)
        trial_start = time.time()
        drops, time_to_grasp = 0, None

set_trial_condition(0)

# =============================
# RUNTIME
# =============================
vt = threading.Thread(target=vision_worker, args=(shared,), daemon=True)
vt.start()

last_halo_color = [1, 0, 0, 0.18]
last_halo_pos = np.array([0, 0, 0], dtype=float)

try:
    while shared.is_running and trial_idx < len(TRIAL_PLAN):
        with shared.lock:
            v = shared.joint_vels.copy()
            g = shared.grip_pos
            event = shared.pinch_event
            p_state = shared.pinch_state_text
            countdown = shared.release_countdown
            shared.pinch_event = None
            display_frame = shared.frame.copy() if shared.frame is not None else None

        # Motor Control
        for name, jid in j_ids.items():
            if name in v:
                p.setJointMotorControl2(robot, jid, p.VELOCITY_CONTROL,
                                        targetVelocity=v[name], force=MAX_FORCE)

        for jid, pos in locked_ids.items():
            p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL,
                                    targetPosition=pos, force=MAX_FORCE)

        p.setJointMotorControl2(robot, j_ids["F_L"], p.POSITION_CONTROL, g, force=100)
        p.setJointMotorControl2(robot, j_ids["F_R"], p.POSITION_CONTROL, g, force=100)

        # --- Cube position (used for off-table detection too) ---
        cp_raw, _ = p.getBasePositionAndOrientation(cube)
        cp = np.array(cp_raw)

        # =============================
        # AUTO FAIL if fell off table (floor)
        # =============================
        if cp[2] < FLOOR_Z_THRESH:
            # If cube is attached, detach safely
            if cube_attached and cid is not None:
                try:
                    p.removeConstraint(cid)
                except:
                    pass
                cube_attached = False
                cid = None
            log_and_advance("fell_off_table", np.nan, cp)
            p.stepSimulation()
            continue

        # Visual Aids: shadow + projection
        ray_res_cube = p.rayTest(cp_raw, [cp_raw[0], cp_raw[1], 0], collisionFilterMask=1)
        shadow_z = (ray_res_cube[0][3][2] + 0.001) if (ray_res_cube and ray_res_cube[0][0] != -1) else 0.001
        p.resetBasePositionAndOrientation(shadow_body, [cp_raw[0], cp_raw[1], shadow_z], [0, 0, 0, 1])

        if not cube_attached:
            gp = get_grasp_point()
            ray_res = p.rayTest(gp, [gp[0], gp[1], 0], collisionFilterMask=1)
            if ray_res and ray_res[0][0] != -1:
                proj_z = ray_res[0][3][2] + 0.0015
                p.resetBasePositionAndOrientation(gripper_proj_body, [gp[0], gp[1], proj_z], [0, 0, 0, 1])
            else:
                p.resetBasePositionAndOrientation(gripper_proj_body, [0, 0, -1], [0, 0, 0, 1])
        else:
            p.resetBasePositionAndOrientation(gripper_proj_body, [0, 0, -1], [0, 0, 0, 1])

        # Assisted grasp detection
        grasp_point = get_grasp_point()
        xy_dist = float(np.linalg.norm(grasp_point[:2] - cp[:2]))
        z_dist = float(abs(grasp_point[2] - cp[2]))
        graspable = (xy_dist < 0.065) and (z_dist < 0.055)

        current_halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
        if current_halo_color != last_halo_color:
            p.changeVisualShape(halo_body, -1, rgbaColor=current_halo_color)
            last_halo_color = current_halo_color

        if np.linalg.norm(np.array(cp_raw) - np.array(last_halo_pos)) > 0.001:
            p.resetBasePositionAndOrientation(halo_body, cp_raw, [0, 0, 0, 1])
            last_halo_pos = np.array(cp_raw)

        # Trial events
        if event == "CLOSE":
            if graspable and not cube_attached:
                cid = create_fixed_constraint_preserve_pose(robot, HAND_LINK, cube)
                cube_attached = True
                if time_to_grasp is None:
                    time_to_grasp = time.time() - trial_start

        elif event == "OPEN" and cube_attached:
            cube_pos_raw, _ = p.getBasePositionAndOrientation(cube)
            current_cube_pos = np.array(cube_pos_raw)

            # If it's basically on the floor right at release -> fell_off_table
            if current_cube_pos[2] < FLOOR_Z_THRESH:
                if cid:
                    p.removeConstraint(cid)
                cube_attached = False
                cid = None
                log_and_advance("fell_off_table", np.nan, current_cube_pos)

            else:
                dist_2d = float(np.linalg.norm(current_cube_pos[:2] - current_target_pos[:2]))
                is_near = dist_2d < SUCCESS_THRESH

                if cid:
                    p.removeConstraint(cid)
                cube_attached = False
                cid = None
                p.resetBaseVelocity(cube, linearVelocity=[0, 0, -0.1])

                if is_near:
                    log_and_advance("success", dist_2d, current_cube_pos)
                else:
                    drops += 1
                    log_and_advance("missed_target", dist_2d, current_cube_pos)

        p.stepSimulation()

        if display_frame is not None:
            h, w = display_frame.shape[:2]
            cv2.putText(display_frame,
                        f"TRIAL: {trial_idx+1}/{len(TRIAL_PLAN)} | difficulty: {current_difficulty}",
                        (10, h - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(display_frame,
                        f"STATE: {p_state} " + (f"DROP IN {countdown:.1f}s" if countdown else ""),
                        (10, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 165, 255) if countdown else (0, 255, 0), 2)

            cv2.putText(display_frame,
                        f"Drops: {drops} | D={current_D:.3f}",
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Vision Feedback", display_frame)
            if cv2.waitKey(1) & 0xFF == 27:
                shared.is_running = False
                break

finally:
    shared.is_running = False
    vt.join()
    cv2.destroyAllWindows()
    p.disconnect()
