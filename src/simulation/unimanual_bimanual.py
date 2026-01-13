import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import mediapipe as mp
import threading
import csv
import os
import uuid
import random

# ============================================================
# RUN-ID CSV HELPERS (persistent across program restarts)
# ============================================================
def ensure_csv_with_header(path: str, header: list[str]) -> None:
    if not os.path.exists(path):
        with open(path, "w", newline="") as f:
            csv.writer(f).writerow(header)

def next_run_id(csv_path: str, participant_id: str) -> int:
    """
    Returns 1 if file missing/empty, else max(run_id) for this participant + 1.
    """
    if not os.path.exists(csv_path):
        return 1
    try:
        with open(csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                return 1
            max_id = 0
            for row in reader:
                if row.get("participant") == participant_id:
                    try:
                        rid = int(row.get("run_id", "0"))
                        max_id = max(max_id, rid)
                    except:
                        pass
            return max_id + 1
    except:
        return 1

def get_next_participant_number(filename="participant_counter.txt") -> int:
    """Reads the next P# from a file so it remembers across restarts."""
    num = 0
    if os.path.exists(filename):
        try:
            with open(filename, "r") as f:
                val = f.read().strip()
                if val.isdigit():
                    num = int(val)
        except:
            pass
            
    # Save the NEXT number immediately
    with open(filename, "w") as f:
        f.write(str(num + 1))
    return num

def generate_participant_id(p_num: int) -> tuple[str, str, list]:
    """
    Generates ID based on number (P01, P02...) and determines Order.
    Returns: (participant_id, group_name, experiment_list)
    """
    # Even numbers = Group A (Unimanual first)
    # Odd numbers  = Group B (Bimanual first)
    if p_num % 2 == 0:
        group_label = "A"
        # Order: Unimanual -> Bimanual
        order = [(run_unimanual, "UNIMANUAL"), (run_bimanual, "BIMANUAL")]
    else:
        group_label = "B"
        # Order: Bimanual -> Unimanual
        order = [(run_bimanual, "BIMANUAL"), (run_unimanual, "UNIMANUAL")]

    # Create clean ID: "P05_GroupB_20231027"
    # :02d ensures "P01" instead of "P1" (better for file sorting)
    date_str = time.strftime("%Y%m%d") 
    p_id = f"P{p_num:02d}_Group{group_label}_{date_str}"
    
    return p_id, group_label, order

# ============================================================
# SHARED HELPERS
# ============================================================
def create_table(x, y, top_z, collision_group=1, collision_mask=-1):
    half = [0.22, 0.22, 0.05]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)

    # lighter table so the shadow (dark) is easier to see
    vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=[0.95, 0.95, 0.95, 1.0]
    )

    t_id = p.createMultiBody(0, col, vis, [x, y, top_z - half[2]])
    p.setCollisionFilterGroupMask(t_id, -1, collision_group, collision_mask)
    return t_id

def create_fixed_constraint_preserve_pose(parent_body, parent_link, child_body):
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

# ============================================================
# TRIAL PLAN (shared)
# - Order is DIFFERENT each run (shuffled)
# - We use the SAME order for unimanual and bimanual in that run
# ============================================================
def make_trial_plan_shuffled(rng: random.Random, reps=2, levels=("easy", "medium", "hard")):
    trials = []
    for _ in range(reps):
        for lvl in levels:
            trials.append({"difficulty": lvl})
    rng.shuffle(trials)
    return trials

# ============================================================
# SHARED WORLD SETUP (used by BOTH unimanual + bimanual)
# ============================================================
def setup_world():
    """
    Creates: plane, robot, tables, cube, target, halo, shadow, gripper projection.
    Returns a dict with everything needed by both experiments.
    """
    # Always reset safely
    if p.isConnected():
        try:
            p.disconnect()
        except:
            pass

    # --- scene constants (shared) ---
    TABLE_X, PICK_Y, PLACE_Y, TABLE_Z = 0.48, -0.28, 0.28, 0.26
    CUBE_SIZE = 0.05

    BASE_CUBE_POS = np.array([TABLE_X, PICK_Y + 0.03, TABLE_Z + CUBE_SIZE / 2], dtype=float)
    BASE_TARGET_POS = np.array([TABLE_X, PLACE_Y - 0.03, TABLE_Z + CUBE_SIZE / 2], dtype=float)

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(fixedTimeStep=1.0 / 240.0, numSolverIterations=50)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.4, cameraYaw=90, cameraPitch=-25,
        cameraTargetPosition=[0.5, 0, 0.3]
    )

    plane_id = p.loadURDF("plane.urdf")
    p.setCollisionFilterGroupMask(plane_id, -1, collisionFilterGroup=1, collisionFilterMask=-1)

    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

    # map joints/links
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

    # neutral pose
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

    # tables
    create_table(TABLE_X, PICK_Y, TABLE_Z)
    create_table(TABLE_X, PLACE_Y, TABLE_Z)

    # cube
    cube = p.loadURDF("cube_small.urdf", BASE_CUBE_POS.tolist(), globalScaling=0.8)
    p.setCollisionFilterGroupMask(cube, -1, collisionFilterGroup=2, collisionFilterMask=-1)

    # target
    target_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.002, rgbaColor=[1, 0, 0, 0.7])
    target_body = p.createMultiBody(0, baseVisualShapeIndex=target_vis, basePosition=BASE_TARGET_POS.tolist())

    # halo
    halo_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.075, rgbaColor=[1, 0, 0, 0.18])
    halo_body = p.createMultiBody(0, baseVisualShapeIndex=halo_vis, basePosition=BASE_CUBE_POS.tolist())

    # shadow (under cube)
    shadow_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.025, length=0.001, rgbaColor=[0, 0, 0, 0.4])
    shadow_body = p.createMultiBody(0, baseVisualShapeIndex=shadow_vis, basePosition=[0, 0, 0])
    p.setCollisionFilterGroupMask(shadow_body, -1, 0, 0)

    # darker shadow for gripper projection (used when NOT carrying)
    gripper_shadow_vis = p.createVisualShape(
        p.GEOM_CYLINDER, radius=0.03, length=0.001, rgbaColor=[0, 0, 0, 0.65]
    )
    gripper_shadow_body = p.createMultiBody(
        0, baseVisualShapeIndex=gripper_shadow_vis, basePosition=[0, 0, -1]
    )
    p.setCollisionFilterGroupMask(gripper_shadow_body, -1, 0, 0)


    # gripper projection
    gripper_proj_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.03, length=0.001, rgbaColor=[1, 1, 1, 0.4])
    gripper_proj_body = p.createMultiBody(0, baseVisualShapeIndex=gripper_proj_vis, basePosition=[0, 0, -1])

    # shared IDs
    j_ids = {
        "BASE": joint_name_to_id["panda_joint1"],
        "SHOULDER": joint_name_to_id["panda_joint2"],
        "ELBOW": joint_name_to_id["panda_joint4"],
        "WRIST": joint_name_to_id["panda_joint6"],
        "F_L": joint_name_to_id["panda_finger_joint1"],
        "F_R": joint_name_to_id["panda_finger_joint2"],
        "HAND": HAND_LINK,
    }

    LOCKED_JOINTS = {"panda_joint3": 0.0, "panda_joint5": 0.0, "panda_joint7": 0.8}
    locked_ids = {joint_name_to_id[name]: val for name, val in LOCKED_JOINTS.items()}

    LEFT_FINGER_LINK = link_name_to_id.get("panda_leftfinger", None)
    RIGHT_FINGER_LINK = link_name_to_id.get("panda_rightfinger", None)

    return dict(
        # scene constants
        TABLE_X=TABLE_X, PICK_Y=PICK_Y, PLACE_Y=PLACE_Y, TABLE_Z=TABLE_Z,
        BASE_CUBE_POS=BASE_CUBE_POS, BASE_TARGET_POS=BASE_TARGET_POS,
        # bodies
        plane_id=plane_id,
        robot=robot,
        cube=cube,
        target_body=target_body,
        halo_body=halo_body,
        shadow_body=shadow_body,
        gripper_proj_body=gripper_proj_body,
        gripper_shadow_body=gripper_shadow_body,
        # mappings
        joint_name_to_id=joint_name_to_id,
        link_name_to_id=link_name_to_id,
        j_ids=j_ids,
        locked_ids=locked_ids,
        neutral=neutral,
        LEFT_FINGER_LINK=LEFT_FINGER_LINK,
        RIGHT_FINGER_LINK=RIGHT_FINGER_LINK,
    )

# ============================================================
# UNIMANUAL
# ============================================================
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

def run_unimanual(participant_id: str, trial_plan: list[dict], block_num: int):    
    global shared
    DEADZONE_TOP = 0.42
    DEADZONE_BOTTOM = 0.58
    JOINT_SPEED = 2.0
    MAX_FORCE = 250
    RELEASE_DELAY = 0.6

    SUCCESS_THRESH = 0.065
    FLOOR_Z_THRESH = 0.06
    GRASP_XY_THRESH = 0.065
    GRASP_Z_THRESH = 0.055

    shared = SharedState()

    def vision_worker(shared_state: SharedState):
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

            v = {"BASE": 0.0, "SHOULDER": 0.0, "WRIST": 0.0, "ELBOW": 0.0}
            g_pos = 0.04
            event = None
            current_region = "NONE"
            p_val = None
            countdown = None

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
                cx, cy = float(np.mean([pt.x for pt in lm])), float(np.mean([pt.y for pt in lm]))

                pixel_x, pixel_y = int(cx * w), int(cy * h)
                cv2.circle(frame, (pixel_x, pixel_y), 8, (0, 255, 0), -1)
                cv2.circle(frame, (pixel_x, pixel_y), 10, (255, 255, 255), 1)

                # region select
                if cx < 0.25:
                    current_region = "BASE"
                    v["BASE"] = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0.0
                elif cx < 0.50:
                    current_region = "SHOULDER"
                    v["SHOULDER"] = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0.0
                elif cx < 0.75:
                    current_region = "WRIST"
                    v["WRIST"] = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0.0
                else:
                    current_region = "ELBOW"
                    v["ELBOW"] = JOINT_SPEED if cy < DEADZONE_TOP else -JOINT_SPEED if cy > DEADZONE_BOTTOM else 0.0

                # pinch
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
                shared_state.joint_vels = v
                shared_state.grip_pos = g_pos
                shared_state.active_region = current_region
                shared_state.pinch_val = p_val
                shared_state.pinch_state_text = pinch_internal_state
                shared_state.release_countdown = countdown
                if event:
                    shared_state.pinch_event = event
                shared_state.frame = frame

        cap.release()

    # --- setup world ---
    W = setup_world()
    robot = W["robot"]
    cube = W["cube"]
    target_body = W["target_body"]
    halo_body = W["halo_body"]
    shadow_body = W["shadow_body"]
    gripper_proj_body = W["gripper_proj_body"]
    gripper_shadow_body = W["gripper_shadow_body"]  
    j_ids = W["j_ids"]
    locked_ids = W["locked_ids"]

    TABLE_X = W["TABLE_X"]; PICK_Y = W["PICK_Y"]
    BASE_CUBE_POS = W["BASE_CUBE_POS"]
    BASE_TARGET_POS = W["BASE_TARGET_POS"]

    # persistent CSV logging
    LOG_FILE = "results_unimanual_all.csv"
    HEADER = [
        "participant", "run_id", "block_num", "trial", "difficulty", "D",
        "time_total", "time_to_grasp", "placement_error",
        "success", "outcome", "drops",
        "cube_x", "cube_y", "cube_z",
        "target_x", "target_y", "target_z"
    ]
    ensure_csv_with_header(LOG_FILE, HEADER)
    RUN_ID = next_run_id(LOG_FILE, participant_id)
    print(f"[UNIMANUAL] participant={participant_id} run_id={RUN_ID}")

    # level positions
    CUBE_POS_BY_LEVEL = {
        "easy":   np.array([TABLE_X - 0.06, PICK_Y + 0.02, BASE_CUBE_POS[2]], dtype=float),
        "medium": BASE_CUBE_POS.copy(),
        "hard":   np.array([TABLE_X + 0.06, PICK_Y - 0.06, BASE_CUBE_POS[2]], dtype=float)
    }
    FIXED_TARGET_POS = BASE_TARGET_POS.copy()

    trial_idx = 0
    trial_start = time.time()
    cube_attached, cid = False, None
    drops, time_to_grasp = 0, None

    current_difficulty = trial_plan[0]["difficulty"]
    current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty].copy()
    current_target_pos = FIXED_TARGET_POS.copy()
    current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

    def set_trial_condition(idx: int):
        nonlocal current_difficulty, current_cube_pos, current_target_pos, current_D
        nonlocal trial_start, cube_attached, cid, drops, time_to_grasp

        current_difficulty = trial_plan[idx]["difficulty"]
        current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty].copy()
        current_target_pos = FIXED_TARGET_POS.copy()

        p.resetBasePositionAndOrientation(cube, current_cube_pos.tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(target_body, current_target_pos.tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(halo_body, current_cube_pos.tolist(), [0, 0, 0, 1])

        # --- NEW CODE START: Reset Robot to Start Position ---
        for j_name, j_val in W["neutral"].items():
            j_id = W["joint_name_to_id"][j_name]
            p.resetJointState(robot, j_id, j_val, targetVelocity=0)
            
        # Optional: Reset the shared velocity command to 0 so it doesn't jerk immediately
        with shared.lock:
            shared.joint_vels = {"BASE": 0.0, "SHOULDER": 0.0, "WRIST": 0.0, "ELBOW": 0.0}
        # --- NEW CODE END ---

        current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

        trial_start = time.time()
        cube_attached, cid = False, None
        drops, time_to_grasp = 0, None

    def get_grasp_point():
        lf = np.array(p.getLinkState(robot, j_ids["F_L"])[0])
        rf = np.array(p.getLinkState(robot, j_ids["F_R"])[0])
        return (lf + rf) * 0.5

    def create_fixed_constraint_preserve_pose_unimanual(parent_robot, parent_link, child_body):
        ee_pos, ee_orn = p.getLinkState(parent_robot, parent_link)[:2]
        cube_pos, cube_orn = p.getBasePositionAndOrientation(child_body)
        inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
        local_pos, local_orn = p.multiplyTransforms(inv_ee_pos, inv_ee_orn, cube_pos, cube_orn)
        return p.createConstraint(parent_robot, parent_link, child_body, -1,
                                  p.JOINT_FIXED, [0, 0, 0], local_pos, [0, 0, 0], local_orn)

    def log_and_advance(outcome, placement_error, cube_pos_np):
        nonlocal trial_idx
        time_total = time.time() - trial_start
        success = 1 if outcome == "success" else 0

        with open(LOG_FILE, "a", newline="") as f:
            csv.writer(f).writerow([
                participant_id, RUN_ID, block_num, trial_idx, current_difficulty, round(current_D, 4),
                round(time_total, 3),
                round(time_to_grasp if time_to_grasp is not None else time_total, 3),
                (round(float(placement_error), 4) if placement_error == placement_error else np.nan),
                success, outcome, drops,
                *np.round(cube_pos_np, 4), *np.round(current_target_pos, 4)
            ])

        trial_idx += 1
        if trial_idx < len(trial_plan):
            set_trial_condition(trial_idx)

    # start trial 0
    set_trial_condition(0)

    vt = threading.Thread(target=vision_worker, args=(shared,), daemon=True)
    vt.start()

    cv2.namedWindow("Vision Feedback (Unimanual)", cv2.WINDOW_NORMAL)

    try:
        while shared.is_running and trial_idx < len(trial_plan):
            with shared.lock:
                v = shared.joint_vels.copy()
                g = shared.grip_pos
                event = shared.pinch_event
                p_state = shared.pinch_state_text
                countdown = shared.release_countdown
                shared.pinch_event = None
                display_frame = shared.frame.copy() if shared.frame is not None else None

            # velocity joints (only these 4)
            p.setJointMotorControl2(robot, j_ids["BASE"], p.VELOCITY_CONTROL, targetVelocity=v["BASE"], force=MAX_FORCE)
            p.setJointMotorControl2(robot, j_ids["SHOULDER"], p.VELOCITY_CONTROL, targetVelocity=v["SHOULDER"], force=MAX_FORCE)
            p.setJointMotorControl2(robot, j_ids["WRIST"], p.VELOCITY_CONTROL, targetVelocity=v["WRIST"], force=MAX_FORCE)
            p.setJointMotorControl2(robot, j_ids["ELBOW"], p.VELOCITY_CONTROL, targetVelocity=v["ELBOW"], force=MAX_FORCE)

            # lock joints
            for jid, pos in locked_ids.items():
                p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=pos, force=MAX_FORCE)

            # gripper
            p.setJointMotorControl2(robot, j_ids["F_L"], p.POSITION_CONTROL, g, force=100)
            p.setJointMotorControl2(robot, j_ids["F_R"], p.POSITION_CONTROL, g, force=100)

            cp_raw, _ = p.getBasePositionAndOrientation(cube)
            cp = np.array(cp_raw, dtype=float)

            # cube fell?
            if cp[2] < FLOOR_Z_THRESH:
                if cube_attached and cid is not None:
                    try:
                        p.removeConstraint(cid)
                    except:
                        pass
                    cube_attached, cid = False, None
                log_and_advance("fell_off_table", np.nan, cp)
                p.stepSimulation()
                continue

            # shadow under cube
            ray_res_cube = p.rayTest(cp_raw, [cp_raw[0], cp_raw[1], 0], collisionFilterMask=1)
            shadow_z = (ray_res_cube[0][3][2] + 0.001) if (ray_res_cube and ray_res_cube[0][0] != -1) else 0.001
            p.resetBasePositionAndOrientation(shadow_body, [cp_raw[0], cp_raw[1], shadow_z], [0, 0, 0, 1])

            # grasp point once per loop
            gp = get_grasp_point()

            # gripper shadow (dark) when NOT carrying
            if not cube_attached:
                ray_res_g = p.rayTest(gp, [gp[0], gp[1], 0], collisionFilterMask=1)
                if ray_res_g and ray_res_g[0][0] != -1:
                    gz = ray_res_g[0][3][2] + 0.001
                    p.resetBasePositionAndOrientation(gripper_shadow_body, [gp[0], gp[1], gz], [0, 0, 0, 1])
                else:
                    p.resetBasePositionAndOrientation(gripper_shadow_body, [0, 0, -1], [0, 0, 0, 1])
            else:
                # hide gripper shadow while carrying
                p.resetBasePositionAndOrientation(gripper_shadow_body, [0, 0, -1], [0, 0, 0, 1])


            # graspable + halo
            xy_dist = float(np.linalg.norm(gp[:2] - cp[:2]))
            z_dist = float(abs(gp[2] - cp[2]))
            graspable = (xy_dist < GRASP_XY_THRESH) and (z_dist < GRASP_Z_THRESH)

            halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
            p.changeVisualShape(halo_body, -1, rgbaColor=halo_color)
            p.resetBasePositionAndOrientation(halo_body, cp_raw, [0, 0, 0, 1])

            # attach / release
            if event == "CLOSE" and graspable and (not cube_attached):
                cid = create_fixed_constraint_preserve_pose_unimanual(robot, j_ids["HAND"], cube)
                cube_attached = True
                if time_to_grasp is None:
                    time_to_grasp = time.time() - trial_start

            elif event == "OPEN" and cube_attached:
                cube_pos_raw, _ = p.getBasePositionAndOrientation(cube)
                current_cube_pos_live = np.array(cube_pos_raw, dtype=float)

                try:
                    if cid:
                        p.removeConstraint(cid)
                except:
                    pass
                cube_attached, cid = False, None

                if current_cube_pos_live[2] < FLOOR_Z_THRESH:
                    log_and_advance("fell_off_table", np.nan, current_cube_pos_live)
                else:
                    dist_2d = float(np.linalg.norm(current_cube_pos_live[:2] - current_target_pos[:2]))
                    is_near = dist_2d < SUCCESS_THRESH

                    # match your unimanual: always nudge downward after release
                    p.resetBaseVelocity(cube, linearVelocity=[0, 0, -0.1])

                    if is_near:
                        log_and_advance("success", dist_2d, current_cube_pos_live)
                    else:
                        drops += 1
                        log_and_advance("missed_target", dist_2d, current_cube_pos_live)

            p.stepSimulation()

            # vision UI
            if display_frame is not None:
                hh, ww = display_frame.shape[:2]
                cv2.putText(display_frame,
                            f"UNIMANUAL | TRIAL: {trial_idx+1}/{len(trial_plan)} | diff:{current_difficulty} | run_id={RUN_ID}",
                            (10, hh - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_frame,
                            f"STATE: {p_state} " + (f"DROP IN {countdown:.1f}s" if countdown else ""),
                            (10, hh - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 165, 255) if countdown else (0, 255, 0), 2)
                cv2.putText(display_frame,
                            f"Drops: {drops} | D={current_D:.3f}",
                            (10, hh - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.imshow("Vision Feedback (Unimanual)", display_frame)
                if (cv2.waitKey(1) & 0xFF) == 27:
                    shared.is_running = False
                    break

        return (trial_idx >= len(trial_plan))

    finally:
        shared.is_running = False
        try:
            vt.join(timeout=2.0)
        except:
            pass
        cv2.destroyAllWindows()
        if p.isConnected():
            p.disconnect()

# ============================================================
# BIMANUAL (vision is your working fast version)
# ============================================================
class BimanualSharedState:
    def __init__(self):
        self.vel_cmd = {"BASE": 0.0, "SHOULDER": 0.0, "WRIST": 0.0, "ELBOW": 0.0}
        self.grip_pos = 0.04
        self.pinch_state_text = "OPEN"
        self.pinch_event = None
        self.release_countdown = None
        self.active_left = "NONE"
        self.active_right = "NONE"
        self.frame = None
        self.is_running = True
        self.lock = threading.Lock()

def bimanual_vision_worker(shared: BimanualSharedState,
                           cam_index=0,
                           deadzone_top=0.42,
                           deadzone_bottom=0.58,
                           joint_speed=2.0,
                           pinch_close=0.04,
                           pinch_open=0.07,
                           release_delay=0.6,
                           hold_seconds=0.12):
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print("[ERROR] Could not open webcam.")
        shared.is_running = False
        return

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils

    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    )

    pinch_state = "OPEN"
    release_start = None

    last_nonzero_v = {"BASE": 0.0, "SHOULDER": 0.0, "WRIST": 0.0, "ELBOW": 0.0}
    last_cmd_time = time.time()

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

    while shared.is_running:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        # region lines
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

        # deadzone overlay
        top_y = int(h * deadzone_top)
        bot_y = int(h * deadzone_bottom)
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, top_y), (w, bot_y), (60, 60, 60), -1)
        frame = cv2.addWeighted(overlay, 0.22, frame, 0.78, 0)

        v = {"BASE": 0.0, "SHOULDER": 0.0, "WRIST": 0.0, "ELBOW": 0.0}
        active_left = "NONE"
        active_right = "NONE"
        event = None
        countdown = None

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        hand_list = []
        pinch_candidates = []

        if results.multi_hand_landmarks:
            for hand_lm in results.multi_hand_landmarks:
                lms = hand_lm.landmark
                cx, cy = hand_center(lms)
                hand_list.append({"cx": cx, "cy": cy, "lms": lms, "hand_lm": hand_lm})

            # leftmost on screen = Left controller
            hand_list.sort(key=lambda d: d["cx"])
            for idx, hd in enumerate(hand_list[:2]):
                controller = "Left" if idx == 0 else "Right"
                lms = hd["lms"]
                cx, cy = hd["cx"], hd["cy"]
                px, py = int(cx * w), int(cy * h)

                mp_draw.draw_landmarks(frame, hd["hand_lm"], mp_hands.HAND_CONNECTIONS)
                cv2.circle(frame, (px, py), 6, (0, 255, 0), -1)

                region = region_from_cx(cx)
                cv2.putText(frame, f"{controller}: {region}",
                            (px - 55, py - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0), 2)

                direction = 0.0
                if cy < deadzone_top:
                    direction = +joint_speed
                elif cy > deadzone_bottom:
                    direction = -joint_speed

                if controller == "Left":
                    if region == "BASE":
                        v["BASE"] = direction
                        active_left = "BASE"
                    elif region == "SHOULDER":
                        v["SHOULDER"] = direction
                        active_left = "SHOULDER"
                else:
                    if region == "WRIST":
                        v["WRIST"] = direction
                        active_right = "WRIST"
                    elif region == "ELBOW":
                        v["ELBOW"] = direction
                        active_right = "ELBOW"

                pinch_candidates.append(pinch_value(lms))

        # pinch state machine
        if pinch_candidates:
            pmin = float(min(pinch_candidates))
            if pinch_state == "OPEN" and pmin < pinch_close:
                pinch_state = "CLOSED"
                event = "CLOSE"
                release_start = None
            elif pinch_state == "CLOSED":
                if pmin > pinch_open:
                    if release_start is None:
                        release_start = time.time()
                    countdown = release_delay - (time.time() - release_start)
                    if countdown <= 0:
                        pinch_state = "OPEN"
                        event = "OPEN"
                        release_start = None
                        countdown = None
                else:
                    release_start = None

        grip = 0.0 if pinch_state == "CLOSED" else 0.04

        # anti-flicker hold
        now = time.time()
        is_any_nonzero = any(abs(v[k]) > 1e-6 for k in v)
        if is_any_nonzero:
            last_nonzero_v = v.copy()
            last_cmd_time = now
        else:
            if (now - last_cmd_time) < hold_seconds:
                v = last_nonzero_v.copy()
                if active_left == "NONE" and active_right == "NONE":
                    active_left = "HOLD"
                    active_right = "HOLD"

        cv2.putText(frame,
                    f"STATE: {pinch_state}" + (f" (DROP IN {countdown:.1f}s)" if countdown else ""),
                    (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 165, 255) if countdown else (0, 255, 0), 2)

        with shared.lock:
            shared.vel_cmd = v
            shared.grip_pos = grip
            shared.pinch_state_text = pinch_state
            shared.release_countdown = countdown
            shared.active_left = active_left
            shared.active_right = active_right
            if event is not None:
                shared.pinch_event = event
            shared.frame = frame

    cap.release()

def run_bimanual(participant_id: str, trial_plan: list[dict], block_num: int):  
    global shared

    SUCCESS_THRESH = 0.065
    FLOOR_Z_THRESH = 0.06
    GRASP_XY_THRESH = 0.065
    GRASP_Z_THRESH = 0.055

    JOINT_SPEED = 2.0
    ARM_FORCE = 250
    GRIP_FORCE = 100

    DEADZONE_TOP = 0.42
    DEADZONE_BOTTOM = 0.58
    PINCH_CLOSE = 0.04
    PINCH_OPEN = 0.07
    RELEASE_DELAY = 0.6
    HOLD_SECONDS = 0.12

    shared = SharedState()

    # --- setup world ---
    W = setup_world()
    robot = W["robot"]
    cube = W["cube"]
    target_body = W["target_body"]
    halo_body = W["halo_body"]
    shadow_body = W["shadow_body"]
    gripper_proj_body = W["gripper_proj_body"]
    gripper_shadow_body = W["gripper_shadow_body"]
    j_ids = W["j_ids"]
    joint_name_to_id = W["joint_name_to_id"]
    neutral = W["neutral"]
    LEFT_FINGER_LINK = W["LEFT_FINGER_LINK"]
    RIGHT_FINGER_LINK = W["RIGHT_FINGER_LINK"]

    TABLE_X = W["TABLE_X"]; PICK_Y = W["PICK_Y"]
    BASE_CUBE_POS = W["BASE_CUBE_POS"]
    BASE_TARGET_POS = W["BASE_TARGET_POS"]

    # joints
    BASE_J = j_ids["BASE"]
    SHOULDER_J = j_ids["SHOULDER"]
    WRIST_J = j_ids["WRIST"]
    ELBOW_J = j_ids["ELBOW"]
    FINGER_L_J = j_ids["F_L"]
    FINGER_R_J = j_ids["F_R"]
    HAND_LINK = j_ids["HAND"]

    LOCKED_JOINTS = ["panda_joint3", "panda_joint5", "panda_joint7"]

    # logging (same columns)
    LOG_FILE = "results_bimanual_all.csv"
    HEADER = [
        "participant", "run_id", "block_num", "trial", "difficulty", "D",
        "time_total", "time_to_grasp", "placement_error",
        "success", "outcome", "drops",
        "cube_x", "cube_y", "cube_z",
        "target_x", "target_y", "target_z"
    ]
    ensure_csv_with_header(LOG_FILE, HEADER)
    RUN_ID = next_run_id(LOG_FILE, participant_id)
    print(f"[BIMANUAL] participant={participant_id} run_id={RUN_ID}")

    # level positions
    CUBE_POS_BY_LEVEL = {
        "easy":   np.array([TABLE_X - 0.06, PICK_Y + 0.02, BASE_CUBE_POS[2]], dtype=float),
        "medium": BASE_CUBE_POS.copy(),
        "hard":   np.array([TABLE_X + 0.06, PICK_Y - 0.06, BASE_CUBE_POS[2]], dtype=float),
    }
    FIXED_TARGET_POS = BASE_TARGET_POS.copy()

    trial = 0
    trial_start = time.time()
    cube_attached = False
    cid = None
    drops = 0
    time_to_grasp = None

    current_target_pos = FIXED_TARGET_POS.copy()
    current_difficulty = trial_plan[0]["difficulty"]
    current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty].copy()
    current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

    def set_trial(i: int):
        global shared
        nonlocal current_difficulty, current_cube_pos, current_D
        nonlocal trial_start, cube_attached, cid, drops, time_to_grasp

        current_difficulty = trial_plan[i]["difficulty"]
        current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty].copy()

        p.resetBasePositionAndOrientation(cube, current_cube_pos.tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(target_body, current_target_pos.tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(halo_body, current_cube_pos.tolist(), [0, 0, 0, 1])

        # --- NEW CODE START: Reset Robot to Start Position ---
        for j_name, j_val in W["neutral"].items():
            j_id = W["joint_name_to_id"][j_name]
            p.resetJointState(robot, j_id, j_val, targetVelocity=0)
            
        # Optional: Reset the shared velocity command to 0
        with shared.lock:
            shared.vel_cmd = {"BASE": 0.0, "SHOULDER": 0.0, "WRIST": 0.0, "ELBOW": 0.0}
        # --- NEW CODE END ---
        
        current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

        # reset per-trial (match unimanual behavior)
        trial_start = time.time()
        cube_attached = False
        cid = None
        drops = 0
        time_to_grasp = None

    def get_grasp_point():
        if LEFT_FINGER_LINK is not None and RIGHT_FINGER_LINK is not None:
            lf = np.array(p.getLinkState(robot, LEFT_FINGER_LINK)[0])
            rf = np.array(p.getLinkState(robot, RIGHT_FINGER_LINK)[0])
            return (lf + rf) * 0.5
        return np.array(p.getLinkState(robot, HAND_LINK)[0])

    def log_and_next(outcome: str, placement_error: float, cube_xyz: np.ndarray):
        nonlocal trial
        time_total = time.time() - trial_start
        success = 1 if outcome == "success" else 0

        with open(LOG_FILE, "a", newline="") as f:
            csv.writer(f).writerow([
                participant_id, RUN_ID, block_num, trial, current_difficulty, round(current_D, 4),
                round(time_total, 3),
                round(time_to_grasp if time_to_grasp is not None else time_total, 3),
                (round(float(placement_error), 4) if placement_error == placement_error else np.nan),
                success, outcome, drops,
                round(float(cube_xyz[0]), 4), round(float(cube_xyz[1]), 4), round(float(cube_xyz[2]), 4),
                round(float(current_target_pos[0]), 4), round(float(current_target_pos[1]), 4), round(float(current_target_pos[2]), 4),
            ])

        trial += 1
        if trial < len(trial_plan):
            set_trial(trial)

    set_trial(0)

    # vision thread
    shared = BimanualSharedState()
    vt = threading.Thread(
        target=bimanual_vision_worker,
        args=(shared,),
        kwargs=dict(
            cam_index=0,
            deadzone_top=DEADZONE_TOP,
            deadzone_bottom=DEADZONE_BOTTOM,
            joint_speed=JOINT_SPEED,
            pinch_close=PINCH_CLOSE,
            pinch_open=PINCH_OPEN,
            release_delay=RELEASE_DELAY,
            hold_seconds=HOLD_SECONDS,
        ),
        daemon=True
    )
    vt.start()

    cv2.namedWindow("Bimanual Vision (threaded)", cv2.WINDOW_NORMAL)

    try:
        while shared.is_running and p.isConnected():
            if trial >= len(trial_plan):
                print(f"[DONE] Completed {len(trial_plan)} bimanual trials.")
                break

            with shared.lock:
                frame = None if shared.frame is None else shared.frame.copy()
                vcmd = shared.vel_cmd.copy()
                grip = shared.grip_pos
                event = shared.pinch_event
                shared.pinch_event = None
                active_left = shared.active_left
                active_right = shared.active_right
                pinch_state = shared.pinch_state_text

            # UI
            if frame is not None:
                h, w = frame.shape[:2]
                cv2.putText(frame,
                            f"BIMANUAL | TRIAL {trial+1}/{len(trial_plan)} | diff:{current_difficulty} | D={current_D:.3f} | run_id={RUN_ID}",
                            (10, h - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame,
                            f"L:{active_left}  R:{active_right}  pinch:{pinch_state} | Drops:{drops}",
                            (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Bimanual Vision (threaded)", frame)

            if (cv2.waitKey(1) & 0xFF) == 27:
                shared.is_running = False
                return False

            # apply velocity control
            p.setJointMotorControl2(robot, BASE_J, p.VELOCITY_CONTROL, targetVelocity=vcmd["BASE"], force=ARM_FORCE)
            p.setJointMotorControl2(robot, SHOULDER_J, p.VELOCITY_CONTROL, targetVelocity=vcmd["SHOULDER"], force=ARM_FORCE)
            p.setJointMotorControl2(robot, WRIST_J, p.VELOCITY_CONTROL, targetVelocity=vcmd["WRIST"], force=ARM_FORCE)
            p.setJointMotorControl2(robot, ELBOW_J, p.VELOCITY_CONTROL, targetVelocity=vcmd["ELBOW"], force=ARM_FORCE)

            p.setJointMotorControl2(robot, FINGER_L_J, p.POSITION_CONTROL, grip, force=GRIP_FORCE)
            p.setJointMotorControl2(robot, FINGER_R_J, p.POSITION_CONTROL, grip, force=GRIP_FORCE)

            # lock joints
            for jn in LOCKED_JOINTS:
                jid = joint_name_to_id[jn]
                p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, neutral[jn], force=ARM_FORCE)

            # cube fell? (MATCH UNIMANUAL)
            cube_pos_live = np.array(p.getBasePositionAndOrientation(cube)[0], dtype=float)
            if cube_pos_live[2] < FLOOR_Z_THRESH:
                if cube_attached and cid is not None:
                    try:
                        p.removeConstraint(cid)
                    except:
                        pass
                    cube_attached = False
                    cid = None
                log_and_next("fell_off_table", np.nan, cube_pos_live)
                p.stepSimulation()
                continue

            # shadow under cube
            cp_raw, _ = p.getBasePositionAndOrientation(cube)
            ray_res_cube = p.rayTest(cp_raw, [cp_raw[0], cp_raw[1], 0], collisionFilterMask=1)
            shadow_z = (ray_res_cube[0][3][2] + 0.001) if (ray_res_cube and ray_res_cube[0][0] != -1) else 0.001
            p.resetBasePositionAndOrientation(shadow_body, [cp_raw[0], cp_raw[1], shadow_z], [0, 0, 0, 1])

            # grasp point once
            gp = get_grasp_point()

            # gripper shadow (dark) when NOT carrying
            if not cube_attached:
                ray_res_g = p.rayTest(gp, [gp[0], gp[1], 0], collisionFilterMask=1)
                if ray_res_g and ray_res_g[0][0] != -1:
                    gz = ray_res_g[0][3][2] + 0.001
                    p.resetBasePositionAndOrientation(gripper_shadow_body, [gp[0], gp[1], gz], [0, 0, 0, 1])
                else:
                    p.resetBasePositionAndOrientation(gripper_shadow_body, [0, 0, -1], [0, 0, 0, 1])
            else:
                p.resetBasePositionAndOrientation(gripper_shadow_body, [0, 0, -1], [0, 0, -1, 1] if False else [0,0,0,1])

            # graspable + halo (MATCH UNIMANUAL thresholds)
            xy_dist = float(np.linalg.norm(gp[:2] - cube_pos_live[:2]))
            z_dist = float(abs(gp[2] - cube_pos_live[2]))
            graspable = (xy_dist < GRASP_XY_THRESH) and (z_dist < GRASP_Z_THRESH)

            halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
            p.changeVisualShape(halo_body, -1, rgbaColor=halo_color)
            p.resetBasePositionAndOrientation(halo_body, cube_pos_live.tolist(), [0, 0, 0, 1])

            # attach
            if event == "CLOSE" and (not cube_attached) and graspable:
                cid = create_fixed_constraint_preserve_pose(robot, HAND_LINK, cube)
                cube_attached = True
                if time_to_grasp is None:
                    time_to_grasp = time.time() - trial_start

            # release (MATCH UNIMANUAL outcome logic)
            if event == "OPEN" and cube_attached:
                current_cube_pos_live = np.array(p.getBasePositionAndOrientation(cube)[0], dtype=float)

                # remove constraint first
                try:
                    if cid is not None:
                        p.removeConstraint(cid)
                except:
                    pass
                cube_attached = False
                cid = None

                if current_cube_pos_live[2] < FLOOR_Z_THRESH:
                    log_and_next("fell_off_table", np.nan, current_cube_pos_live)
                else:
                    dist_2d = float(np.linalg.norm(current_cube_pos_live[:2] - current_target_pos[:2]))
                    is_near = dist_2d < SUCCESS_THRESH

                    # match unimanual: always nudge downward after release
                    p.resetBaseVelocity(cube, linearVelocity=[0, 0, -0.1])

                    if is_near:
                        log_and_next("success", dist_2d, current_cube_pos_live)
                    else:
                        drops += 1
                        log_and_next("missed_target", dist_2d, current_cube_pos_live)

            p.stepSimulation()

        return True

    finally:
        shared.is_running = False
        try:
            vt.join(timeout=2.0)
        except:
            pass
        cv2.destroyAllWindows()
        if p.isConnected():
            p.disconnect()

if __name__ == "__main__":
    # 1. Get next number automatically
    p_num = get_next_participant_number()
    
    # 2. Generate ID and get the correct Order automatically
    PARTICIPANT_ID, GROUP, EXPERIMENT_ORDER = generate_participant_id(p_num)

    # 3. Setup Trial Plan (shuffled difficulties)
    seed = int(uuid.uuid4().hex[:8], 16)
    rng = random.Random(seed)
    TRIAL_PLAN = make_trial_plan_shuffled(rng, reps=2, levels=("easy", "medium", "hard"))

    print("="*60)
    print(f"SESSION DETAILS")
    print(f"Participant: {PARTICIPANT_ID}")
    print(f"Group:       {GROUP}")
    print(f"Order:       {EXPERIMENT_ORDER[0][1]} -> {EXPERIMENT_ORDER[1][1]}")
    print("="*60)

    # 4. Run the experiments in the assigned order
    for idx, (func, name) in enumerate(EXPERIMENT_ORDER):
        print(f"\n>>> [{idx+1}/2] Starting {name}...")
        
        finished = func(PARTICIPANT_ID, TRIAL_PLAN, block_num=(idx + 1))
        
        if not finished:
            print(f"[STOP] {name} ended early. Stopping session.")
            break
        
        print(f">>> {name} Finished.")
        time.sleep(1.0) # brief pause for physics cleanup

    print("\n[DONE] Session Complete.")
