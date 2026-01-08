import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import mediapipe as mp
import threading
import csv
import os


# ============================================================
# SHARED HELPERS
# ============================================================
def create_table(x, y, top_z, collision_group=1, collision_mask=-1):
    half = [0.22, 0.22, 0.05]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=[0.75, 0.75, 0.75, 1])
    t_id = p.createMultiBody(0, col, vis, [x, y, top_z - half[2]])
    p.setCollisionFilterGroupMask(t_id, -1, collision_group, collision_mask)
    return t_id


# ============================================================
# UNIMANUAL EXPERIMENT (your version, wrapped)
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


def run_unimanual():
    # -----------------------------
    # Constants (same as yours)
    # -----------------------------
    DEADZONE_TOP = 0.42
    DEADZONE_BOTTOM = 0.58
    JOINT_SPEED = 2
    MAX_FORCE = 250
    RELEASE_DELAY = 0.6
    SUCCESS_THRESH = 0.065
    FLOOR_Z_THRESH = 0.06

    shared = SharedState()

    # -----------------------------
    # Vision worker (same logic)
    # -----------------------------
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

    # -----------------------------
    # PYBULLET setup (same scene)
    # -----------------------------
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(fixedTimeStep=1.0 / 240.0, numSolverIterations=50)

    p.resetDebugVisualizerCamera(cameraDistance=1.4, cameraYaw=90, cameraPitch=-25,
                                 cameraTargetPosition=[0.5, 0, 0.3])

    plane_id = p.loadURDF("plane.urdf")
    p.setCollisionFilterGroupMask(plane_id, -1, collisionFilterGroup=1, collisionFilterMask=-1)

    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

    joint_name_to_id = {}
    link_name_to_id = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        joint_name_to_id[info[1].decode()] = i
        link_name_to_id[info[12].decode()] = i

    j_ids = {"BASE": 0, "SHOULDER": 1, "ELBOW": 3, "WRIST": 5, "F_L": 9, "F_R": 10, "HAND": 8}
    HAND_LINK = link_name_to_id["panda_link8"]

    neutral = {
        "panda_joint1": 0.0, "panda_joint2": -0.4, "panda_joint3": 0.0,
        "panda_joint4": -2.0, "panda_joint5": 0.0, "panda_joint6": 1.7, "panda_joint7": 0.8
    }
    LOCKED_JOINTS = {"panda_joint3": 0.0, "panda_joint5": 0.0, "panda_joint7": 0.8}
    locked_ids = {joint_name_to_id[name]: val for name, val in LOCKED_JOINTS.items()}
    for j_name, val in neutral.items():
        p.resetJointState(robot, joint_name_to_id[j_name], val)

    TABLE_X, PICK_Y, PLACE_Y, TABLE_Z = 0.48, -0.28, 0.28, 0.26
    create_table(TABLE_X, PICK_Y, TABLE_Z)
    create_table(TABLE_X, PLACE_Y, TABLE_Z)

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

    # -----------------------------
    # Logging (same as your unimanual)
    # -----------------------------
    PARTICIPANT_ID = "P01"
    LOG_FILE = f"results_{PARTICIPANT_ID}_threaded.csv"

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
        "medium": BASE_CUBE_POS.copy(),
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
        nonlocal current_difficulty, current_cube_pos, current_target_pos, current_D
        current_difficulty = TRIAL_PLAN[idx]["difficulty"]
        current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty].copy()
        current_target_pos = FIXED_TARGET_POS.copy()

        p.resetBasePositionAndOrientation(cube, current_cube_pos.tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(target_body, current_target_pos.tolist(), [0, 0, 0, 1])

        current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

    def log_and_advance(outcome, placement_error, cube_pos_np):
        nonlocal trial_idx, trial_start, drops, time_to_grasp
        time_total = time.time() - trial_start
        success = 1 if outcome == "success" else 0

        with open(LOG_FILE, "a", newline="") as f:
            csv.writer(f).writerow([
                PARTICIPANT_ID, trial_idx, current_difficulty,
                round(current_D, 4),
                round(time_total, 3),
                round(time_to_grasp if time_to_grasp is not None else time_total, 3),
                (round(float(placement_error), 4) if placement_error == placement_error else np.nan),
                success, outcome, drops,
                *np.round(cube_pos_np, 4), *np.round(current_target_pos, 4)
            ])

        trial_idx += 1
        if trial_idx < len(TRIAL_PLAN):
            set_trial_condition(trial_idx)
            trial_start = time.time()
            drops, time_to_grasp = 0, None

    current_difficulty = TRIAL_PLAN[0]["difficulty"]
    current_cube_pos = BASE_CUBE_POS.copy()
    current_target_pos = FIXED_TARGET_POS.copy()
    current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))
    set_trial_condition(0)

    # -----------------------------
    # Start thread
    # -----------------------------
    vt = threading.Thread(target=vision_worker, args=(shared,), daemon=True)
    vt.start()

    last_halo_color = [1, 0, 0, 0.18]
    last_halo_pos = np.array([0, 0, 0], dtype=float)

    completed_all = False

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

            for name, jid in j_ids.items():
                if name in v:
                    p.setJointMotorControl2(robot, jid, p.VELOCITY_CONTROL,
                                            targetVelocity=v[name], force=MAX_FORCE)

            for jid, pos in locked_ids.items():
                p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL,
                                        targetPosition=pos, force=MAX_FORCE)

            p.setJointMotorControl2(robot, j_ids["F_L"], p.POSITION_CONTROL, g, force=100)
            p.setJointMotorControl2(robot, j_ids["F_R"], p.POSITION_CONTROL, g, force=100)

            # cube pos
            cp_raw, _ = p.getBasePositionAndOrientation(cube)
            cp = np.array(cp_raw)

            # fell off table => auto end trial
            if cp[2] < FLOOR_Z_THRESH:
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

            # shadow
            ray_res_cube = p.rayTest(cp_raw, [cp_raw[0], cp_raw[1], 0], collisionFilterMask=1)
            shadow_z = (ray_res_cube[0][3][2] + 0.001) if (ray_res_cube and ray_res_cube[0][0] != -1) else 0.001
            p.resetBasePositionAndOrientation(shadow_body, [cp_raw[0], cp_raw[1], shadow_z], [0, 0, 0, 1])

            # gripper proj
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

            # graspable
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

            # events
            if event == "CLOSE":
                if graspable and not cube_attached:
                    cid = create_fixed_constraint_preserve_pose(robot, HAND_LINK, cube)
                    cube_attached = True
                    if time_to_grasp is None:
                        time_to_grasp = time.time() - trial_start

            elif event == "OPEN" and cube_attached:
                cube_pos_raw, _ = p.getBasePositionAndOrientation(cube)
                current_cube_pos_live = np.array(cube_pos_raw)

                if current_cube_pos_live[2] < FLOOR_Z_THRESH:
                    if cid:
                        p.removeConstraint(cid)
                    cube_attached = False
                    cid = None
                    log_and_advance("fell_off_table", np.nan, current_cube_pos_live)
                else:
                    dist_2d = float(np.linalg.norm(current_cube_pos_live[:2] - current_target_pos[:2]))
                    is_near = dist_2d < SUCCESS_THRESH

                    if cid:
                        p.removeConstraint(cid)
                    cube_attached = False
                    cid = None
                    p.resetBaseVelocity(cube, linearVelocity=[0, 0, -0.1])

                    if is_near:
                        log_and_advance("success", dist_2d, current_cube_pos_live)
                    else:
                        drops += 1
                        log_and_advance("missed_target", dist_2d, current_cube_pos_live)

            p.stepSimulation()

            if display_frame is not None:
                hh, ww = display_frame.shape[:2]
                cv2.putText(display_frame,
                            f"UNIMANUAL | TRIAL: {trial_idx+1}/{len(TRIAL_PLAN)} | difficulty: {current_difficulty}",
                            (10, hh - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_frame,
                            f"STATE: {p_state} " + (f"DROP IN {countdown:.1f}s" if countdown else ""),
                            (10, hh - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 165, 255) if countdown else (0, 255, 0), 2)
                cv2.putText(display_frame,
                            f"Drops: {drops} | D={current_D:.3f}",
                            (10, hh - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.imshow("Vision Feedback", display_frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    shared.is_running = False
                    break

        completed_all = (trial_idx >= len(TRIAL_PLAN))

    finally:
        shared.is_running = False
        try:
            vt.join(timeout=2.0)
        except:
            pass
        cv2.destroyAllWindows()
        if p.isConnected():
            p.disconnect()

    return completed_all


# ============================================================
# BIMANUAL EXPERIMENT (your version, already updated, wrapped)
# ============================================================
def run_bimanual():
    # --- thresholds (same as your bimanual updated)
    SUCCESS_THRESH = 0.05
    FLOOR_Z_THRESH = 0.06

    # ------------------------------------------------------------
    # PYBULLET SETUP (fresh)
    # ------------------------------------------------------------
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(fixedTimeStep=1.0 / 240.0, numSolverIterations=200)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.4, cameraYaw=90, cameraPitch=-25,
        cameraTargetPosition=[0.5, 0, 0.3]
    )

    p.loadURDF("plane.urdf")

    # ------------------------------------------------------------
    # LOAD PANDA
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # NEUTRAL
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # TABLES
    # ------------------------------------------------------------
    TABLE_X = 0.48
    PICK_Y, PLACE_Y = -0.28, 0.28
    TABLE_Z = 0.26

    create_table(TABLE_X, PICK_Y, TABLE_Z, collision_group=1, collision_mask=-1)
    create_table(TABLE_X, PLACE_Y, TABLE_Z, collision_group=1, collision_mask=-1)

    # ------------------------------------------------------------
    # CUBE + TARGET
    # ------------------------------------------------------------
    CUBE_SIZE = 0.05
    BASE_CUBE_POS = np.array([TABLE_X, PICK_Y + 0.03, TABLE_Z + CUBE_SIZE / 2], dtype=float)
    BASE_TARGET_POS = np.array([TABLE_X, PLACE_Y - 0.03, TABLE_Z + CUBE_SIZE / 2], dtype=float)

    cube = p.loadURDF("cube_small.urdf", BASE_CUBE_POS.tolist(), globalScaling=0.8)

    target_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.002, rgbaColor=[1, 0, 0, 0.7])
    target_body = p.createMultiBody(0, baseVisualShapeIndex=target_vis, basePosition=BASE_TARGET_POS.tolist())

    # ------------------------------------------------------------
    # MEDIAPIPE
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # Control params
    # ------------------------------------------------------------
    JOINT_VEL = {"BASE": 2.0, "SHOULDER": 2.0, "WRIST": 2.0, "ELBOW": 2.0}
    DEADZONE_TOP = 0.42
    DEADZONE_BOTTOM = 0.58
    PINCH_CLOSE = 0.04
    PINCH_OPEN = 0.07
    ARM_FORCE = 250
    GRIP_FORCE = 100

    # Assisted grasp
    GRASP_XY_THRESH = 0.065
    GRASP_Z_THRESH = 0.055

    halo_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.075, rgbaColor=[1, 0, 0, 0.18])
    halo_body = p.createMultiBody(0, baseVisualShapeIndex=halo_vis, basePosition=BASE_CUBE_POS.tolist())

    # ------------------------------------------------------------
    # Experiment plan (difficulty only)
    # ------------------------------------------------------------
    PARTICIPANT_ID = "P01"
    LOG_FILE = f"results_{PARTICIPANT_ID}_bimanual.csv"

    N_REPS_PER_LEVEL = 2
    LEVELS = ["easy", "medium", "hard"]

    CUBE_POS_BY_LEVEL = {
        "easy":   np.array([TABLE_X - 0.06, PICK_Y + 0.02, BASE_CUBE_POS[2]], dtype=float),
        "medium": BASE_CUBE_POS.copy(),
        "hard":   np.array([TABLE_X + 0.06, PICK_Y - 0.06, BASE_CUBE_POS[2]], dtype=float),
    }
    FIXED_TARGET_POS = BASE_TARGET_POS.copy()

    TRIAL_PLAN = []
    for _ in range(N_REPS_PER_LEVEL):
        for lvl in LEVELS:
            TRIAL_PLAN.append({"difficulty": lvl})
    N_TRIALS = len(TRIAL_PLAN)

    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            csv.writer(f).writerow([
                "participant",
                "trial",
                "difficulty",
                "D",
                "time_total",
                "time_to_grasp",
                "placement_error",
                "success",
                "outcome",
                "drops",
                "cube_x", "cube_y", "cube_z",
                "target_x", "target_y", "target_z",
            ])

    trial = 0
    trial_start = time.time()
    cube_attached = False
    cid = None
    drops = 0
    time_to_grasp = None

    current_difficulty = TRIAL_PLAN[0]["difficulty"]
    current_cube_pos = BASE_CUBE_POS.copy()
    current_target_pos = FIXED_TARGET_POS.copy()
    current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

    def reset_trial_state():
        nonlocal trial_start, cube_attached, cid, drops, time_to_grasp
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

    def set_trial_condition(trial_idx: int):
        nonlocal current_difficulty, current_cube_pos, current_target_pos, current_D
        current_difficulty = TRIAL_PLAN[trial_idx]["difficulty"]
        current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty].copy()
        current_target_pos = FIXED_TARGET_POS.copy()

        p.resetBasePositionAndOrientation(cube, current_cube_pos.tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(target_body, current_target_pos.tolist(), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(halo_body, current_cube_pos.tolist(), [0, 0, 0, 1])

        current_D = float(np.linalg.norm(current_cube_pos - current_target_pos))

    def log_and_advance(outcome: str, placement_error: float, cube_pos_xyz):
        nonlocal trial
        time_total = time.time() - trial_start
        success = 1 if outcome == "success" else 0

        with open(LOG_FILE, "a", newline="") as f:
            csv.writer(f).writerow([
                PARTICIPANT_ID,
                trial,
                current_difficulty,
                round(current_D, 4),
                round(time_total, 3),
                round(time_to_grasp if time_to_grasp is not None else time_total, 3),
                (round(float(placement_error), 4) if placement_error == placement_error else np.nan),
                success,
                outcome,
                drops,
                round(float(cube_pos_xyz[0]), 4), round(float(cube_pos_xyz[1]), 4), round(float(cube_pos_xyz[2]), 4),
                round(float(current_target_pos[0]), 4), round(float(current_target_pos[1]), 4), round(float(current_target_pos[2]), 4),
            ])

        trial += 1
        if trial < N_TRIALS:
            set_trial_condition(trial)
            reset_trial_state()

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

    set_trial_condition(0)
    reset_trial_state()

    pinch_state = "OPEN"
    release_start_time = None
    RELEASE_DELAY = 0.6

    try:
        while True:
            if trial >= N_TRIALS:
                print(f"[DONE] Completed {N_TRIALS} bimanual trials.")
                break

            if not p.isConnected():
                print("[ERROR] PyBullet disconnected. Exiting cleanly.")
                break

            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape

            # AUTO FAIL if fell off table
            cube_pos_live, _ = p.getBasePositionAndOrientation(cube)
            cube_pos_live = np.array(cube_pos_live, dtype=float)

            if cube_pos_live[2] < FLOOR_Z_THRESH:
                if cube_attached and cid is not None:
                    try:
                        p.removeConstraint(cid)
                    except:
                        pass
                    cube_attached = False
                    cid = None
                log_and_advance("fell_off_table", np.nan, cube_pos_live)
                p.stepSimulation()
                time.sleep(1 / 240)
                continue

            # UI regions
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

            # deadzone
            top_y = int(h * DEADZONE_TOP)
            bot_y = int(h * DEADZONE_BOTTOM)
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, top_y), (w, bot_y), (60, 60, 60), -1)
            frame = cv2.addWeighted(overlay, 0.22, frame, 0.78, 0)

            joint_vel = {BASE_J: 0.0, SHOULDER_J: 0.0, WRIST_J: 0.0, ELBOW_J: 0.0}

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

                    label = handedness.classification[0].label
                    lms = hand_lm.landmark

                    cx, cy = hand_center(lms)
                    px, py = int(cx * w), int(cy * h)
                    cv2.circle(frame, (px, py), 6, (0, 255, 0), -1)

                    region = region_from_cx(cx)
                    cv2.putText(frame, f"{label}: {region}", (px - 55, py - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                    direction = 0
                    if cy < DEADZONE_TOP:
                        direction = +1
                    elif cy > DEADZONE_BOTTOM:
                        direction = -1

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

                    pinch_candidates.append(pinch_value(lms))

            if pinch_candidates:
                pinch_val_best = float(min(pinch_candidates))

                if pinch_state == "OPEN" and pinch_val_best < PINCH_CLOSE:
                    pinch_state = "CLOSED"
                    pinch_close_event = True
                    release_start_time = None

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

            # Apply control
            p.setJointMotorControl2(robot, BASE_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[BASE_J], force=ARM_FORCE)
            p.setJointMotorControl2(robot, SHOULDER_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[SHOULDER_J], force=ARM_FORCE)
            p.setJointMotorControl2(robot, WRIST_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[WRIST_J], force=ARM_FORCE)
            p.setJointMotorControl2(robot, ELBOW_J, p.VELOCITY_CONTROL, targetVelocity=joint_vel[ELBOW_J], force=ARM_FORCE)

            p.setJointMotorControl2(robot, FINGER_L_J, p.POSITION_CONTROL, grip, force=GRIP_FORCE)
            p.setJointMotorControl2(robot, FINGER_R_J, p.POSITION_CONTROL, grip, force=GRIP_FORCE)

            for jn in LOCKED_JOINTS:
                jid = joint_name_to_id[jn]
                p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, neutral[jn], force=ARM_FORCE)

            # graspable
            cube_pos, _ = p.getBasePositionAndOrientation(cube)
            cube_pos_np = np.array(cube_pos, dtype=float)
            gp = get_grasp_point()
            xy_dist = float(np.linalg.norm(gp[:2] - cube_pos_np[:2]))
            z_dist = float(abs(gp[2] - cube_pos_np[2]))
            graspable = (xy_dist < GRASP_XY_THRESH) and (z_dist < GRASP_Z_THRESH)

            halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]
            p.changeVisualShape(halo_body, -1, rgbaColor=halo_color)
            p.resetBasePositionAndOrientation(halo_body, cube_pos, [0, 0, 0, 1])

            # attach
            if pinch_close_event and (not cube_attached) and graspable:
                cid = create_fixed_constraint_preserve_pose(robot, HAND_LINK, cube)
                cube_attached = True
                if time_to_grasp is None:
                    time_to_grasp = time.time() - trial_start

            # release ends trial
            if pinch_open_event and cube_attached:
                current_cube_pos_live = np.array(p.getBasePositionAndOrientation(cube)[0], dtype=float)

                if current_cube_pos_live[2] < FLOOR_Z_THRESH:
                    outcome = "fell_off_table"
                    placement_error = np.nan
                else:
                    dist_2d = float(np.linalg.norm(current_cube_pos_live[:2] - current_target_pos[:2]))
                    placement_error = dist_2d
                    outcome = "success" if dist_2d < SUCCESS_THRESH else "missed_target"

                try:
                    p.removeConstraint(cid)
                except:
                    pass
                cube_attached = False
                cid = None

                if outcome == "missed_target":
                    drops += 1
                    p.resetBaseVelocity(cube, linearVelocity=[0, 0, -0.1])

                log_and_advance(outcome, placement_error, current_cube_pos_live)

            # HUD
            cv2.putText(frame,
                        f"BIMANUAL | TRIAL: {trial+1}/{N_TRIALS} | difficulty={current_difficulty} | D={current_D:.2f}",
                        (10, h - 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"LEFT: {active_left} | RIGHT: {active_right}",
                        (10, h - 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            t2g_txt = "--" if time_to_grasp is None else f"{time_to_grasp:.2f}s"
            cv2.putText(frame, f"drops={drops} | t_grasp={t2g_txt}",
                        (10, h - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

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


# ============================================================
# RUN BOTH SEQUENTIALLY
# ============================================================
if __name__ == "__main__":
    print("[1/2] Starting UNIMANUAL experiment...")
    finished_unimanual = run_unimanual()

    if not finished_unimanual:
        print("[STOP] Unimanual ended early (ESC or stop). Not starting bimanual.")
    else:
        print("[2/2] Unimanual complete. Starting BIMANUAL experiment...")
        run_bimanual()

    print("[DONE] All experiments finished.")
