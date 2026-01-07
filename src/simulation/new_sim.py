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

# =============================
# VISION WORKER THREAD
# =============================
def vision_worker(shared_state):
    cap = cv2.VideoCapture(0)
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
    
    pinch_internal_state = "OPEN"
    release_start_time = None
    
    while shared_state.is_running:
        ret, frame = cap.read()
        if not ret: break
        
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
        cv2.rectangle(overlay, (0, top_y), (w, bot_y), (60,60,60), -1)
        frame = cv2.addWeighted(overlay, 0.22, frame, 0.78, 0)
        
        splits = [0.25, 0.50, 0.75]
        labels = ["BASE", "SHOULDER", "WRIST", "ELBOW"]
        for s in splits:
            cv2.line(frame, (int(w*s), 0), (int(w*s), h), (255,255,255), 1)
        for i, l in enumerate(labels):
            cv2.putText(frame, l, (int(w*(i*0.25+0.125))-30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        if res.multi_hand_landmarks:
            lm = res.multi_hand_landmarks[0].landmark
            # Calculate the tracked point (mean of all landmarks)
            cx, cy = np.mean([pt.x for pt in lm]), np.mean([pt.y for pt in lm])
            
            # Convert normalized (0-1) coordinates to pixel coordinates
            pixel_x, pixel_y = int(cx * w), int(cy * h)

            # Draw only the tracking point
            # A solid circle with a white border for visibility
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
            if event: shared_state.pinch_event = event
            shared_state.frame = frame

    cap.release()

# =============================
# MAIN THREAD (PYBULLET & LOGIC)
# =============================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(fixedTimeStep=1.0/240.0, numSolverIterations=50)

p.resetDebugVisualizerCamera(cameraDistance=1.4, cameraYaw=90, cameraPitch=-25, cameraTargetPosition=[0.5, 0, 0.3])
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
neutral = {"panda_joint1": 0.0, "panda_joint2": -0.4, "panda_joint3": 0.0, "panda_joint4": -2.0, "panda_joint5": 0.0, "panda_joint6": 1.7, "panda_joint7": 0.8}
LOCKED_JOINTS = {"panda_joint3": 0.0, "panda_joint5": 0.0, "panda_joint7": 0.8}
locked_ids = {joint_name_to_id[name]: val for name, val in LOCKED_JOINTS.items()}
for j_name, val in neutral.items(): p.resetJointState(robot, joint_name_to_id[j_name], val)

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


# Create a flat black shadow (semi-transparent)
shadow_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.025, length=0.001, rgbaColor=[0, 0, 0, 0.4])
shadow_body = p.createMultiBody(0, baseVisualShapeIndex=shadow_vis, basePosition=[0, 0, 0])


# Create a gripper projection (e.g., a blue or white ring)
# This will follow the gripper only when the cube IS NOT attached
gripper_proj_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.03, length=0.001, rgbaColor=[1, 1, 1, 0.4])
gripper_proj_body = p.createMultiBody(0, baseVisualShapeIndex=gripper_proj_vis, basePosition=[0, 0, -1]) # Start hidde

# Ensure the shadow doesn't collide with anything
p.setCollisionFilterGroupMask(shadow_body, -1, 0, 0)

# =============================
# EXPERIMENT SETUP
# =============================
PARTICIPANT_ID = "P01"
LOG_FILE = f"results_{PARTICIPANT_ID}_threaded.csv"
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow(["participant","trial","factor","difficulty","D","time_total","time_to_grasp","placement_error","success","grasp_attempts","drops","cube_x","cube_y","cube_z","target_x","target_y","target_z"])

LEVELS = ["easy", "medium", "hard"]
TARGET_Y_BY_LEVEL = {"easy": float(BASE_TARGET_POS[1] - 0.10), "medium": float(BASE_TARGET_POS[1]), "hard": float(BASE_TARGET_POS[1] + 0.10)}
CUBE_POS_BY_LEVEL = {"easy": np.array([TABLE_X-0.06, PICK_Y+0.02, BASE_CUBE_POS[2]]), "medium": BASE_CUBE_POS, "hard": np.array([TABLE_X+0.06, PICK_Y-0.06, BASE_CUBE_POS[2]])}

TRIAL_PLAN = []
for _ in range(2):
    for lvl in LEVELS: TRIAL_PLAN.append({"factor": "transport_distance", "difficulty": lvl})
    for lvl in LEVELS: TRIAL_PLAN.append({"factor": "grasp_difficulty", "difficulty": lvl})

trial_idx = 0
trial_start = time.time()
cube_attached, cid = False, None
grasp_attempts, drops, time_to_grasp = 0, 0, None

def get_grasp_point():
    # Calculates the midpoint between the two gripper fingers
    lf = np.array(p.getLinkState(robot, j_ids["F_L"])[0])
    rf = np.array(p.getLinkState(robot, j_ids["F_R"])[0])
    return (lf + rf) * 0.5

def create_fixed_constraint_preserve_pose(parent_robot, parent_link, child_body):
    # Gets current relative transform so cube doesn't "snap" or jump
    ee_pos, ee_orn = p.getLinkState(parent_robot, parent_link)[:2]
    cube_pos, cube_orn = p.getBasePositionAndOrientation(child_body)
    inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
    local_pos, local_orn = p.multiplyTransforms(inv_ee_pos, inv_ee_orn, cube_pos, cube_orn)
    return p.createConstraint(parent_robot, parent_link, child_body, -1, 
                              p.JOINT_FIXED, [0, 0, 0], local_pos, [0, 0, 0], local_orn)


def set_trial_condition(idx):
    global current_factor, current_difficulty, current_cube_pos, current_target_pos, current_D
    cond = TRIAL_PLAN[idx]
    current_factor, current_difficulty = cond["factor"], cond["difficulty"]
    current_cube_pos = CUBE_POS_BY_LEVEL[current_difficulty] if current_factor == "grasp_difficulty" else BASE_CUBE_POS
    current_target_pos = np.array([BASE_TARGET_POS[0], TARGET_Y_BY_LEVEL[current_difficulty], BASE_TARGET_POS[2]]) if current_factor == "transport_distance" else BASE_TARGET_POS
    p.resetBasePositionAndOrientation(cube, current_cube_pos.tolist(), [0,0,0,1])
    p.resetBasePositionAndOrientation(target_body, current_target_pos.tolist(), [0,0,0,1])
    current_D = np.linalg.norm(current_cube_pos - current_target_pos)

set_trial_condition(0)

# =============================
# RUNTIME
# =============================
vt = threading.Thread(target=vision_worker, args=(shared,), daemon=True)
vt.start()


last_halo_color = [1, 0, 0, 0.18]
last_halo_pos = np.array([0, 0, 0])


p.changeVisualShape(gripper_proj_body, -1, rgbaColor=[1, 1, 1, 0.6])
p.changeVisualShape(shadow_body, -1, rgbaColor=[0, 0, 0, 0.4])

try:
    while shared.is_running and trial_idx < len(TRIAL_PLAN):
        with shared.lock:
            v = shared.joint_vels.copy()
            g = shared.grip_pos
            event = shared.pinch_event
            p_val = shared.pinch_val
            p_state = shared.pinch_state_text
            a_reg = shared.active_region
            countdown = shared.release_countdown
            shared.pinch_event = None
            display_frame = shared.frame.copy() if shared.frame is not None else None

        # Motor Control
        for name, jid in j_ids.items():
            if name in v: p.setJointMotorControl2(robot, jid, p.VELOCITY_CONTROL, targetVelocity=v[name], force=MAX_FORCE)
        for jid, pos in locked_ids.items():
            p.setJointMotorControl2(robot, jid, p.POSITION_CONTROL, targetPosition=pos, force=MAX_FORCE)
        p.setJointMotorControl2(robot, j_ids["F_L"], p.POSITION_CONTROL, g, force=100)
        p.setJointMotorControl2(robot, j_ids["F_R"], p.POSITION_CONTROL, g, force=100)

        # Proximity/Halo Logic
        cp_raw, _ = p.getBasePositionAndOrientation(cube)
        cp = np.array(cp_raw)
        ee = np.array(p.getLinkState(robot, HAND_LINK)[0])
        graspable = np.linalg.norm(ee[:2] - cp[:2]) < 0.065 and abs(ee[2] - cp[2]) < 0.055
        # p.changeVisualShape(halo_body, -1, rgbaColor=[0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18])
        # p.resetBasePositionAndOrientation(halo_body, cp_raw, [0,0,0,1])


# --- Inside while loop (Cube Shadow Logic) ---
        cp_raw, _ = p.getBasePositionAndOrientation(cube)
        
        # Ray cast from the center of the cube down to the floor/tables
        # We start slightly inside the cube and go down
        ray_start = [cp_raw[0], cp_raw[1], cp_raw[2]]
        ray_end = [cp_raw[0], cp_raw[1], 0]
        
        # collisionFilterMask=1 hits only Group 1 (Tables/Floor)
        ray_res = p.rayTest(ray_start, ray_end, collisionFilterMask=1)

        if ray_res and ray_res[0][0] != -1:
            hit_z = ray_res[0][3][2]
            shadow_z = hit_z + 0.001
            
            # OPTIONAL: Visual height feedback
            # Calculate distance from cube to the surface
            dist_to_surf = cp_raw[2] - hit_z
            # Make shadow slightly larger/fainter as it gets higher (up to 20cm)
            scale_factor = min(1.0 + dist_to_surf * 2, 2.5) 
            alpha = max(0.1, 0.4 - dist_to_surf * 1.5)
            
            
            # Note: scaling a multi-body requires resetVisualShape in some versions, 
            # but usually, keeping a fixed size is clearer for teleoperation.
        else:
            shadow_z = 0.001

        p.resetBasePositionAndOrientation(shadow_body, [cp_raw[0], cp_raw[1], shadow_z], [0, 0, 0, 1])


# ---------- 1. Assisted Grasp Detection ----------
        cube_pos_raw, _ = p.getBasePositionAndOrientation(cube)
        cube_pos_np = np.array(cube_pos_raw)
        
        grasp_point = get_grasp_point()
        xy_dist = float(np.linalg.norm(grasp_point[:2] - cube_pos_np[:2]))
        z_dist  = float(abs(grasp_point[2] - cube_pos_np[2]))

        
        # Use your constants or hardcoded thresholds
        graspable = (xy_dist < 0.065) and (z_dist < 0.055)

        print("graspable: ", graspable)

        # --- Inside the loop ---
        current_halo_color = [0, 1, 0, 0.25] if graspable else [1, 0, 0, 0.18]

        # Only update color if it actually changed (State Check)
        if current_halo_color != last_halo_color:
            p.changeVisualShape(halo_body, -1, rgbaColor=current_halo_color)
            last_halo_color = current_halo_color

        # Only update position if the cube has actually moved (Efficiency Check)
        # Use a small threshold to avoid updating for microscopic physics jitter
        if np.linalg.norm(np.array(cp_raw) - np.array(last_halo_pos)) > 0.001:
            p.resetBasePositionAndOrientation(halo_body, cp_raw, [0, 0, 0, 1])
            last_halo_pos = cp_raw


        # --- Inside while loop (Visual Aids Logic) ---
        
        # 1. Project the gripper position onto the surface ONLY if cube is NOT attached
        if not cube_attached:
            gp = get_grasp_point()
            # Use the same rayTest logic to find the floor/table height
            ray_res = p.rayTest(gp, [gp[0], gp[1], 0], collisionFilterMask=1)
            
            if ray_res and ray_res[0][0] != -1:
                proj_z = ray_res[0][3][2] + 0.0015 # Slightly higher than floor
                # Update position
                p.resetBasePositionAndOrientation(gripper_proj_body, [gp[0], gp[1], proj_z], [0, 0, 0, 1])
                # Ensure it is visible (Alpha 0.4)
            else:
                p.resetBasePositionAndOrientation(gripper_proj_body, [0, 0, -1], [0, 0, 0, 1])
        else:
            # Hide the gripper projection when carrying the cube
            p.resetBasePositionAndOrientation(gripper_proj_body, [0, 0, -1], [0, 0, 0, 1])
            # Or make it transparent
            p.changeVisualShape(gripper_proj_body, -1, rgbaColor=[0, 0, 0, 0])

        # 2. Existing Shadow Logic (Project from Cube)
        # You can also make this only show when cube_attached is True if you prefer
        cp_raw, _ = p.getBasePositionAndOrientation(cube)
        ray_res_cube = p.rayTest(cp_raw, [cp_raw[0], cp_raw[1], 0], collisionFilterMask=1)
        
        if ray_res_cube and ray_res_cube[0][0] != -1:
            shadow_z = ray_res_cube[0][3][2] + 0.001
            p.resetBasePositionAndOrientation(shadow_body, [cp_raw[0], cp_raw[1], shadow_z], [0, 0, 0, 1])



        # ---------- 2. Trial Events (Pinch Logic) ----------
        if event == "CLOSE":
            grasp_attempts += 1
            if graspable and not cube_attached:
                cid = create_fixed_constraint_preserve_pose(robot, HAND_LINK, cube)
                cube_attached = True
                if time_to_grasp is None:
                    time_to_grasp = time.time() - trial_start

        elif event == "OPEN" and cube_attached:
                    # 1. Get the current position of the cube at the moment of release
                    cube_pos_raw, _ = p.getBasePositionAndOrientation(cube)
                    current_cube_pos = np.array(cube_pos_raw)
                    
                    # 2. Calculate 2D distance only (X and Y)
                    # This ignores how high the cube is held
                    dist_2d = np.linalg.norm(current_cube_pos[:2] - current_target_pos[:2])
                    is_near = dist_2d < SUCCESS_THRESH
                    
                    # 3. Release the physical bond
                    if cid: 
                        p.removeConstraint(cid)
                    cube_attached = False
                    
                    # 4. Wake up physics (so it actually falls if it wasn't a success)
                    p.resetBaseVelocity(cube, linearVelocity=[0, 0, -0.1])

                    if is_near:
                        # SUCCESS: Log data immediately based on the release position
                        err = float(dist_2d)
                        time_total = time.time() - trial_start
                        
                        with open(LOG_FILE, "a", newline="") as f:
                            csv.writer(f).writerow([
                                PARTICIPANT_ID, trial_idx, current_factor, current_difficulty,
                                round(current_D, 4), round(time_total, 3), 
                                round(time_to_grasp if time_to_grasp else time_total, 3),
                                round(err, 4), 1, grasp_attempts, drops,
                                *np.round(current_cube_pos, 4), *np.round(current_target_pos, 4)
                            ])
                        
                        # Advance Trial
                        trial_idx += 1
                        if trial_idx < len(TRIAL_PLAN):
                            set_trial_condition(trial_idx)
                            trial_start = time.time()
                            grasp_attempts, drops, time_to_grasp = 0, 0, None
                    else:
                        # MISS: It was dropped outside the 2D radius
                        drops += 1

        is_near = np.linalg.norm(current_cube_pos - current_target_pos) < SUCCESS_THRESH
        print(current_cube_pos, current_target_pos, is_near, np.linalg.norm(current_cube_pos - current_target_pos))                

        p.stepSimulation()
        if display_frame is not None:
            # Add HUD to frame
            h, w = display_frame.shape[:2]
            cv2.putText(display_frame, f"TRIAL: {trial_idx+1}/{len(TRIAL_PLAN)} | {current_factor} ({current_difficulty})", (10, h-80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(display_frame, f"STATE: {p_state} " + (f"DROP IN {countdown:.1f}s" if countdown else ""), (10, h-50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255) if countdown else (0,255,0), 2)
            cv2.putText(display_frame, f"Attempts: {grasp_attempts} | Drops: {drops}", (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.imshow("Vision Feedback", display_frame)

finally:
    shared.is_running = False
    vt.join()
    cv2.destroyAllWindows()
    p.disconnect()