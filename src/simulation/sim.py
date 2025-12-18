import pybullet as p
import pybullet_data
import time
import numpy as np
import csv
import os

# =============================
# Setup
# =============================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)

p.resetDebugVisualizerCamera(
    cameraDistance=1.4,
    cameraYaw=0,
    cameraPitch=-30,
    cameraTargetPosition=[0.55, 0, 0.30]
)

p.loadURDF("plane.urdf")

# =============================
# Load Panda
# =============================
robot = p.loadURDF(
    "franka_panda/panda.urdf",
    basePosition=[0, 0, 0],
    useFixedBase=True
)

# =============================
# Joint info + EE link
# =============================
joint_name_to_id = {}
HAND_LINK = None

for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    joint = info[1].decode()
    link = info[12].decode()
    joint_name_to_id[joint] = i
    if link == "panda_link8":
        HAND_LINK = i

if HAND_LINK is None:
    raise RuntimeError("Could not find panda_link8")

# =============================
# Joint mapping (add active wrist pitch)
# =============================
BASE_J = joint_name_to_id["panda_joint1"]
SHOULDER_J = joint_name_to_id["panda_joint2"]
ELBOW_J = joint_name_to_id["panda_joint4"]
WRIST_J = joint_name_to_id["panda_joint6"]  # ACTIVE wrist pitch (NEW)
FINGER_L = joint_name_to_id["panda_finger_joint1"]
FINGER_R = joint_name_to_id["panda_finger_joint2"]

# =============================
# Neutral posture
# =============================
neutral = {
    "panda_joint1": 0.0,
    "panda_joint2": -0.4,
    "panda_joint3": 0.0,
    "panda_joint4": -2.0,
    "panda_joint5": 0.0,
    "panda_joint6": 1.7,  # wrist pitch neutral
    "panda_joint7": 0.8,
}
for j, v in neutral.items():
    p.resetJointState(robot, joint_name_to_id[j], v)

# =============================
# Tables (LEFT / RIGHT, different heights)
# =============================
PICK_Z = 0.30     # higher
PLACE_Z = 0.22    # lower

PICK_Y = -0.35    # LEFT
PLACE_Y = 0.35    # RIGHT

TABLE_X = 0.55    # same distance from robot

def create_table(x, y, z):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.22, 0.22, 0.02])
    vis = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.22, 0.22, 0.02],
        rgbaColor=[0.7, 0.7, 0.7, 1]
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[x, y, z - 0.02]
    )

create_table(TABLE_X, PICK_Y, PICK_Z)    # pick (left, higher)
create_table(TABLE_X, PLACE_Y, PLACE_Z)  # place (right, lower)

# =============================
# Cube (LEFT, higher)
# =============================
CUBE_SIZE = 0.04
cube_start_pos = [TABLE_X, PICK_Y, PICK_Z + CUBE_SIZE / 2]

cube = p.loadURDF(
    "cube_small.urdf",
    basePosition=cube_start_pos,
    globalScaling=CUBE_SIZE / 0.05
)

# =============================
# Target (RIGHT, lower)
# =============================
TARGET_POS = np.array([TABLE_X, PLACE_Y, PLACE_Z + CUBE_SIZE / 2])

target_vis = p.createVisualShape(
    p.GEOM_CYLINDER,
    radius=0.05,
    length=0.002,
    rgbaColor=[1, 0, 0, 0.7]
)
p.createMultiBody(0, baseVisualShapeIndex=target_vis, basePosition=TARGET_POS)

# =============================
# Control + experiment state
# =============================
base = neutral["panda_joint1"]
shoulder = neutral["panda_joint2"]
elbow = neutral["panda_joint4"]
wrist = neutral["panda_joint6"]   # NEW
grip = 0.04

cube_attached, grasp_cid = False, None

PARTICIPANT_ID = "P01"
LOG_FILE = f"results_{PARTICIPANT_ID}.csv"
trial_index = 0
trial_start_time = time.time()
regrasp_count = 0
SUCCESS_THRESH = 0.03

# Wrist limits (kept conservative)
WRIST_MIN, WRIST_MAX = 0.2, 2.6

if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.writer(f).writerow(
            ["participant", "trial", "time", "placement_error", "success", "regrasp_count"]
        )

print("""
LEFT  table (higher): PICK
RIGHT table (lower): PLACE

Controls:
A/D : base rotate
W/S : shoulder
Q/E : elbow
R/F : wrist pitch (NEW)
O/C : gripper
ESC : quit
""")

# =============================
# Main loop
# =============================
while True:
    keys = p.getKeyboardEvents()
    if 27 in keys:
        break

    if ord('a') in keys: base += 0.02
    if ord('d') in keys: base -= 0.02
    if ord('w') in keys: shoulder += 0.02
    if ord('s') in keys: shoulder -= 0.02
    if ord('q') in keys: elbow += 0.02
    if ord('e') in keys: elbow -= 0.02

    # Wrist pitch control (NEW)
    if ord('r') in keys: wrist += 0.02
    if ord('f') in keys: wrist -= 0.02

    if ord('o') in keys: grip += 0.001
    if ord('c') in keys: grip -= 0.001

    base = np.clip(base, -2.8, 2.8)
    shoulder = np.clip(shoulder, -1.5, 1.2)
    elbow = np.clip(elbow, -3.0, 0.0)
    wrist = np.clip(wrist, WRIST_MIN, WRIST_MAX)
    grip = np.clip(grip, 0.0, 0.04)

    # Controlled joints
    p.setJointMotorControl2(robot, BASE_J, p.POSITION_CONTROL, base, force=200)
    p.setJointMotorControl2(robot, SHOULDER_J, p.POSITION_CONTROL, shoulder, force=200)
    p.setJointMotorControl2(robot, ELBOW_J, p.POSITION_CONTROL, elbow, force=200)
    p.setJointMotorControl2(robot, WRIST_J, p.POSITION_CONTROL, wrist, force=120)  # NEW
    p.setJointMotorControl2(robot, FINGER_L, p.POSITION_CONTROL, grip, force=100)
    p.setJointMotorControl2(robot, FINGER_R, p.POSITION_CONTROL, grip, force=100)

    # Lock other joints (keep consistent)
    for j, v in neutral.items():
        if j in ("panda_joint1", "panda_joint2", "panda_joint4", "panda_joint6"):
            continue
        p.setJointMotorControl2(robot, joint_name_to_id[j], p.POSITION_CONTROL, v, force=200)

    # Snap grasp
    ee_pos = p.getLinkState(robot, HAND_LINK)[0]
    cube_pos, _ = p.getBasePositionAndOrientation(cube)

    if grip < 0.01 and not cube_attached:
        if np.linalg.norm(np.array(ee_pos) - np.array(cube_pos)) < 0.05:
            grasp_cid = p.createConstraint(
                robot, HAND_LINK,
                cube, -1,
                p.JOINT_FIXED,
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            )
            cube_attached = True
            regrasp_count += 1

    if grip > 0.02 and cube_attached:
        p.removeConstraint(grasp_cid)
        cube_attached = False

        cube_pos, _ = p.getBasePositionAndOrientation(cube)
        error = np.linalg.norm(np.array(cube_pos) - TARGET_POS)
        duration = time.time() - trial_start_time

        with open(LOG_FILE, "a", newline="") as f:
            csv.writer(f).writerow(
                [PARTICIPANT_ID, trial_index, round(duration, 3), round(error, 4),
                 int(error < SUCCESS_THRESH), regrasp_count]
            )

        trial_index += 1
        trial_start_time = time.time()
        regrasp_count = 0

        # Reset cube
        p.resetBasePositionAndOrientation(cube, cube_start_pos, [0, 0, 0, 1])

        # Reset wrist to neutral each trial (keeps trials comparable)
        wrist = neutral["panda_joint6"]

    p.stepSimulation()
    time.sleep(1 / 240)

p.disconnect()
