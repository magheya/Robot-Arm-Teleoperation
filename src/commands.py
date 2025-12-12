import time

# --- SERVO ID MAPPING (Matches robot_arm_control.ino) ---
SHOULDER_ID = 0  # Pin 5
ELBOW_ID    = 1  # Pin 10
WRIST_ID    = 2  # Pin 6
GRIP_L_ID   = 3  # Pin A4
GRIP_R_ID   = 4  # Pin A5

def _send_packet(ser, packet):
    msg = f"{packet}\n"
    ser.write(msg.encode('utf-8'))

# --- 1. BASE (Stepper) ---
def send_rotate_left(ser):
    _send_packet(ser, "STARTSTEP -1")

def send_rotate_right(ser):
    _send_packet(ser, "STARTSTEP 1")

def send_rotate_stop(ser):
    _send_packet(ser, "STOPSTEP")

# --- 2. SHOULDER (Up/Down) ---
def send_shoulder(ser, angle):
    # Angle 0-180
    _send_packet(ser, f"{SHOULDER_ID} {angle}")

# --- 3. ELBOW (Extend/Retract) ---
def send_elbow(ser, angle):
    _send_packet(ser, f"{ELBOW_ID} {angle}")

# --- 4. WRIST (Rotate) ---
def send_wrist(ser, angle):
    _send_packet(ser, f"{WRIST_ID} {angle}")

# --- 5. GRIPPER (Open/Close) ---
def send_grip(ser, is_closed):
    angle = 160 if is_closed else 90 # 160=Closed, 90=Open
    _send_packet(ser, f"{GRIP_L_ID} {angle}")
    _send_packet(ser, f"{GRIP_R_ID} {angle}")
