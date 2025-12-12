# Servo IDs
SHOULDER_ID = 0
ELBOW_ID = 1
WRIST_ID = 2
GRIPPER_LEFT_ID = 3
GRIPPER_RIGHT_ID = 4

# --- Helper Functions ---

def _send_servo_command(ser, servo_id, angle):
    """Helper to format and send a direct servo angle command."""
    packet = f"{servo_id} {angle}\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")

def _send_stepper_startstep(ser, speed):
    """Helper to format and send a continuous stepper command."""
    packet = f"STARTSTEP {speed}\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")

def _send_stepper_stopstep(ser):
    """Helper to format and send a stepper stop command."""
    packet = "STOPSTEP\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")

def _send_servo_increment_start(ser, servo_id, amount):
    """Helper to start continuous servo incrementing."""
    packet = f"STARTINC {servo_id} {amount}\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")

def _send_servo_increment_stop(ser):
    """Helper to stop continuous servo incrementing."""
    packet = "STOPINC\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")

# --- Public Command Functions ---

def send_close_grip(ser):
    # Assumes closing is one servo to 180 and the other to 0
    _send_servo_command(ser, GRIPPER_LEFT_ID, 180)
    _send_servo_command(ser, GRIPPER_RIGHT_ID, 0)

def send_open_grip(ser):
    # Assumes opening is one servo to 0 and the other to 180
    _send_servo_command(ser, GRIPPER_LEFT_ID, 0)
    _send_servo_command(ser, GRIPPER_RIGHT_ID, 180)

def send_rotate_left(ser):
    _send_stepper_startstep(ser, 1)

def send_rotate_right(ser):
    _send_stepper_startstep(ser, -1)

def send_rotate_stop(ser):
    _send_stepper_stopstep(ser)

def send_move_up(ser):
    # Use shoulder servo for vertical movement
    _send_servo_increment_start(ser, SHOULDER_ID, 1)

def send_move_down(ser):
    # Use shoulder servo for vertical movement
    _send_servo_increment_start(ser, SHOULDER_ID, -1)

def send_stop_move_vertical(ser):
    _send_servo_increment_stop(ser)