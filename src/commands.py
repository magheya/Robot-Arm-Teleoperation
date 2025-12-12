# ID 0: Shoulder, ID 1: Elbow, ID 2: Wrist
# ID 3: Gripper Left, ID 4: Gripper Right
GRIPPER_LEFT_ID = 3
GRIPPER_RIGHT_ID = 4


def _send_servo_command(ser, servo_id, angle):
    """Helper to format and send a servo command."""
    packet = f"{servo_id} {angle}\n"
    print("packet: ", packet.encode('utf-8'))
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


def _send_stepper_command(ser, steps):
    """Helper to format and send a manual stepper command."""
    packet = f"STEP {steps}\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")


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