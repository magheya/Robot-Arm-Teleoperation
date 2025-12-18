import time

def _send_command(ser, cmd_str):
    """Encodes and sends a string command to Arduino."""
    if ser:
        full_cmd = f"{cmd_str}\n"
        ser.write(full_cmd.encode('utf-8'))
        # Optional: slight delay to not flood Arduino buffer if it's slow
        # time.sleep(0.01) 

# --- 1. BASE (Stepper) ---
# Arduino expects: 'F' (Forward), 'B' (Backward), 'S' (Stop)
def send_rotate_left(ser):
    _send_command(ser, "F") # Adjust direction if inverted

def send_rotate_right(ser):
    _send_command(ser, "B") # Adjust direction if inverted

def send_rotate_stop(ser):
    _send_command(ser, "S")

# --- 2. ARM JOINTS ---
# Arduino expects: 's <angle>', 'e <angle>', 'w <angle>'

def send_shoulder(ser, angle):
    # Map Python angle to Robot Limits if necessary
    # Example: limit shoulder between 40 and 160
    _send_command(ser, f"s {angle}")

def send_elbow(ser, angle):
    _send_command(ser, f"e {angle}")

def send_wrist(ser, angle):
    # FULL RANGE (0 to 180) - No limits for testing
    _send_command(ser, f"w {angle}")


# --- 3. GRIPPERS ---
# Arduino expects: 'l <angle>' and 'r <angle>'
def send_grip(ser, is_closed):
    if is_closed:
        # FULL CLOSE (Try 0 first. If inverted, swap with 180)
        _send_command(ser, "l 0") 
        _send_command(ser, "r 0")
    else:
        # FULL OPEN (Removed the 90 limit, now sends 180)
        _send_command(ser, "l 180")
        _send_command(ser, "r 180")