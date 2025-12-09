


LOWER_GRIP_ID = 0
UPPER_GRIP_ID = 1


def _send_servo_command(ser, servo_id, angle):
    packet = f"{servo_id} {angle}\n"
    print("packet: ", packet.encode('utf-8'))
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")


def _send_servo_startinc(ser, servo_id, speed):
    packet = f"STARTINC {servo_id} {speed}\n"
    print("packet: ", packet.encode('utf-8'))
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")


def _send_servo_stopinc(ser, servo_id):
    packet = f"STOPINC {servo_id}\n"
    print("packet: ", packet.encode('utf-8'))
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")       

def _send_stepper_startstep(ser, speed):
    packet = f"STARTSTEP {speed}\n"
    print("packet: ", packet.encode('utf-8'))
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")         

def _send_stepper_stopstep(ser):
    packet = f"STOPSTEP\n"
    print("packet: ", packet.encode('utf-8'))
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")      

def _send_stepper_command(ser, steps):
    packet = f"STEP {steps}\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")  


def send_close_grip(ser):
    _send_servo_command(ser, LOWER_GRIP_ID, 180)
    _send_servo_command(ser, UPPER_GRIP_ID, 0)

def send_open_grip(ser):
    _send_servo_command(ser, LOWER_GRIP_ID, 0)
    _send_servo_command(ser, UPPER_GRIP_ID, 180)

def send_move_up(ser):
    _send_servo_startinc(ser, 3, -1)

def send_move_down(ser):
    _send_servo_startinc(ser, 3, 1)

def send_stop_vertical(ser):
    _send_servo_stopinc(ser, 3)

def send_rotate_left(ser):
    _send_stepper_startstep(ser, 1)

def send_rotate_right(ser):
    _send_stepper_startstep(ser, -1)

def send_rotate_stop(ser):
    _send_stepper_stopstep(ser)                                    

