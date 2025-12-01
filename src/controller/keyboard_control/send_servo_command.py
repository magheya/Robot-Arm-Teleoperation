import serial
import time

SERIAL_PORT = "/dev/cu.usbmodem1201"  

BAUD_RATE = 115200

def send_servo_command(ser, servo_id, angle):
    """Send a single servo command as '<id> <angle>\\n'"""
    packet = f"{servo_id} {angle}\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")

def send_stepper_command(ser, steps):
    """Send a single stepper command as 'STEP <steps>\\n'"""
    packet = f"STEP {steps}\n"
    ser.write(packet.encode('utf-8'))
    print(f"Sent: {packet.strip()}")    

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(1)  
        send_servo_command(ser, 0, 180) # servo id, angle
        time.sleep(1)  
        send_stepper_command(ser, 50) # servo id, angle
        send_stepper_command(ser, -50) 

    except serial.SerialException as e:
        print("Error opening serial port:", e)

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()