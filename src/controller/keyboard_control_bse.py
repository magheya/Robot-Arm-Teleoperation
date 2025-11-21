import serial
import time
from pynput import keyboard

# CHANGE THIS PORT depending on your Mac
# Check with: ls /dev/tty.*
SERIAL_PORT = "/dev/cu.usbmodem2101"
BAUD_RATE = 115200

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)  # wait for Arduino to reset


def on_press(key):
    try:
        if key == keyboard.Key.left:
            print("LEFT")
            arduino.write(b'L')

        elif key == keyboard.Key.right:
            print("RIGHT")
            arduino.write(b'R')

    except Exception as e:
        print("Error:", e)


def on_release(key):
    if key == keyboard.Key.esc:
        print("Exiting…")
        return False


def main():
    print("Ready. Use ← → arrows to move. Press ESC to quit.")
    
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


if __name__ == "__main__":
    main()
