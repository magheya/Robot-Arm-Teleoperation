import serial
import time
from pynput import keyboard

# CHANGE THIS PORT depending on your Mac
# Check with: ls /dev/tty.*
SERIAL_PORT = "/dev/cu.usbmodem2101"
BAUD_RATE = 9600  # Match Arduino baud rate

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)  # wait for Arduino to reset


def on_press(key):
    try:
        # Rotation controls
        if key == keyboard.Key.left:
            print("LEFT ROTATION")
            arduino.write(b'L')

        elif key == keyboard.Key.right:
            print("RIGHT ROTATION")
            arduino.write(b'R')
            
        # Gripper controls
        elif key == keyboard.Key.up:
            print("OPEN GRIPPERS")
            arduino.write(b'O')
            
        elif key == keyboard.Key.down:
            print("CLOSE GRIPPERS")
            arduino.write(b'C')
            
        # Additional controls
        elif key == keyboard.Key.space:
            print("HOME POSITION")
            arduino.write(b'H')
            
        elif key.char == 's':
            print("STATUS")
            arduino.write(b'S')

    except AttributeError:
        # Special keys (like arrows) don't have char attribute
        pass
    except Exception as e:
        print("Error:", e)


def on_release(key):
    if key == keyboard.Key.esc:
        print("Exiting...")
        return False


def main():
    print("🤖 Robot Arm Keyboard Control")
    print("=" * 40)
    print("Controls:")
    print("  ← → arrows  = Rotate left/right")
    print("  ↑ ↓ arrows  = Open/close grippers")
    print("  SPACE       = Home position")
    print("  S           = Status")
    print("  ESC         = Exit")
    print("\nReady! Press keys to control the robot arm.")
    
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()


if __name__ == "__main__":
    main()
