from pynput import keyboard
import serial
import time

# Connect to Arduino
arduino = serial.Serial('/dev/cu.usbmodem2101', 9600)
time.sleep(2)  # wait for Arduino to reset

def on_press(key):
    try:
        if key == keyboard.Key.up:
            arduino.write(b'u')
            print("UP")
        elif key == keyboard.Key.down:
            arduino.write(b'd')
            print("DOWN")
    except Exception as e:
        print(f"Error: {e}")

def on_release(key):
    # Exit on ESC key
    if key == keyboard.Key.esc:
        print("ESC pressed - Stopping gripper control...")
        return False

print("🤖 Gripper Control Ready!")
print("Controls:")
print("  ⬆️ UP arrow   = Move gripper up")
print("  ⬇️ DOWN arrow = Move gripper down")
print("  🚪 ESC key    = Exit program")
print("\nPress keys to control the gripper...")

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

arduino.close()
print("Gripper control stopped.")
