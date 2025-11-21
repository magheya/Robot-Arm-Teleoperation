# Keyboard Control System Setup & Testing Guide

## Overview
This project creates a keyboard-to-Arduino communication system where arrow key presses are sent to an Arduino via serial communication.

## Hardware Requirements
- Arduino (Uno, Nano, ESP32, etc.)
- USB cable to connect Arduino to computer
- Built-in LED (or external LED on pin 13)

## Software Setup

### 1. Arduino Setup
1. Open Arduino IDE
2. Connect your Arduino via USB
3. Open `src/hw/keyboard_receiver.ino`
4. Select your board and port in Arduino IDE
5. Upload the sketch to your Arduino

### 2. Python Environment
The Python environment and packages are already configured. You have:
- Virtual environment at `.venv/`
- Required packages: `pynput`, `pyserial`

## Testing Steps

### Step 1: Test Arduino Connection
Run the test script to verify everything is connected:

```bash
python test_system.py
```

This will:
- List available serial ports
- Test communication with Arduino
- Send test commands and verify responses

### Step 2: Run the Keyboard Control
If the test passes, run the main application:

```bash
python src/controller/keyboard_control.py
```

### Step 3: Test Functionality
1. Make sure the terminal running the Python script is active
2. Press the **Left Arrow** key → Arduino LED should flash 2 times
3. Press the **Right Arrow** key → Arduino LED should flash 3 times
4. Check the Arduino Serial Monitor for command confirmations

## Troubleshooting

### Port Issues
If you get "Permission denied" or port not found:
1. Check available ports: `ls /dev/cu.*`
2. Update the port in both files:
   - `src/controller/keyboard_control.py` (line 6)
   - `test_system.py` (line 15)

### Permission Issues on macOS
If you get permission errors, you may need to:
1. Grant accessibility permissions to Terminal/VS Code
2. Go to System Preferences → Security & Privacy → Accessibility
3. Add your terminal application

### Arduino Not Responding
1. Check if Arduino IDE can connect to the board
2. Try pressing the reset button on Arduino
3. Verify the baud rate is 9600 in both Python and Arduino code
4. Check the Serial Monitor in Arduino IDE for debug messages

## How It Works

1. **Python Script** (`keyboard_control.py`):
   - Listens for keyboard events using `pynput`
   - Detects left/right arrow key presses
   - Sends 'L' or 'R' characters to Arduino via serial

2. **Arduino Code** (`keyboard_receiver.ino`):
   - Listens for serial data at 9600 baud
   - Processes 'L' and 'R' commands
   - Flashes LED and prints confirmation messages

## Stopping the Program
Press `Ctrl+C` in the terminal to stop the keyboard listener.

## Next Steps
- Add more key bindings (WASD, spacebar, etc.)
- Control servos, motors, or other actuators
- Add feedback from Arduino to Python (sensors, status)
- Create a GUI interface