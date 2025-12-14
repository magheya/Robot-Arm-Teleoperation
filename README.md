# Webcam-Controlled Robot Arm Teleoperation

This project enables real-time control of a multi-axis robotic arm using hand gestures captured via a standard webcam. It uses MediaPipe for advanced hand tracking and gesture recognition in Python, communicating with an Arduino for precise, low-level motor control.

The system supports two primary modes:
*   **Bimanual Mode:** Uses two hands for intuitive, concurrent control over aiming and action.
*   **Unimanual Mode:** Uses a single hand to control all axes, including depth-based control for arm extension.

## 🛠 Hardware Requirements

*   **Arduino Uno**
*   **Arduino Motor Shield R3** (Critical: The firmware is specifically designed for this shield)
*   **Webcam** (for hand tracking)
*   **Robot Arm Components:**
    *   1x Bipolar Stepper Motor (Base Rotation)
    *   5x Servo Motors (Shoulder, Elbow, Wrist, and a two-servo Gripper)
*   **External Power Supply** (9V - 12V DC) - *Required to drive the motors*

## 🔌 Wiring Guide

**⚠️ Important:** The firmware (`src/robot_arm_control.ino`) uses hardcoded pins. The stepper motor connects to the Motor Shield's screw terminals, while the servos connect to the digital and analog pins.

| Robot Part | Motor Type | Connection on Arduino/Shield | Firmware Pin Name |
| :--- | :--- | :--- | :--- |
| **Base Rotation** | Stepper Motor | **Channel A & B** Screw Terminals | `myStepper` (Pins 12, 13, 3, 11) |
| **Shoulder** | Servo | **Pin 10** | `SHOULDER_PIN` |
| **Elbow** | Servo | **Pin 6** | `ELBOW_PIN` |
| **Wrist** | Servo | **Pin 5** | `WRIST_PIN` |
| **Left Gripper** | Servo | **Pin A4** | `GRIP_L_PIN` |
| **Right Gripper** | Servo | **Pin A5** | `GRIP_R_PIN` |
| **Power** | DC Input | **Vin / GND** Screw Terminals | - |

*Note: Do not power the motors solely from USB. Connect an external battery or power adapter to the Arduino's barrel jack or the shield's Vin terminals.*

## 💻 Software Setup

### 1. Arduino Firmware
1.  Open `src/robot_arm_control.ino` in the Arduino IDE.
2.  Ensure the standard `Servo` and `Stepper` libraries are installed.
3.  Upload the sketch to your Arduino Uno.

### 2. Python Controller
1.  Install the required Python dependencies. It is recommended to use a virtual environment.
    ```bash
    pip install opencv-python mediapipe pyserial
    ```
2.  Identify the COM port your Arduino is connected to (e.g., `COM3` on Windows).
3.  Update the `SERIAL_PORT` variable in the main script you intend to run (e.g., `src/cv_main.py`).
4.  Run the main controller:
    ```bash
    python src/cv_main.py
    ```

## ✋ Control Modes

The main application (`cv_main.py`) starts in Bimanual mode. You can switch modes by pressing keys in the OpenCV window.

*   Press **`b`** for **Bimanual Mode**.
*   Press **`u`** for **Unimanual Mode**.
*   Press **`q`** to quit.

### Bimanual Controls (Two Hands)

This mode offers the most intuitive control by splitting tasks between hands.

| Hand | Gesture | Robot Action |
| :--- | :--- | :--- |
| **Right Hand** | Move Left / Right | Rotates Base Left / Right |
| | Move Up / Down | Raises / Lowers Shoulder |
| **Left Hand** | Move Up / Down | Extends / Retracts Elbow (Reach) |
| | Make a Fist | Closes Gripper |
| | Open Hand | Opens Gripper |

### Unimanual Controls (One Hand)

This mode maps all controls to a single hand (the right hand).

| Gesture | Robot Action |
| :--- | :--- |
| **Move Hand Left / Right** | Rotates Base Left / Right |
| **Move Hand Up / Down** | Raises / Lowers Shoulder |
| **Move Hand Closer/Farther** | Extends / Retracts Elbow (Depth) |
| **Make a Fist** | Closes Gripper |
| **Open Hand** | Opens Gripper |
| **Make a Peace Sign** | Toggles a "Motor Lock" to freeze the arm in place. |

## 📂 Project Structure

*   **`src/cv_main.py`**: The main application for webcam control. Connects to the camera and Arduino, and allows switching between control modes.
*   **`src/robot_arm_control.ino`**: Arduino firmware that listens for serial commands (e.g., `"e 90"`, `"S"`) to control the stepper and five servos.
*   **`src/cv_recognizer.py`**: The core logic engine. Converts MediaPipe hand landmark data into robot commands for both unimanual and bimanual modes.
*   **`src/commands.py`**: Helper script that formats high-level actions into the specific serial command strings expected by the Arduino.
*   **`src/utils.py`**: Contains shared enumerations for gesture types (e.g., `MOVE_LEFT`, `CLOSED_HAND`).
*   **`src/main_unimanual.py` / `main_bimanual.py`**: Standalone scripts for testing each control mode individually.
*   **`src/test_*.py`**: A suite of scripts for testing and calibrating individual joints (elbow, wrist, gripper, etc.).
*   **`src/main.py`**: (Legacy) The original implementation using a Leap Motion controller.