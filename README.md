# Leap Motion Robot Arm Teleoperation

This project enables real-time control of a robotic arm using hand gestures via a Leap Motion controller. It uses a hybrid software stack with Python for gesture recognition and Arduino for low-level motor control.

## 🛠 Hardware Requirements

*   **Arduino Uno**
*   **Arduino Motor Shield R3** (Critical: The firmware is specifically designed for this shield)
*   **Leap Motion Controller** (V2)
*   **Robot Arm Components:**
    *   1x Bipolar Stepper Motor (Base Rotation)
    *   1x Servo Motor (Gripper)
    *   1x Servo Motor (Forearm Extension)
*   **External Power Supply** (9V - 12V DC) - *Required to drive the motors*

## 🔌 Wiring Guide

**⚠️ Important:** The firmware (`robot_arm_control.ino`) uses hardcoded pins specific to the Motor Shield R3. Connect your motors exactly as shown below:

| Robot Part | Motor Type | Connection on Motor Shield | Arduino Pin (Internal) |
| :--- | :--- | :--- | :--- |
| **Base Rotation** | Stepper Motor | **Channel A & B** Screw Terminals | 12, 13 (Dir) & 3, 11 (PWM) |
| **Gripper** | Servo | **Pin 5** (Orange/White pin) | Pin 5 |
| **Forearm** | Servo | **Pin 6** (Orange/White pin) | Pin 6 |
| **Power** | DC Input | **Vin / GND** Screw Terminals | - |

*Note: Do not power the motors solely from USB. Connect an external battery or power adapter to the Arduino's barrel jack or the shield's Vin terminals.*

## 💻 Software Setup

### 1. Arduino Firmware
1.  Open `src/robot_arm_control.ino` in the Arduino IDE.
2.  Ensure the standard `Servo` and `Stepper` libraries are installed.
3.  Upload the sketch to your Arduino.

### 2. Python Controller
1.  Ensure the Leap Motion V2 Desktop SDK is installed and the service is running.
2.  Install Python dependencies:
    ```bash
    pip install pyserial
    ```
3.  Run the main controller:
    ```bash
    python src/main.py
    ```

## ✋ Controls (One-Handed Mode)

The system currently uses a single hand to control all 3 axes of movement:

| Hand Gesture | Robot Action |
| :--- | :--- |
| **Move Hand Left / Right** | Rotates Base Left / Right |
| **Move Hand Forward / Backward** | Extends / Retracts Forearm |
| **Make a Fist (Grab)** | Closes Gripper |
| **Open Hand (Release)** | Opens Gripper |
| **Hold Hand in Center** | Stops all movement (Deadzone) |

## 📂 Project Structure

*   **`src/main.py`**: The main application. Connects to Leap Motion and Arduino, and orchestrates the control loop.
*   **`src/robot_arm_control.ino`**: Advanced Arduino firmware. Supports non-blocking stepper movement and servo control via text protocol.
*   **`src/simple_recognizer.py`**: Logic engine. Converts raw hand coordinates (X, Z, Grip Strength) into abstract robot commands.
*   **`src/commands.py`**: Protocol helper. Formats commands into the text packets expected by the Arduino (e.g., `"STARTSTEP 10"`).
*   **`src/utils.py`**: Shared definitions and Enums.