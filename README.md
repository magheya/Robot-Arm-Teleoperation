# Robot Arm Teleoperation via Webcam

This project allows for real-time teleoperation of a simulated 6-DOF robot arm using a standard webcam. It leverages the MediaPipe library for robust hand tracking and the PyBullet physics engine for simulation.


*(Suggestion: Record a GIF of the simulation in action and replace the link above!)*

---

## Features

- **Real-time Control:** Control the robot arm in real-time using hand movements.
- **Dual Control Modes:** Switch between two-handed (Bimanual) and single-handed (Unimanual) control.
- **Intuitive Zoned Controls:** The screen is split into dedicated zones for left and right hands to prevent accidental inputs.
- **Physics Simulation:** Utilizes PyBullet to simulate realistic movements, gravity, and object interactions.
- **Object Manipulation:** Pick up and place objects in the simulated environment.
- **Visual Feedback:** On-screen markers and text provide clear feedback on control zones, modes, and arm status.
- **Live Reset:** Reset the simulation to its initial state at any time with a keypress.

---

## Control Scheme

The control mapping is designed to be as intuitive as possible.

### Bimanual Mode (Two Hands)

This mode offers the most granular control by dedicating each hand to specific functions.

| Hand        | Movement                 | Robot Control          |
|-------------|--------------------------|------------------------|
| **Right Hand** | Up / Down (Y-axis)       | Shoulder Angle         |
|             | Left / Right (X-axis)    | Base Rotation          |
|             | Forward / Back (Z-axis)  | Elbow Angle            |
| **Left Hand**  | Up / Down (Y-axis)       | Wrist Angle            |
|             | Gesture (Open/Close)     | Gripper (Open/Close)   |

### Unimanual Mode (One Hand)

This mode allows for full control using only the right hand.

| Hand        | Movement                 | Robot Control          |
|-------------|--------------------------|------------------------|
| **Right Hand** | Up / Down (Y-axis)       | Shoulder Angle         |
|             | Left / Right (X-axis)    | Base Rotation          |
|             | Forward / Back (Z-axis)  | Elbow Angle            |
|             | Gesture (Open/Close)     | Gripper (Open/Close)   |

---

## Setup and Installation

### Prerequisites
- Python 3.8+

### Installation Steps

1.  **Clone the repository:**
    ```bash
    git clone <your-repository-url>
    cd Robot-Arm-Teleoperation
    ```

2.  **Create and activate a virtual environment (recommended):**
    ```bash
    # For Windows
    python -m venv venv
    .\venv\Scripts\activate

    # For macOS/Linux
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install the required packages:**
    Create a file named `requirements.txt` in the root directory with the following content:
    ```
    opencv-python
    mediapipe
    pybullet
    ```
    Then, run the installation command:
    ```bash
    pip install -r requirements.txt
    ```

---

## How to Run

Execute the main script from the root directory:

```bash
py src/cv_main_sim.py
```

A window will appear showing your webcam feed and the PyBullet simulation.

### Keyboard Controls

-   `b`: Switch to **Bimanual** Mode.
-   `u`: Switch to **Unimanual** Mode.
-   `r`: **Reset** the simulation state.
-   `q`: **Quit** the application.

---

## Project Structure

```
Robot-Arm-Teleoperation/
│
├── src/
│   ├── cv_main_sim.py      # Main application: camera, UI, main loop
│   ├── cv_recognizer.py    # Translates hand landmarks into robot commands
│   ├── simulator.py        # Manages the PyBullet simulation and physics
│   ├── utils.py            # Shared utilities (e.g., GestureType enum)
│   └── sim_files/
│       └── robot_arm.urdf  # The URDF model of the robot arm
│
└── README.md               # This file
```

---

## Future Development Ideas

- **Physical Robot Integration:** Adapt the controller to send commands to a real robot arm via serial or a network protocol.
- **Unity/Unreal Engine Integration:** Replace PyBullet with a high-fidelity game engine for enhanced graphics and simulation capabilities.
- **Complex Tasks:** Create scenarios with multiple objects and specific goals (e.g., stacking blocks, sorting colors).
- **Advanced Gesture Recognition:** Train a custom machine learning model to recognize a wider variety of gestures for more complex commands.