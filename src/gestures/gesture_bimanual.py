import cv2
import mediapipe as mp
import numpy as np

# =============================
# MediaPipe setup
# =============================
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,   # BIMANUAL
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)

# =============================
# Webcam
# =============================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

# Store previous centers per handedness
prev_centers = {"Left": None, "Right": None}

# Pinch thresholds
PINCH_CLOSE = 0.04
PINCH_OPEN  = 0.07
pinch_state = "OPEN"  # OPEN or CLOSED (edge-triggered)

# Deadzone (bigger + transparent)
DEADZONE_TOP = 0.42
DEADZONE_BOTTOM = 0.58

print("""
BIMANUAL 4-REGION CONTROL (HAND-LOCKED)

| BASE | SHOULDER | WRIST | ELBOW |

Left hand controls: BASE + SHOULDER only
Right hand controls: WRIST + ELBOW only

Pinch (either hand): gripper OPEN/CLOSE (stable state)
ESC to quit
""")

def region_from_cx(cx: float) -> str:
    if cx < 0.25:
        return "BASE"
    elif cx < 0.50:
        return "SHOULDER"
    elif cx < 0.75:
        return "WRIST"
    else:
        return "ELBOW"

# def allowed(handedness: str, region: str) -> bool:
#     # Left hand allowed only in left-half regions
#     if handedness == "Left":
#         return region in ["BASE", "SHOULDER"]
#     # Right hand allowed only in right-half regions
#     if handedness == "Right":
#         return region in ["WRIST", "ELBOW"]
#     return False

def allowed(handedness: str, region: str) -> bool:
    return True

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    # -----------------------------
    # Draw regions (vertical splits)
    # -----------------------------
    splits = [0.25, 0.50, 0.75]
    labels = ["BASE", "SHOULDER", "WRIST", "ELBOW"]

    for s in splits:
        x = int(w * s)
        cv2.line(frame, (x, 0), (x, h), (255, 255, 255), 2)

    for i, label in enumerate(labels):
        cv2.putText(frame, label,
                    (int(w*(i*0.25 + 0.125)) - 45, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)

    # -----------------------------
    # Draw deadzone band (transparent)
    # -----------------------------
    top_y = int(h * DEADZONE_TOP)
    bot_y = int(h * DEADZONE_BOTTOM)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, top_y), (w, bot_y), (60, 60, 60), -1)
    frame = cv2.addWeighted(overlay, 0.22, frame, 0.78, 0)

    cv2.line(frame, (0, top_y), (w, top_y), (0, 255, 0), 2)
    cv2.line(frame, (0, bot_y), (w, bot_y), (0, 255, 0), 2)

    # -----------------------------
    # MediaPipe detection
    # -----------------------------
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    # Aggregated control signals (for your next step into PyBullet)
    base_cmd = 0
    shoulder_cmd = 0
    wrist_cmd = 0
    elbow_cmd = 0

    pinch_val_debug = None

    if results.multi_hand_landmarks and results.multi_handedness:
        for hand_landmarks, handedness_info in zip(results.multi_hand_landmarks,
                                                   results.multi_handedness):

            handedness = handedness_info.classification[0].label  # "Left" or "Right"

            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            xs = [lm.x for lm in hand_landmarks.landmark]
            ys = [lm.y for lm in hand_landmarks.landmark]
            cx, cy = float(np.mean(xs)), float(np.mean(ys))

            px, py = int(cx * w), int(cy * h)
            cv2.circle(frame, (px, py), 6, (0, 255, 0), -1)

            region = region_from_cx(cx)

            # Direction from deadzone (thresholded)
            direction = 0
            if cy < DEADZONE_TOP:
                direction = +1
            elif cy > DEADZONE_BOTTOM:
                direction = -1

            # Apply ONLY if correct hand is in allowed region
            if allowed(handedness, region):
                if region == "BASE":
                    base_cmd = direction
                elif region == "SHOULDER":
                    shoulder_cmd = direction
                elif region == "WRIST":
                    wrist_cmd = direction
                elif region == "ELBOW":
                    elbow_cmd = direction

                region_text = f"{handedness}: {region} (ACTIVE)"
                color = (0, 255, 0)
            else:
                region_text = f"{handedness}: {region} (LOCKED)"
                color = (0, 0, 255)

            cv2.putText(frame, region_text,
                        (px - 80, py - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        color, 2)

            # Pinch (either hand) -> stable OPEN/CLOSED
            thumb = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
            index = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            pinch = float(np.linalg.norm(
                np.array([thumb.x, thumb.y]) - np.array([index.x, index.y])
            ))
            pinch_val_debug = pinch  # show last one (fine for debugging)

            if pinch_state == "OPEN" and pinch < PINCH_CLOSE:
                pinch_state = "CLOSED"
            elif pinch_state == "CLOSED" and pinch > PINCH_OPEN:
                pinch_state = "OPEN"

    # -----------------------------
    # HUD
    # -----------------------------
    hud = (f"CMD  BASE={base_cmd:+d}  SHO={shoulder_cmd:+d}  "
           f"WR={wrist_cmd:+d}  EL={elbow_cmd:+d}  |  GRIP={pinch_state}")
    cv2.putText(frame, hud, (10, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if pinch_val_debug is not None:
        cv2.putText(frame, f"PINCH={pinch_val_debug:.3f}", (10, h - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Bimanual 4-Region (Left/Right Hand Locked)", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()