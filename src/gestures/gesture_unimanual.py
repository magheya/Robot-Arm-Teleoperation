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
    max_num_hands=1,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)

# =============================
# Webcam
# =============================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

prev_center = None

# =============================
# Wrist + pinch parameters
# =============================
PINCH_CLOSE_THRESH = 0.04
PINCH_OPEN_THRESH = 0.07

print("""
4-REGION GESTURE CONTROL (SIMPLIFIED)

| BASE | SHOULDER | WRIST | ELBOW |

Hand motion:
- BASE     : left / right
- SHOULDER : up / down
- WRIST    : up / down
- ELBOW    : up / down

Pinch (any region):
- OPEN / CLOSE gripper

ESC to quit
""")

# =============================
# Main loop
# =============================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    # -----------------------------
    # Draw region separators
    # -----------------------------
    r1 = int(w * 0.25)
    r2 = int(w * 0.50)
    r3 = int(w * 0.75)

    cv2.line(frame, (r1, 0), (r1, h), (255, 255, 255), 2)
    cv2.line(frame, (r2, 0), (r2, h), (255, 255, 255), 2)
    cv2.line(frame, (r3, 0), (r3, h), (255, 255, 255), 2)

    cv2.putText(frame, "BASE", (int(w*0.125)-30, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    cv2.putText(frame, "SHOULDER", (int(w*0.375)-55, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    cv2.putText(frame, "WRIST", (int(w*0.625)-30, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
    cv2.putText(frame, "ELBOW", (int(w*0.875)-30, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # -----------------------------
        # Hand center
        # -----------------------------
        xs = [lm.x for lm in hand_landmarks.landmark]
        ys = [lm.y for lm in hand_landmarks.landmark]
        cx = np.mean(xs)
        cy = np.mean(ys)

        px, py = int(cx * w), int(cy * h)
        cv2.circle(frame, (px, py), 6, (0, 255, 0), -1)

        # -----------------------------
        # Delta motion
        # -----------------------------
        if prev_center is not None:
            dx = cx - prev_center[0]
            dy = cy - prev_center[1]
        else:
            dx, dy = 0.0, 0.0

        prev_center = (cx, cy)

        # -----------------------------
        # Determine region
        # -----------------------------
        if cx < 0.25:
            region = "BASE"
            active_dx, active_dy = dx, 0.0
        elif cx < 0.50:
            region = "SHOULDER"
            active_dx, active_dy = 0.0, dy
        elif cx < 0.75:
            region = "WRIST"
            active_dx, active_dy = 0.0, dy
        else:
            region = "ELBOW"
            active_dx, active_dy = 0.0, dy

        # -----------------------------
        # Pinch → gripper only
        # -----------------------------
        thumb = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
        index = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
        pinch = np.linalg.norm(
            np.array([thumb.x, thumb.y]) -
            np.array([index.x, index.y])
        )

        if pinch < PINCH_CLOSE_THRESH:
            grip_state = "CLOSE"
        elif pinch > PINCH_OPEN_THRESH:
            grip_state = "OPEN"
        else:
            grip_state = "HOLD"

        # -----------------------------
        # Display
        # -----------------------------
        text = (
            f"{region} | "
            f"dx={active_dx:+.4f} dy={active_dy:+.4f} | "
            f"pinch={pinch:.3f} | "
            f"grip={grip_state}"
        )

        cv2.putText(frame, text, (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)

        print(text)

    cv2.imshow("4-Region Gesture Control (Up/Down Wrist)", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
