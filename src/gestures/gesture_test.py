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
    max_num_hands=1,   # unimanual
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
WRIST_NEUTRAL = 1.7
WRIST_MIN = 0.2
WRIST_MAX = 2.6
WRIST_SCALE = 3.0

PINCH_CLOSE_THRESH = 0.04
PINCH_OPEN_THRESH = 0.07

wrist_value = WRIST_NEUTRAL

print("Gesture test with spatial regions + contextual clutching")
print("LEFT  : base + gripper")
print("CENTER: shoulder + wrist (pinch)")
print("RIGHT : elbow + gripper")
print("ESC to quit\n")

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
    left_x = int(w * 0.33)
    right_x = int(w * 0.66)
    cv2.line(frame, (left_x, 0), (left_x, h), (255, 255, 255), 2)
    cv2.line(frame, (right_x, 0), (right_x, h), (255, 255, 255), 2)

    cv2.putText(frame, "BASE", (int(w * 0.16) - 40, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "SHOULDER / WRIST", (int(w * 0.5) - 90, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, "ELBOW", (int(w * 0.83) - 40, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
        if cx < 0.33:
            region = "LEFT"
            active_dx = dx
            active_dy = 0.0
        elif cx < 0.66:
            region = "CENTER"
            active_dx = 0.0
            active_dy = dy
        else:
            region = "RIGHT"
            active_dx = 0.0
            active_dy = dy

        # -----------------------------
        # Pinch (thumb–index)
        # -----------------------------
        thumb = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
        index = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
        pinch = np.linalg.norm(
            np.array([thumb.x, thumb.y]) -
            np.array([index.x, index.y])
        )

        # -----------------------------
        # Contextual clutching
        # -----------------------------
        if region == "CENTER":
            # Wrist control only
            wrist_delta = (PINCH_OPEN_THRESH - pinch) * WRIST_SCALE
            wrist_value += wrist_delta
            wrist_value = np.clip(wrist_value, WRIST_MIN, WRIST_MAX)
            grip_state = "—"

        else:
            # Gripper control only
            if pinch < PINCH_CLOSE_THRESH:
                grip_state = "CLOSE"
            elif pinch > PINCH_OPEN_THRESH:
                grip_state = "OPEN"
            else:
                grip_state = "HOLD"

        # -----------------------------
        # Display + debug
        # -----------------------------
        text = (
            f"{region} | "
            f"dx={active_dx:+.4f} dy={active_dy:+.4f} | "
            f"pinch={pinch:.3f} | "
            f"wrist={wrist_value:.2f} | "
            f"grip={grip_state}"
        )

        cv2.putText(
            frame, text,
            (10, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 255, 0), 2
        )

        print(text)

    cv2.imshow("Gesture Regions + Clutching", frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
