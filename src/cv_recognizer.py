import mediapipe as mp
from utils import GestureType
import math

class CVHandRecognizer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands

    def _get_grip_gesture(self, landmarks):
        """Helper to determine if a hand is open or closed."""
        try:
            tip_indices = [8, 12, 16, 20]
            pip_indices = [6, 10, 14, 18]
            tip_y = [landmarks[i].y for i in tip_indices]
            pip_y = [landmarks[i].y for i in pip_indices]
            if all(tip > pip for tip, pip in zip(tip_y, pip_y)):
                return GestureType.CLOSED_HAND
            return GestureType.OPEN_HAND
        except:
            return GestureType.OPEN_HAND

    def _map_value(self, value, from_min, from_max, to_min, to_max):
        """Maps a value from one range to another."""
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def analyze_bimanual(self, right_hand, left_hand):
        """Bimanual control with elbow on right hand depth."""
        rh_wrist = right_hand[self.mp_hands.HandLandmark.WRIST]
        lh_wrist = left_hand[self.mp_hands.HandLandmark.WRIST]
        
        # --- Right Hand Controls ---
        # Base rotation from X-axis
        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        if rh_wrist.x < 0.7: base_cmd = GestureType.MOVE_LEFT
        elif rh_wrist.x > 0.8: base_cmd = GestureType.MOVE_RIGHT
        
        # Shoulder angle from Y-axis
        sh_angle = self._map_value(rh_wrist.y, 0.8, 0.2, 0, -90)
        sh_angle = max(-90, min(0, sh_angle))

        # --- NEW: Elbow angle from Right Hand's depth (Z-axis) ---
        # Hand closer (z=-0.4) -> Elbow open (0 deg) | Hand farther (z=0.4) -> Elbow closed (150 deg)
        el_angle = self._map_value(rh_wrist.z, 1e-7, 5e-7, 150, 0)
        # print('ELBOW Z:', rh_wrist.z, '-> Angle:', el_angle)
        el_angle = max(0, min(150, el_angle))

        # --- Left Hand Controls ---
        # --- NEW: Gripper is back on the LEFT hand ---
        grip_cmd = self._get_grip_gesture(left_hand)

        # --- NEW: Wrist angle from Left Hand's up/down (Y-axis) ---
        wr_angle = self._map_value(lh_wrist.y, 0.2, 0.8, 180, 0)
        wr_angle = max(0, min(180, wr_angle))

        debug_str = f"Grip:{grip_cmd.name} | Base:{base_cmd.name} | Sh:{int(sh_angle)} El:{int(el_angle)} Wr:{int(wr_angle)}"
        return grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str

    def analyze_unimanual(self, right_hand):
        """Unimanual control (remains unchanged)."""
        rh_wrist = right_hand[self.mp_hands.HandLandmark.WRIST]

        base_cmd = GestureType.STOP_MOVE_HORIZONTAL
        if rh_wrist.x < 0.7: base_cmd = GestureType.MOVE_LEFT
        elif rh_wrist.x > 0.8: base_cmd = GestureType.MOVE_RIGHT

        sh_angle = self._map_value(rh_wrist.y, 0.8, 0.2, 0, -90)
        sh_angle = max(-90, min(0, sh_angle))

        el_angle = self._map_value(rh_wrist.z, 1e-7, 8e-7, 150, 0)
        print('ELBOW Z:', rh_wrist.z, '-> Angle:', el_angle)
        el_angle = max(0, min(150, el_angle))

        wr_angle = 90

        grip_cmd = self._get_grip_gesture(right_hand)

        debug_str = f"Grip:{grip_cmd.name} | Base:{base_cmd.name} | Sh:{int(sh_angle)} El:{int(el_angle)} Wr:{int(wr_angle)}"
        return grip_cmd, base_cmd, sh_angle, el_angle, wr_angle, debug_str