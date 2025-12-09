from enum import Enum


class GestureType(Enum):
    UNKNOWN = "unknown"
    OPEN_HAND = "open_hand"
    CLOSED_HAND = "closed_hand"
    # New States for Stepper
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    STOP_MOVE_HORIZONTAL = "stop_move_horizontal"

    MOVE_UP = "move_up"
    MOVE_DOWN = "move_down"
    STOP_MOVE_VERTICAL = "stop_move_vertical"