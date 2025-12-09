from utils import GestureType

class BaseHandRecognizer:
    def __init__(self):
        pass
    
    def analyze_hand(self, hand):
        """
        Returns a tuple: (GripCommand, MoveHorizontalCommand, MoveVerticalCommand, DebugInfo)
        """

        return None, None, None, None
