"""
Simple Gesture Control Example
Demonstrates how to use gesture recognition for practical applications
like controlling media playback, presentations, or games.
"""

import leap
import time
from gesture_recognition import GestureRecognizer, GestureType


class MediaController:
    """Example application: Control media playback with gestures"""
    
    def __init__(self):
        self.is_playing = False
        self.volume = 50
        self.current_track = 1
        
    def play_pause(self):
        self.is_playing = not self.is_playing
        status = "▶️ PLAYING" if self.is_playing else "⏸️ PAUSED"
        print(f"Media: {status}")
        
    def next_track(self):
        self.current_track += 1
        print(f"🎵 Next track: Track {self.current_track}")
        
    def previous_track(self):
        self.current_track = max(1, self.current_track - 1)
        print(f"🎵 Previous track: Track {self.current_track}")
        
    def volume_up(self):
        self.volume = min(100, self.volume + 10)
        print(f"🔊 Volume: {self.volume}%")
        
    def volume_down(self):
        self.volume = max(0, self.volume - 10)
        print(f"🔉 Volume: {self.volume}%")


class GestureControlListener(leap.Listener):
    def __init__(self):
        self.gesture_recognizer = GestureRecognizer()
        self.media_controller = MediaController()
        self.last_gesture_time = {}  # Prevent rapid gesture triggering
        self.cooldown_period = 1.0  # seconds between same gesture
        
    def on_connection_event(self, event):
        print("🎮 Gesture Control Ready!")
        print("Gestures:")
        print("  ✊ Closed Fist = Play/Pause")
        print("  👉 Pointing = Select/Click")
        print("  🤏 Pinch = Grab/Hold")
        print("  ✌️ Peace Sign = Stop/Reset")
        print("  👍 Thumbs Up = Like/Confirm")
        print("  ⬅️ Swipe Left = Previous")
        print("  ➡️ Swipe Right = Next")
        print("  ⬆️ Swipe Up = Volume Up")
        print("  ⬇️ Swipe Down = Volume Down")
        
    def on_device_event(self, event):
        print("📱 Device connected and ready for gestures!")
        
    def on_tracking_event(self, event):
        current_time = time.time()
        
        for hand in event.hands:
            detected_gesture = self.gesture_recognizer.analyze_hand(hand)
            
            if detected_gesture and self._can_process_gesture(hand.id, detected_gesture, current_time):
                self._handle_gesture(detected_gesture, hand)
                self.last_gesture_time[f"{hand.id}_{detected_gesture.value}"] = current_time
    
    def _can_process_gesture(self, hand_id, gesture, current_time):
        """Check if enough time has passed since the last same gesture"""
        key = f"{hand_id}_{gesture.value}"
        return key not in self.last_gesture_time or \
               (current_time - self.last_gesture_time[key]) > self.cooldown_period
    
    def _handle_gesture(self, gesture, hand):
        """Map gestures to actions"""
        hand_type = 'left' if str(hand.type) == "HandType.Left" else 'right'
        
        print(f"\n🎯 {hand_type.upper()} HAND: {gesture.value.upper()}")
        
        if gesture == GestureType.CLOSED_FIST:
            self.media_controller.play_pause()
            
        elif gesture == GestureType.POINTING:
            print("👆 Click action triggered")
            
        elif gesture == GestureType.PINCH:
            print("🤏 Grab/Select action triggered")
            
        elif gesture == GestureType.PEACE_SIGN:
            print("✌️ Stop/Reset action triggered")
            
        elif gesture == GestureType.THUMBS_UP:
            print("👍 Like/Confirm action triggered")
            
        elif gesture == GestureType.SWIPE_LEFT:
            self.media_controller.previous_track()
            
        elif gesture == GestureType.SWIPE_RIGHT:
            self.media_controller.next_track()
            
        elif gesture == GestureType.SWIPE_UP:
            self.media_controller.volume_up()
            
        elif gesture == GestureType.SWIPE_DOWN:
            self.media_controller.volume_down()
            
        elif gesture == GestureType.OPEN_HAND:
            print("✋ Open hand - Release action")


class CursorController:
    """Example: Control cursor with hand position"""
    
    def __init__(self):
        self.last_position = None
        
    def update_cursor(self, hand_position):
        """Update cursor based on hand position"""
        x, y, z = hand_position
        
        # Normalize coordinates (you'd map these to screen coordinates)
        normalized_x = max(0, min(1, (x + 200) / 400))  # Assuming -200 to 200 range
        normalized_y = max(0, min(1, (300 - y) / 300))   # Assuming 0 to 300 range
        
        print(f"🖱️ Cursor: ({normalized_x:.2f}, {normalized_y:.2f})")
        self.last_position = (normalized_x, normalized_y)


def main():
    print("🚀 Starting Gesture Control Demo...")
    print("This demo shows how to use gestures to control applications.")
    print("Wave your hands in front of the Leap Motion sensor!\n")
    
    listener = GestureControlListener()
    connection = leap.Connection()
    connection.add_listener(listener)
    
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n👋 Gesture control demo stopped.")


if __name__ == "__main__":
    main()