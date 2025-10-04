#!/usr/bin/env python3
"""
Leap Motion Hand Streaming Script
Captures hand tracking data and streams it over UDP to ROS2 system
"""

import leap
import socket
import json
import time
import threading
from typing import Optional, Dict, List

class HandStreamer(leap.Listener):
    def __init__(self, target_ip: str = "192.168.8.151", target_port: int = 9999):
        """
        Initialize hand streamer
        
        Args:
            target_ip: IP address of Ubuntu VM (your robot's IP)
            target_port: UDP port to send data to
        """
        super().__init__()
        self.target_ip = target_ip
        self.target_port = target_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.running = False
        
        # Gesture recognition state
        self.current_gesture = "none"
        self.gesture_start_time = 0
        self.gesture_duration_threshold = 2.0  # 2 seconds
        
        print(f"Hand Streamer initialized")
        print(f"Target: {target_ip}:{target_port}")
        print(f"Connect your Leap Motion and move your hands!")
        print(f"Gestures: Point to START, Peace sign to STOP")
        
    def on_connection_event(self, event):
        """Called when connection to Leap Motion service is established"""
        print("Connected to Leap Motion service")
        
    def on_device_event(self, event):
        """Called when a Leap Motion device is connected"""
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()
        print(f"Found Leap Motion device: {info.serial}")
        
    def on_tracking_event(self, event):
        """Called for each tracking frame - this is where we capture hand data"""
        self.frame_count += 1
        
        # Extract hand data
        hands_data = []
        for hand in event.hands:
            hand_data = self.extract_hand_data(hand)
            hands_data.append(hand_data)
        
        # Create and send message
        message = self.create_message(hands_data, event.tracking_frame_id)
        self.send_data(message)
        
        # Print stats every 60 frames
        if self.frame_count % 60 == 0:
            gesture_info = f" | Gesture: {self.current_gesture}" if self.current_gesture != "none" else ""
            self.print_stats(len(hands_data), gesture_info)
    
    def detect_gesture(self, fingers: List[Dict]) -> str:
        """Detect hand gestures based on finger positions"""
        if not fingers or len(fingers) < 5:
            return "none"
        
        # Get extension state for each finger (0=thumb, 1=index, 2=middle, 3=ring, 4=pinky)
        finger_states = [False] * 5
        for finger in fingers:
            idx = finger.get('index', -1)
            if 0 <= idx < 5:
                finger_states[idx] = finger.get('extended', False)
        
        # Count extended fingers
        extended_count = sum(finger_states)
        
        # Pointing gesture: only index finger extended (ignore thumb)
        if extended_count == 1 and finger_states[1]:  # Only index finger
            return "pointing"
        elif extended_count == 2 and finger_states[0] and finger_states[1]:  # Thumb + index
            return "pointing"  # Also accept thumb + index as pointing
        
        # Peace sign: index and middle fingers extended, others down
        if extended_count == 2 and finger_states[1] and finger_states[2]:
            return "peace"
        elif extended_count == 3 and finger_states[0] and finger_states[1] and finger_states[2]:
            return "peace"  # Accept with thumb up too
        
        return "none"
    
    def update_gesture_state(self, gesture: str):
        """Update gesture recognition state with timing"""
        current_time = time.time()
        
        if gesture != self.current_gesture:
            # New gesture detected
            self.current_gesture = gesture
            self.gesture_start_time = current_time
        
        # Check if gesture held long enough
        if gesture != "none":
            gesture_duration = current_time - self.gesture_start_time
            if gesture_duration >= self.gesture_duration_threshold:
                return f"{gesture}_confirmed"
        
        return gesture
        
    def extract_hand_data(self, hand) -> Dict:
        """Extract relevant hand data for robot control including finger positions"""
        hand_type = "left" if str(hand.type) == "HandType.Left" else "right"
        
        # Extract finger data for gesture recognition
        fingers = []
        finger_debug = []
        
        try:
            # Debug: Print available hand attributes
            if self.frame_count % 120 == 0:  # Every 2 seconds at 60fps
                print(f"Hand attributes: {[attr for attr in dir(hand) if not attr.startswith('_')]}")
            
            # Try different ways to access finger data
            if hasattr(hand, 'digits'):
                for i, digit in enumerate(hand.digits):
                    finger_data = {
                        "index": i,  # 0=thumb, 1=index, 2=middle, 3=ring, 4=pinky
                        "extended": digit.is_extended,
                    }
                    fingers.append(finger_data)
                    finger_debug.append(f"F{i}:{'✓' if digit.is_extended else '✗'}")
            elif hasattr(hand, 'fingers'):
                for i, finger in enumerate(hand.fingers):
                    finger_data = {
                        "index": i,
                        "extended": finger.is_extended,
                    }
                    fingers.append(finger_data)
                    finger_debug.append(f"F{i}:{'✓' if finger.is_extended else '✗'}")
            else:
                print("Warning: No finger data found in hand object")
                
        except Exception as e:
            print(f"Warning: Could not extract finger data: {e}")
            fingers = []
        
        hand_data = {
            "id": hand.id,
            "type": hand_type,
            "palm": {
                "position": [hand.palm.position.x, hand.palm.position.y, hand.palm.position.z],
                "normal": [hand.palm.normal.x, hand.palm.normal.y, hand.palm.normal.z],
                "direction": [hand.palm.direction.x, hand.palm.direction.y, hand.palm.direction.z]
            },
            "grab_strength": hand.grab_strength,
            "pinch_strength": hand.pinch_strength,
            "fingers": fingers,
            "finger_debug": " ".join(finger_debug)
        }
        
        # Detect and update gestures
        gesture = self.detect_gesture(fingers)
        gesture_state = self.update_gesture_state(gesture)
        hand_data["gesture"] = gesture_state
        
        # Debug output for gestures
        if gesture != "none" or gesture_state.endswith("_confirmed"):
            print(f"Gesture detected: {gesture} -> {gesture_state} | Fingers: {' '.join(finger_debug)}")
        
        return hand_data
    
    def create_message(self, hands_data: List[Dict], frame_id: int) -> Dict:
        """Create network message with timestamp and hand data"""
        return {
            "timestamp": time.time(),
            "frame_id": frame_id,
            "frame_count": self.frame_count,
            "hands": hands_data
        }
    
    def send_data(self, message: Dict):
        """Send data over UDP"""
        try:
            json_data = json.dumps(message)
            self.socket.sendto(json_data.encode('utf-8'), (self.target_ip, self.target_port))
        except Exception as e:
            print(f"Send error: {e}")
    
    def print_stats(self, hand_count: int, extra_info: str = ""):
        """Print performance statistics"""
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        print(f"Frames: {self.frame_count:6d} | FPS: {fps:6.1f} | Hands: {hand_count}{extra_info}")
    
    def stream(self):
        """Main streaming loop"""
        print("\nStarting hand tracking stream...")
        print("Press Ctrl+C to stop\n")
        
        connection = leap.Connection()
        connection.add_listener(self)
        self.running = True
        
        try:
            with connection.open():
                connection.set_tracking_mode(leap.TrackingMode.Desktop)
                while self.running:
                    time.sleep(0.1)  # Keep main thread alive
                    
        except KeyboardInterrupt:
            print(f"\n\nStreaming stopped by user")
        except Exception as e:
            print(f"\n\nError: {e}")
        finally:
            self.running = False
            self.socket.close()
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            print(f"Final stats: {self.frame_count} frames in {elapsed:.1f}s (avg {avg_fps:.1f} FPS)")

def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║                 LEAP MOTION → ROBOT STREAMER                 ║
║              High-frequency hand tracking stream             ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    # Configuration
    TARGET_IP = "192.168.8.151"  # Ubuntu VM IP address
    TARGET_PORT = 9999
    
    print(f"Configuration:")
    print(f"   Target IP: {TARGET_IP}")
    print(f"   Target Port: {TARGET_PORT}")
    print()
    
    # Start streaming
    streamer = HandStreamer(TARGET_IP, TARGET_PORT)
    streamer.stream()

if __name__ == "__main__":
    main()