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
        
        print(f"Hand Streamer initialized")
        print(f"Target: {target_ip}:{target_port}")
        print(f"Connect your Leap Motion and move your hands!")
        
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
            self.print_stats(len(hands_data))
        
    def extract_hand_data(self, hand) -> Dict:
        """Extract relevant hand data for robot control"""
        hand_type = "left" if str(hand.type) == "HandType.Left" else "right"
        return {
            "id": hand.id,
            "type": hand_type,
            "palm": {
                "position": [hand.palm.position.x, hand.palm.position.y, hand.palm.position.z],
                "normal": [hand.palm.normal.x, hand.palm.normal.y, hand.palm.normal.z],
                "direction": [hand.palm.direction.x, hand.palm.direction.y, hand.palm.direction.z]
            },
            "grab_strength": hand.grab_strength,
            "pinch_strength": hand.pinch_strength
        }
    
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
    
    def print_stats(self, hand_count: int):
        """Print performance statistics"""
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        print(f"Frames: {self.frame_count:6d} | FPS: {fps:6.1f} | Hands: {hand_count}")
    
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