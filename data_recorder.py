#!/usr/bin/env python3
"""
Leap Motion Data Recorder for ML Training
Records hand tracking data and saves to JSON files for training intentional vs unintentional movement classifier
"""

import leap
import json
import time
import os
import sys
from typing import Dict, List
from datetime import datetime
import threading

class HandDataRecorder(leap.Listener):
    def __init__(self):
        """Initialize the data recorder"""
        super().__init__()
        
        # Recording state
        self.recording = False
        self.recorded_frames = []
        self.frame_count = 0
        self.recording_start_time = 0
        self.recording_duration = 45  # Fixed 45 second recordings
        
        # Create data directories automatically
        self.intentional_dir = "intentional"
        self.unintentional_dir = "unintentional"
        
        # Create folders if they don't exist
        if not os.path.exists(self.intentional_dir):
            os.makedirs(self.intentional_dir)
            print(f"✓ Created folder: {self.intentional_dir}")
        else:
            print(f"✓ Using existing folder: {self.intentional_dir}")
            
        if not os.path.exists(self.unintentional_dir):
            os.makedirs(self.unintentional_dir)
            print(f"✓ Created folder: {self.unintentional_dir}")
        else:
            print(f"✓ Using existing folder: {self.unintentional_dir}")
        
        print("Hand Data Recorder initialized")
        print(f"\nRecordings are fixed at {self.recording_duration} seconds each")
        print("\nControls:")
        print("  'i' + Enter: Record INTENTIONAL movements (45s auto-recording)")
        print("  'u' + Enter: Record UNINTENTIONAL movements (45s auto-recording)") 
        print("  'q' + Enter: Quit")
        print("\nConnect your Leap Motion and get ready!")
        
    def on_connection_event(self, event):
        """Called when connection to Leap Motion service is established"""
        print("✓ Connected to Leap Motion service")
        
    def on_device_event(self, event):
        """Called when a Leap Motion device is connected"""
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()
        print(f"✓ Found Leap Motion device: {info.serial}")
        
    def on_tracking_event(self, event):
        """Called for each tracking frame - capture data if recording"""
        if not self.recording:
            return
            
        # Extract hand data
        hands_data = []
        for hand in event.hands:
            hand_data = self.extract_hand_data(hand)
            hands_data.append(hand_data)
        
        # Create frame data with timestamp
        frame_data = {
            "timestamp": time.time() - self.recording_start_time,
            "frame_id": event.tracking_frame_id,
            "hands": hands_data
        }
        
        self.recorded_frames.append(frame_data)
        self.frame_count += 1
    
    def extract_hand_data(self, hand) -> Dict:
        """Extract hand data"""
        hand_type = "left" if str(hand.type) == "HandType.Left" else "right"
        
        # Extract finger data
        fingers = []
        try:
            if hasattr(hand, 'digits'):
                for i, digit in enumerate(hand.digits):
                    finger_data = {
                        "index": i,
                        "extended": digit.is_extended,
                    }
                    fingers.append(finger_data)
            elif hasattr(hand, 'fingers'):
                for i, finger in enumerate(hand.fingers):
                    finger_data = {
                        "index": i,
                        "extended": finger.is_extended,
                    }
                    fingers.append(finger_data)
        except Exception as e:
            fingers = []
        
        return {
            "id": hand.id,
            "type": hand_type,
            "palm": {
                "position": [hand.palm.position.x, hand.palm.position.y, hand.palm.position.z],
                "normal": [hand.palm.normal.x, hand.palm.normal.y, hand.palm.normal.z],
                "direction": [hand.palm.direction.x, hand.palm.direction.y, hand.palm.direction.z],
                "velocity": [hand.palm.velocity.x, hand.palm.velocity.y, hand.palm.velocity.z]
            },
            "grab_strength": hand.grab_strength,
            "pinch_strength": hand.pinch_strength,
            "fingers": fingers
        }
    
    def countdown_timer(self):
        """Display countdown during recording"""
        while self.recording:
            elapsed = time.time() - self.recording_start_time
            remaining = self.recording_duration - elapsed
            
            if remaining <= 0:
                break
                
            # Update countdown display
            sys.stdout.write(f"\r🔴 Recording... {remaining:.1f}s remaining | Frames: {self.frame_count}    ")
            sys.stdout.flush()
            time.sleep(0.1)
    
    def start_recording(self, label_type: str):
        """Start recording with automatic 45 second timer"""
        if self.recording:
            print("Already recording! Please wait...")
            return
            
        self.recording = True
        self.recorded_frames = []
        self.frame_count = 0
        self.recording_start_time = time.time()
        self.current_label = label_type
        
        print(f"\n🔴 RECORDING {label_type.upper()} movements for {self.recording_duration} seconds...")
        print("Get ready...\n")
        
        # Start countdown display in separate thread
        countdown_thread = threading.Thread(target=self.countdown_timer)
        countdown_thread.daemon = True
        countdown_thread.start()
        
        # Wait for recording duration
        time.sleep(self.recording_duration)
        
        # Stop recording
        self.recording = False
        print("\n")  # New line after countdown
        
        # Auto-save
        self.save_recording()
        
        # Clear terminal and show menu again
        self.clear_terminal()
        self.print_help()
    
    def save_recording(self):
        """Save recording to file"""
        duration = time.time() - self.recording_start_time
        
        if len(self.recorded_frames) == 0:
            print("⚠️  No frames recorded!")
            return
        
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.current_label}_{timestamp}.json"
        
        # Choose directory based on label
        if self.current_label == "intentional":
            filepath = os.path.join(self.intentional_dir, filename)
        else:
            filepath = os.path.join(self.unintentional_dir, filename)
        
        # Create metadata
        recording_data = {
            "metadata": {
                "label": self.current_label,
                "duration": duration,
                "frame_count": len(self.recorded_frames),
                "fps": len(self.recorded_frames) / duration,
                "recorded_at": datetime.now().isoformat()
            },
            "frames": self.recorded_frames
        }
        
        # Save to file
        try:
            with open(filepath, 'w') as f:
                json.dump(recording_data, f, indent=2)
            
            print(f"✅ SAVED: {filename}")
            print(f"   Frames: {len(self.recorded_frames)}")
            print(f"   Duration: {duration:.1f}s")
            print(f"   Average FPS: {len(self.recorded_frames)/duration:.1f}")
            print(f"   File: {filepath}")
            
        except Exception as e:
            print(f"❌ Error saving file: {e}")
    
    def clear_terminal(self):
        """Clear the terminal screen"""
        os.system('cls' if os.name == 'nt' else 'clear')
        print("""
╔════════════════════════════════════════════════════════════════╗
║                    LEAP MOTION DATA RECORDER                   ║
║              Record training data for ML classifier            ║
╚════════════════════════════════════════════════════════════════╝
""")
    
    def run(self):
        """Main recording loop with user interface"""
        connection = leap.Connection()
        connection.add_listener(self)
        
        try:
            with connection.open():
                connection.set_tracking_mode(leap.TrackingMode.Desktop)
                
                print("\nReady! Leap Motion connected.")
                self.print_help()
                
                while True:
                    command = input("\nEnter command: ").strip().lower()
                    
                    if command == 'i':
                        self.start_recording("intentional")
                    elif command == 'u':
                        self.start_recording("unintentional")
                    elif command == 'q':
                        if self.recording:
                            print("Recording in progress! Please wait...")
                            continue
                        print("\nGoodbye! Total recordings collected:")
                        intentional_count = len([f for f in os.listdir(self.intentional_dir) if f.endswith('.json')])
                        unintentional_count = len([f for f in os.listdir(self.unintentional_dir) if f.endswith('.json')])
                        print(f"  Intentional: {intentional_count}")
                        print(f"  Unintentional: {unintentional_count}")
                        print(f"  Total: {intentional_count + unintentional_count}")
                        break
                    elif command == 'h':
                        self.print_help()
                    else:
                        print("Unknown command. Type 'h' for help.")
                        
        except KeyboardInterrupt:
            print(f"\n\nStopped by user")
        except Exception as e:
            print(f"Error: {e}")
    
    def print_help(self):
        """Print help message"""
        print("\nCommands:")
        print(f"  i - Record INTENTIONAL movements ({self.recording_duration}s auto-recording)")
        print(f"  u - Record UNINTENTIONAL movements ({self.recording_duration}s auto-recording)")
        print("  h - Show this help")
        print("  q - Quit program")
        
        # Show existing files
        intentional_files = len([f for f in os.listdir(self.intentional_dir) if f.endswith('.json')])
        unintentional_files = len([f for f in os.listdir(self.unintentional_dir) if f.endswith('.json')])
        total_files = intentional_files + unintentional_files
        print(f"\nExisting recordings: {intentional_files} intentional, {unintentional_files} unintentional (Total: {total_files}/400)")

def main():
    print("""
╔════════════════════════════════════════════════════════════════╗
║                    LEAP MOTION DATA RECORDER                   ║
║              Record training data for ML classifier            ║
╚════════════════════════════════════════════════════════════════╝
""")
    
    recorder = HandDataRecorder()
    recorder.run()

if __name__ == "__main__":
    main()