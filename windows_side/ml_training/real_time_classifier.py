#!/usr/bin/env python3
"""
Real-time Hand Movement Classifier
Integrates trained ML model with live Leap Motion stream
OPTIMIZED FOR LOW LATENCY
"""

import leap
import numpy as np
import joblib
import time
from collections import deque
from typing import Dict, List
import os

class RealTimeHandClassifier(leap.Listener):
    def __init__(self, model_path: str, scaler_path: str, window_length_sec=0.5):
        """
        Initialize real-time classifier
        
        Args:
            model_path: Path to trained model pickle file
            scaler_path: Path to feature scaler pickle file
            window_length_sec: Window length in seconds (reduced to 0.5s for faster response)
        """
        super().__init__()
        
        # Load trained model and scaler
        print(f"Loading model from: {model_path}")
        self.model = joblib.load(model_path)
        print(f"Loading scaler from: {scaler_path}")
        self.scaler = joblib.load(scaler_path)
        
        # Window parameters - OPTIMIZED FOR LOW LATENCY
        self.window_length = window_length_sec
        self.window_buffer = deque(maxlen=100)  # Buffer for ~0.5 second at 100fps
        
        # Statistics
        self.frame_count = 0
        self.classification_count = 0
        self.intentional_count = 0
        self.unintentional_count = 0
        self.start_time = time.time()
        
        # Current state
        self.current_classification = "unknown"
        self.current_confidence = 0.0
        
        print(f"Real-time classifier initialized")
        print(f"Window length: {window_length_sec}s (OPTIMIZED FOR LOW LATENCY)")
        print(f"Model loaded successfully\n")
    
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
        print(f"✓ Found Leap Motion device: {info.serial}\n")
        print("=" * 70)
        print("REAL-TIME CLASSIFICATION ACTIVE - LOW LATENCY MODE")
        print("=" * 70)
        print("Move your hands and watch the classification in real-time!")
        print("Press Ctrl+C to stop\n")
        
    def on_tracking_event(self, event):
        """Called for each tracking frame - classify in real-time"""
        self.frame_count += 1
        
        # Extract frame data
        frame_data = {
            'timestamp': time.time(),
            'hands': []
        }
        
        for hand in event.hands:
            hand_data = self.extract_hand_data(hand)
            frame_data['hands'].append(hand_data)
        
        # Add to buffer
        self.window_buffer.append(frame_data)
        
        # Classify every 5 frames (REDUCED from 10 for faster updates)
        if self.frame_count % 5 == 0 and len(self.window_buffer) >= 30:
            self.classify_current_window()
    
    def extract_hand_data(self, hand) -> Dict:
        """Extract hand data from Leap Motion frame"""
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
        except Exception:
            fingers = []
        
        return {
            "id": hand.id,
            "type": hand_type,
            "palm": {
                "position": [hand.palm.position.x, hand.palm.position.y, hand.palm.position.z],
                "velocity": [hand.palm.velocity.x, hand.palm.velocity.y, hand.palm.velocity.z],
                "normal": [hand.palm.normal.x, hand.palm.normal.y, hand.palm.normal.z],
                "direction": [hand.palm.direction.x, hand.palm.direction.y, hand.palm.direction.z]
            },
            "grab_strength": hand.grab_strength,
            "pinch_strength": hand.pinch_strength,
            "fingers": fingers
        }
    
    def compute_window_features(self, window: List[Dict]) -> np.ndarray:
        """
        Compute features for current window
        Returns feature vector matching training format
        """
        # Extract data from window
        positions = []
        velocities = []
        grab_strengths = []
        pinch_strengths = []
        finger_extensions = []
        hand_present = []
        
        for frame in window:
            if len(frame['hands']) > 0:
                hand = frame['hands'][0]
                palm = hand['palm']
                
                positions.append(palm['position'])
                velocities.append(palm['velocity'])
                grab_strengths.append(hand['grab_strength'])
                pinch_strengths.append(hand['pinch_strength'])
                
                extended_count = sum(1 for f in hand['fingers'] if f['extended'])
                finger_extensions.append(extended_count)
                hand_present.append(1)
            else:
                hand_present.append(0)
        
        # Compute features (must match training feature order)
        features = {}
        
        # Hand presence
        features['hand_presence_ratio'] = np.mean(hand_present)
        features['hand_present_frames'] = sum(hand_present)
        
        if len(positions) == 0:
            # No hand detected - return zero features
            return np.zeros(18)
        
        positions = np.array(positions)
        velocities = np.array(velocities)
        
        # Displacement
        if len(positions) > 1:
            features['displacement_mm'] = np.linalg.norm(positions[-1] - positions[0])
        else:
            features['displacement_mm'] = 0.0
        
        # Velocity statistics
        velocity_magnitudes = np.linalg.norm(velocities, axis=1)
        features['velocity_mean'] = np.mean(velocity_magnitudes)
        features['velocity_std'] = np.std(velocity_magnitudes)
        features['velocity_max'] = np.max(velocity_magnitudes)
        
        # Acceleration
        if len(velocity_magnitudes) > 1:
            accelerations = np.diff(velocity_magnitudes)
            features['acceleration_mean'] = np.mean(np.abs(accelerations))
        else:
            features['acceleration_mean'] = 0.0
        
        # Direction consistency
        if len(velocities) > 1:
            velocity_norms = np.linalg.norm(velocities, axis=1, keepdims=True)
            velocity_norms[velocity_norms == 0] = 1
            normalized_velocities = velocities / velocity_norms
            
            dot_products = []
            for i in range(len(normalized_velocities) - 1):
                dot = np.dot(normalized_velocities[i], normalized_velocities[i+1])
                dot_products.append(np.clip(dot, -1, 1))
            
            features['direction_consistency'] = np.mean(dot_products) if dot_products else 1.0
        else:
            features['direction_consistency'] = 1.0
        
        # Position statistics
        features['position_x_mean'] = np.mean(positions[:, 0])
        features['position_y_mean'] = np.mean(positions[:, 1])
        features['position_z_mean'] = np.mean(positions[:, 2])
        features['position_std'] = np.mean(np.std(positions, axis=0))
        
        # Grab and pinch
        features['grab_mean'] = np.mean(grab_strengths)
        features['grab_max'] = np.max(grab_strengths)
        features['pinch_mean'] = np.mean(pinch_strengths)
        features['pinch_max'] = np.max(pinch_strengths)
        
        # Finger extension
        features['finger_mean'] = np.mean(finger_extensions)
        features['finger_std'] = np.std(finger_extensions)
        
        # Convert to array in correct order
        feature_order = [
            'hand_presence_ratio', 'hand_present_frames', 'displacement_mm',
            'velocity_mean', 'velocity_std', 'velocity_max', 'acceleration_mean',
            'direction_consistency', 'position_x_mean', 'position_y_mean',
            'position_z_mean', 'position_std', 'grab_mean', 'grab_max',
            'pinch_mean', 'pinch_max', 'finger_mean', 'finger_std'
        ]
        
        feature_vector = np.array([features[f] for f in feature_order])
        return feature_vector
    
    def classify_current_window(self):
        """Classify the current window buffer"""
        # Get window from buffer
        window = list(self.window_buffer)
        
        # Compute features
        features = self.compute_window_features(window)
        
        # Scale features
        features_scaled = self.scaler.transform(features.reshape(1, -1))
        
        # Predict
        prediction = self.model.predict(features_scaled)[0]
        confidence = np.max(self.model.predict_proba(features_scaled))
        
        self.current_classification = prediction
        self.current_confidence = confidence
        self.classification_count += 1
        
        if prediction == "intentional":
            self.intentional_count += 1
        else:
            self.unintentional_count += 1
        
        # Display result
        self.display_classification()
    
    def display_classification(self):
        """Display current classification with color coding"""
        elapsed = time.time() - self.start_time
        
        # Color codes for terminal
        if self.current_classification == "intentional":
            color = "\033[92m"  # Green
            symbol = "✓"
        else:
            color = "\033[91m"  # Red
            symbol = "✗"
        
        reset = "\033[0m"
        
        # Clear line and print
        print(f"\r{color}{symbol} {self.current_classification.upper():15s}{reset} "
              f"| Conf: {self.current_confidence:.2f} "
              f"| Intent: {self.intentional_count:4d} "
              f"| Uninten: {self.unintentional_count:4d} "
              f"| FPS: {self.frame_count/elapsed:.0f} "
              f"| Time: {elapsed:.1f}s", end='', flush=True)
    
    def run(self):
        """Main classification loop"""
        connection = leap.Connection()
        connection.add_listener(self)
        
        try:
            with connection.open():
                connection.set_tracking_mode(leap.TrackingMode.Desktop)
                
                while True:
                    time.sleep(0.05)  # Reduced sleep for faster response
                    
        except KeyboardInterrupt:
            print(f"\n\n{'='*70}")
            print("CLASSIFICATION STOPPED")
            print(f"{'='*70}")
            print(f"Total classifications: {self.classification_count}")
            print(f"  Intentional: {self.intentional_count} ({self.intentional_count/self.classification_count*100:.1f}%)")
            print(f"  Unintentional: {self.unintentional_count} ({self.unintentional_count/self.classification_count*100:.1f}%)")
            print(f"Runtime: {time.time() - self.start_time:.1f}s")
            print(f"Average FPS: {self.frame_count/(time.time() - self.start_time):.1f}")
        except Exception as e:
            print(f"\nError: {e}")

def main():
    print("""
╔════════════════════════════════════════════════════════════════╗
║              REAL-TIME HAND MOVEMENT CLASSIFIER                ║
║           Test your trained model with live data               ║
║                    LOW LATENCY MODE                            ║
╚════════════════════════════════════════════════════════════════╝
""")
    
    # Model paths
    MODEL_DIR = "models"
    MODEL_FILE = "hand_classifier_20251002_221301.pkl"
    SCALER_FILE = "feature_scaler_20251002_221301.pkl"
    
    model_path = os.path.join(MODEL_DIR, MODEL_FILE)
    scaler_path = os.path.join(MODEL_DIR, SCALER_FILE)
    
    # Check if files exist
    if not os.path.exists(model_path):
        print(f"Error: Model file not found at {model_path}")
        print("Make sure you moved the model files to the 'models' folder")
        return
    
    if not os.path.exists(scaler_path):
        print(f"Error: Scaler file not found at {scaler_path}")
        return
    
    # Initialize classifier
    classifier = RealTimeHandClassifier(model_path, scaler_path)
    
    # Run
    classifier.run()

if __name__ == "__main__":
    main()