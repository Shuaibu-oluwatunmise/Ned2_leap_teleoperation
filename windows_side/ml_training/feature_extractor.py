#!/usr/bin/env python3
"""
Feature Extraction for Leap Motion Hand Movement Classification
Converts JSON recordings into ML-ready features using sliding windows
"""

import json
import os
import numpy as np
import pandas as pd
from pathlib import Path
from typing import List, Dict, Tuple
from datetime import datetime

class LeapMotionFeatureExtractor:
    def __init__(self, window_length_sec=1.0, window_stride_sec=0.5):
        """
        Initialize feature extractor
        
        Args:
            window_length_sec: Length of each window in seconds (default 1.0s)
            window_stride_sec: Stride between windows in seconds (default 0.5s)
        """
        self.window_length = window_length_sec
        self.window_stride = window_stride_sec
        
        print(f"Feature Extractor initialized:")
        print(f"  Window length: {window_length_sec}s")
        print(f"  Window stride: {window_stride_sec}s")
        print(f"  Window overlap: {(1 - window_stride_sec/window_length_sec)*100:.0f}%")
    
    def load_recording(self, filepath: str) -> Tuple[Dict, List[Dict]]:
        """Load a JSON recording file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        return data['metadata'], data['frames']
    
    def extract_windows(self, frames: List[Dict], metadata: Dict) -> List[List[Dict]]:
        """
        Extract sliding windows from frames
        
        Returns list of windows, where each window is a list of frames
        """
        if len(frames) == 0:
            return []
        
        # Get FPS from metadata to convert time to frame indices
        fps = metadata.get('fps', 100)  # Default to 100 if not specified
        
        window_frames = int(self.window_length * fps)
        stride_frames = int(self.window_stride * fps)
        
        windows = []
        start_idx = 0
        
        while start_idx + window_frames <= len(frames):
            window = frames[start_idx:start_idx + window_frames]
            windows.append(window)
            start_idx += stride_frames
        
        return windows
    
    def compute_window_features(self, window: List[Dict]) -> Dict:
        """
        Compute features for a single window
        
        Returns dictionary of feature name -> value
        """
        features = {}
        
        # Extract all hand data from window
        positions = []
        velocities = []
        normals = []
        directions = []
        grab_strengths = []
        pinch_strengths = []
        finger_extensions = []
        hand_present = []
        
        for frame in window:
            if len(frame['hands']) > 0:
                # Use first hand if multiple (can modify to handle both)
                hand = frame['hands'][0]
                
                palm = hand['palm']
                positions.append(palm['position'])
                velocities.append(palm['velocity'])
                normals.append(palm['normal'])
                directions.append(palm['direction'])
                
                grab_strengths.append(hand['grab_strength'])
                pinch_strengths.append(hand['pinch_strength'])
                
                # Count extended fingers
                extended_count = sum(1 for f in hand['fingers'] if f['extended'])
                finger_extensions.append(extended_count)
                
                hand_present.append(1)
            else:
                hand_present.append(0)
        
        # Feature 1: Hand presence (tracking continuity)
        features['hand_presence_ratio'] = np.mean(hand_present)
        features['hand_present_frames'] = sum(hand_present)
        
        # If no hand detected in window, return minimal features
        if len(positions) == 0:
            features.update({
                'displacement_mm': 0.0,
                'velocity_mean': 0.0,
                'velocity_std': 0.0,
                'velocity_max': 0.0,
                'acceleration_mean': 0.0,
                'direction_consistency': 0.0,
                'position_x_mean': 0.0,
                'position_y_mean': 0.0,
                'position_z_mean': 0.0,
                'position_std': 0.0,
                'grab_mean': 0.0,
                'grab_max': 0.0,
                'pinch_mean': 0.0,
                'pinch_max': 0.0,
                'finger_mean': 0.0,
                'finger_std': 0.0
            })
            return features
        
        # Convert to numpy arrays
        positions = np.array(positions)
        velocities = np.array(velocities)
        
        # Feature 2: Displacement (total movement in window)
        if len(positions) > 1:
            displacement = np.linalg.norm(positions[-1] - positions[0])
            features['displacement_mm'] = displacement
        else:
            features['displacement_mm'] = 0.0
        
        # Feature 3-5: Velocity statistics
        velocity_magnitudes = np.linalg.norm(velocities, axis=1)
        features['velocity_mean'] = np.mean(velocity_magnitudes)
        features['velocity_std'] = np.std(velocity_magnitudes)
        features['velocity_max'] = np.max(velocity_magnitudes)
        
        # Feature 6: Acceleration (change in velocity)
        if len(velocity_magnitudes) > 1:
            accelerations = np.diff(velocity_magnitudes)
            features['acceleration_mean'] = np.mean(np.abs(accelerations))
        else:
            features['acceleration_mean'] = 0.0
        
        # Feature 7: Direction consistency (how stable is movement direction)
        if len(velocities) > 1:
            # Normalize velocity vectors
            velocity_norms = np.linalg.norm(velocities, axis=1, keepdims=True)
            velocity_norms[velocity_norms == 0] = 1  # Avoid division by zero
            normalized_velocities = velocities / velocity_norms
            
            # Compute pairwise dot products
            dot_products = []
            for i in range(len(normalized_velocities) - 1):
                dot = np.dot(normalized_velocities[i], normalized_velocities[i+1])
                dot_products.append(np.clip(dot, -1, 1))
            
            features['direction_consistency'] = np.mean(dot_products) if dot_products else 1.0
        else:
            features['direction_consistency'] = 1.0
        
        # Feature 8-11: Position statistics (where is the hand)
        features['position_x_mean'] = np.mean(positions[:, 0])
        features['position_y_mean'] = np.mean(positions[:, 1])
        features['position_z_mean'] = np.mean(positions[:, 2])
        features['position_std'] = np.mean(np.std(positions, axis=0))
        
        # Feature 12-15: Grab and pinch strength
        features['grab_mean'] = np.mean(grab_strengths)
        features['grab_max'] = np.max(grab_strengths)
        features['pinch_mean'] = np.mean(pinch_strengths)
        features['pinch_max'] = np.max(pinch_strengths)
        
        # Feature 16-17: Finger extension patterns
        features['finger_mean'] = np.mean(finger_extensions)
        features['finger_std'] = np.std(finger_extensions)
        
        return features
    
    def process_file(self, filepath: str, label: str) -> List[Dict]:
        """
        Process a single recording file
        
        Returns list of feature dictionaries (one per window)
        """
        metadata, frames = self.load_recording(filepath)
        windows = self.extract_windows(frames, metadata)
        
        window_features = []
        for window in windows:
            features = self.compute_window_features(window)
            features['label'] = label
            features['source_file'] = os.path.basename(filepath)
            window_features.append(features)
        
        return window_features
    
    def process_directory(self, intentional_dir: str, unintentional_dir: str) -> pd.DataFrame:
        """
        Process all files in intentional and unintentional directories
        
        Returns pandas DataFrame with all features
        """
        all_features = []
        
        # Process intentional files
        print(f"\nProcessing intentional files from: {intentional_dir}")
        intentional_files = list(Path(intentional_dir).glob("*.json"))
        print(f"Found {len(intentional_files)} intentional files")
        
        for i, filepath in enumerate(intentional_files, 1):
            if i % 20 == 0:
                print(f"  Processed {i}/{len(intentional_files)} intentional files...")
            
            features = self.process_file(str(filepath), label='intentional')
            all_features.extend(features)
        
        print(f"✓ Processed all {len(intentional_files)} intentional files")
        
        # Process unintentional files
        print(f"\nProcessing unintentional files from: {unintentional_dir}")
        unintentional_files = list(Path(unintentional_dir).glob("*.json"))
        print(f"Found {len(unintentional_files)} unintentional files")
        
        for i, filepath in enumerate(unintentional_files, 1):
            if i % 20 == 0:
                print(f"  Processed {i}/{len(unintentional_files)} unintentional files...")
            
            features = self.process_file(str(filepath), label='unintentional')
            all_features.extend(features)
        
        print(f"✓ Processed all {len(unintentional_files)} unintentional files")
        
        # Convert to DataFrame
        df = pd.DataFrame(all_features)
        
        print(f"\n{'='*60}")
        print(f"Feature extraction complete!")
        print(f"{'='*60}")
        print(f"Total files processed: {len(intentional_files) + len(unintentional_files)}")
        print(f"Total windows extracted: {len(df)}")
        print(f"  Intentional windows: {len(df[df['label'] == 'intentional'])}")
        print(f"  Unintentional windows: {len(df[df['label'] == 'unintentional'])}")
        print(f"Features per window: {len(df.columns) - 2}")  # Exclude label and source_file
        
        return df

def main():
    print("""
╔════════════════════════════════════════════════════════════════╗
║              LEAP MOTION FEATURE EXTRACTOR                     ║
║        Convert JSON recordings to ML-ready features            ║
╚════════════════════════════════════════════════════════════════╝
""")
    
    # Configuration
    INTENTIONAL_DIR = "intentional"
    UNINTENTIONAL_DIR = "unintentional"
    OUTPUT_FILE = "training_features.csv"
    
    # Window parameters
    WINDOW_LENGTH = 1.0  # seconds
    WINDOW_STRIDE = 0.5  # seconds
    
    # Initialize extractor
    extractor = LeapMotionFeatureExtractor(
        window_length_sec=WINDOW_LENGTH,
        window_stride_sec=WINDOW_STRIDE
    )
    
    # Process all files
    df = extractor.process_directory(INTENTIONAL_DIR, UNINTENTIONAL_DIR)
    
    # Save to CSV
    df.to_csv(OUTPUT_FILE, index=False)
    print(f"\n✅ Features saved to: {OUTPUT_FILE}")
    print(f"   File size: {os.path.getsize(OUTPUT_FILE) / 1024:.1f} KB")
    
    # Show sample of features
    print(f"\nSample features (first 3 windows):")
    print(df.head(3).to_string())
    
    print("\nFeature columns:")
    feature_cols = [col for col in df.columns if col not in ['label', 'source_file']]
    for i, col in enumerate(feature_cols, 1):
        print(f"  {i:2d}. {col}")

if __name__ == "__main__":
    main()