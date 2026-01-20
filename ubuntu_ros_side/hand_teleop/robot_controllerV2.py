#!/usr/bin/env python3
"""
ROS2 Intuitive Hand-Robot Mirror Controller
Fixed version addressing wrist control and workspace limitations
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String, Header
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
import numpy as np
import time
from typing import Optional, List, Dict
from enum import Enum

class ControlState(Enum):
    IDLE = "idle"
    MIRRORING = "mirroring"
    HOMING = "homing"

class IntuitiveHandRobotController(Node):
    def __init__(self):
        super().__init__('intuitive_hand_robot_controller')
        
        # Control state
        self.control_state = ControlState.IDLE
        self.reference_position = None
        self.reference_orientation = None
        self.reference_joints = None
        self.home_complete = False
        
        # Hand tracking subscribers (focus on right hand for control)
        self.right_palm_sub = self.create_subscription(
            PoseStamped, '/hand_tracking/right_palm', 
            self.right_palm_callback, 10)
        self.right_grab_sub = self.create_subscription(
            Float32, '/hand_tracking/right_grab_strength',
            self.right_grab_callback, 10)
        self.right_gesture_sub = self.create_subscription(
            String, '/hand_tracking/right_gesture',
            self.right_gesture_callback, 10)
        
        # Robot action client
        self.robot_action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/Three/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Hand data storage
        self.right_palm = None
        self.right_grab = 0.0
        self.current_gesture = "none"
        self.last_command_time = 0
        
        # Control parameters
        self.command_rate_limit = 0.04  # 25Hz updates
        self.movement_threshold = 0.012
        
        # Movement scaling factors
        self.scale_factors = {
            'left_right': 2.8,
            'forward_back': 2.2,
            'up_down': 2.5,
            'wrist_roll': 0.4,
            'wrist_pitch': 1.2
        }
        
        # Home position
        self.home_joints = [0.0, 0.0, -0.8, 0.0, 0.5, 0.0]
        self.current_joints = self.home_joints.copy()
        
        # Extended joint limits
        self.joint_limits = {
            0: (-1.57, 1.57),
            1: (-1.57, 1.2),
            2: (-1.57, 0.6),
            3: (-3.14, 3.14),
            4: (-1.57, 1.57),
            5: (-3.14, 3.14),
        }
        
        # Wrist control improvements
        self.orientation_deadband = 0.1
        self.last_wrist_roll = 0.0
        self.last_wrist_pitch = 0.0
        
        # Movement smoothing
        self.position_history = []
        self.orientation_history = []
        self.history_length = 2
        
        self.get_logger().info("Enhanced Hand-Robot Controller initialized")
        self.get_logger().info("Fixes applied:")
        self.get_logger().info("  - Reduced wrist roll sensitivity (10x less)")
        self.get_logger().info("  - Extended workspace limits")
        self.get_logger().info("  - Lower home position for table reach")
        self.get_logger().info("  - Base rotation: ±90 degrees")
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.04, self.control_loop)
        
    def right_palm_callback(self, msg: PoseStamped):
        self.right_palm = msg
        
    def right_grab_callback(self, msg: Float32):
        self.right_grab = msg.data
        
    def right_gesture_callback(self, msg: String):
        new_gesture = msg.data
        if new_gesture != self.current_gesture:
            self.current_gesture = new_gesture
            self.handle_gesture_change(new_gesture)
    
    def handle_gesture_change(self, gesture: str):
        if gesture == "pointing_confirmed" and self.home_complete:
            self.start_mirroring_mode()
        elif gesture == "peace_confirmed":
            self.stop_mirroring_mode()
    
    def start_mirroring_mode(self):
        if self.right_palm is None:
            self.get_logger().warn("Cannot start mirroring - no hand data")
            return
            
        self.control_state = ControlState.MIRRORING
        
        # Capture reference hand position
        self.reference_position = np.array([
            self.right_palm.pose.position.x,
            self.right_palm.pose.position.y,
            self.right_palm.pose.position.z
        ])
        
        # Capture reference hand orientation (quaternion)
        self.reference_orientation = np.array([
            self.right_palm.pose.orientation.x,
            self.right_palm.pose.orientation.y,
            self.right_palm.pose.orientation.z,
            self.right_palm.pose.orientation.w
        ])
        
        # Store current joint positions as reference
        self.reference_joints = np.array(self.current_joints)
        
        # Reset wrist tracking
        ref_roll, ref_pitch, _ = self.quaternion_to_euler(self.reference_orientation)
        self.last_wrist_roll = ref_roll
        self.last_wrist_pitch = ref_pitch
        
        # Clear history for fresh start
        self.position_history = []
        self.orientation_history = []
        
        self.get_logger().info("MIRRORING STARTED - Natural hand control active")
        self.get_logger().info(f"Reference position: [{self.reference_position[0]:.3f}, {self.reference_position[1]:.3f}, {self.reference_position[2]:.3f}]")
    
    def stop_mirroring_mode(self):
        self.control_state = ControlState.IDLE
        self.reference_position = None
        self.reference_orientation = None
        self.reference_joints = None
        self.position_history = []
        self.orientation_history = []
        
        self.get_logger().info("MIRRORING STOPPED")
    
    def quaternion_to_euler(self, quat):
        """Convert quaternion to roll, pitch, yaw with numerical stability"""
        x, y, z, w = quat
        
        # Normalize quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Roll (rotation about x-axis)
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        
        # Pitch (rotation about y-axis)
        sin_pitch = 2 * (w * y - z * x)
        sin_pitch = np.clip(sin_pitch, -1, 1)
        pitch = np.arcsin(sin_pitch)
        
        # Yaw (rotation about z-axis)
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        
        return roll, pitch, yaw
    
    def smooth_data(self, new_position, new_orientation):
        """Apply temporal smoothing to reduce jitter"""
        self.position_history.append(new_position.copy())
        self.orientation_history.append(new_orientation.copy())
        
        # Keep only recent history
        if len(self.position_history) > self.history_length:
            self.position_history.pop(0)
            self.orientation_history.pop(0)
        
        # Simple averaging
        smoothed_pos = np.mean(self.position_history, axis=0)
        smoothed_orient = np.mean(self.orientation_history, axis=0)
        
        return smoothed_pos, smoothed_orient
    
    def map_hand_to_joints(self, position_delta, orientation_delta):
        """Improved hand-to-joint mapping with wrist control fixes"""
        
        # Start with reference joint positions
        target_joints = self.reference_joints.copy()
        
        # Extract position deltas
        dx = position_delta[0]  # Left/right
        dy = position_delta[1]  # Up/down
        dz = position_delta[2]  # Forward/back
        
        # 1. Base rotation (Joint 1): Hand left/right movement
        base_delta = -dx * self.scale_factors['left_right']
        target_joints[0] += base_delta
        
        # 2. Forward/back movement: Shoulder + Elbow coordination
        if abs(dz) > 0.005:
            shoulder_delta = dz * self.scale_factors['forward_back'] * 0.7
            elbow_delta = -dz * self.scale_factors['forward_back'] * 0.4
            
            target_joints[1] += shoulder_delta
            target_joints[2] += elbow_delta
        
        # 3. Up/down movement: Elbow-dominant with shoulder compensation
        if abs(dy) > 0.005:
            elbow_delta = dy * self.scale_factors['up_down'] * 0.8
            shoulder_delta = dy * self.scale_factors['up_down'] * 0.3
            
            target_joints[2] += elbow_delta
            target_joints[1] += shoulder_delta
        
        # 4. Wrist orientation control
        ref_roll, ref_pitch, _ = self.quaternion_to_euler(self.reference_orientation)
        curr_roll, curr_pitch, _ = self.quaternion_to_euler(orientation_delta)
        
        # Calculate deltas
        roll_delta = curr_roll - ref_roll
        pitch_delta = curr_pitch - ref_pitch
        
        # Apply deadband to reduce noise
        if abs(roll_delta) > self.orientation_deadband:
            roll_change = roll_delta * self.scale_factors['wrist_roll']
            roll_change = np.clip(roll_change, -0.1, 0.1)
            target_joints[3] += roll_change
            self.last_wrist_roll = curr_roll
        
        if abs(pitch_delta) > self.orientation_deadband:
            pitch_change = pitch_delta * self.scale_factors['wrist_pitch']
            pitch_change = np.clip(pitch_change, -0.15, 0.15)
            target_joints[4] += pitch_change
            self.last_wrist_pitch = curr_pitch
        
        # Apply joint limits
        for i, (min_val, max_val) in self.joint_limits.items():
            target_joints[i] = np.clip(target_joints[i], min_val, max_val)
        
        return target_joints
    
    def send_robot_command(self, joint_angles: List[float], duration: float = 0.08):
        """Send trajectory command to robot"""
        if not self.robot_action_client.wait_for_server(timeout_sec=0.05):
            return False
        
        # Create trajectory message
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Send command
        future = self.robot_action_client.send_goal_async(goal)
        self.current_joints = joint_angles.copy()
        return True
        
    def control_loop(self):
        """Main control loop"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_command_time < self.command_rate_limit:
            return
        
        # Only process if in mirroring mode
        if self.control_state != ControlState.MIRRORING:
            return
        
        # Check for valid data
        if (self.right_palm is None or self.reference_position is None or 
            self.reference_orientation is None):
            return
        
        # Check data freshness
        hand_age = current_time - (self.right_palm.header.stamp.sec + 
                                  self.right_palm.header.stamp.nanosec * 1e-9)
        if hand_age > 0.15:
            return
        
        try:
            # Get current hand data
            current_position = np.array([
                self.right_palm.pose.position.x,
                self.right_palm.pose.position.y,
                self.right_palm.pose.position.z
            ])
            
            current_orientation = np.array([
                self.right_palm.pose.orientation.x,
                self.right_palm.pose.orientation.y,
                self.right_palm.pose.orientation.z,
                self.right_palm.pose.orientation.w
            ])
            
            # Calculate deltas from reference
            position_delta = current_position - self.reference_position
            
            # Apply smoothing
            smoothed_pos_delta, smoothed_orient = self.smooth_data(
                position_delta, current_orientation)
            
            # Check if movement is significant
            movement_magnitude = np.linalg.norm(smoothed_pos_delta)
            if movement_magnitude < self.movement_threshold:
                return
            
            # Map hand movements to joint angles
            target_joints = self.map_hand_to_joints(smoothed_pos_delta, smoothed_orient)
            
            # Check if robot movement is significant
            joint_diff = np.array(target_joints) - np.array(self.current_joints)
            if np.linalg.norm(joint_diff) < 0.02:
                return
            
            # Send command to robot
            if self.send_robot_command(target_joints, duration=0.08):
                self.last_command_time = current_time
                
                # Log movement info periodically
                if current_time % 1.0 < 0.04:
                    self.get_logger().info(
                        f"Hand: L/R={smoothed_pos_delta[0]:.3f} U/D={smoothed_pos_delta[1]:.3f} F/B={smoothed_pos_delta[2]:.3f} "
                        f"-> Base={joint_diff[0]:.2f} Shldr={joint_diff[1]:.2f} Elbow={joint_diff[2]:.2f} "
                        f"WRoll={joint_diff[3]:.2f} WPitch={joint_diff[4]:.2f}"
                    )
                
        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
    
    def go_home(self):
        """Move robot to home position and wait for completion"""
        self.get_logger().info("Moving to home position - robot will go lower for table reach")
        self.control_state = ControlState.HOMING
        
        if self.send_robot_command(self.home_joints, duration=3.0):
            # Wait for home movement to complete
            time.sleep(3.5)
            self.home_complete = True
            self.control_state = ControlState.IDLE
            self.get_logger().info("Home position reached - ready for gesture activation")
        else:
            self.get_logger().error("Failed to send home command")

def main(args=None):
    print("""
╔══════════════════════════════════════════════════════════════╗
║         FIXED INTUITIVE HAND-ROBOT CONTROLLER               ║
║     Better workspace + Improved wrist control               ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    rclpy.init(args=args)
    
    try:
        controller = IntuitiveHandRobotController()
        
        print("FIXES APPLIED:")
        print("   - Wrist roll sensitivity reduced 10x")
        print("   - Extended joint limits for full workspace")  
        print("   - Lower home position for table reach")
        print("   - Base rotation: proper ±90° range")
        print("   - Orientation deadbands to reduce jitter")
        print()
        print("CONTROLS:")
        print("   Left/Right hand    -> Base rotation (±90°)")
        print("   Forward/Back hand  -> Shoulder+Elbow reach")
        print("   Up/Down hand       -> Elbow+Shoulder height")
        print("   Wrist rotation     -> Wrist roll (much gentler)")
        print("   Wrist tilt         -> Wrist pitch")
        print()
        print("Point for 2s to START, Peace sign for 2s to STOP")
        print("Robot is moving to home position...")
        print()
        
        # Actually move to home position and wait
        time.sleep(1)
        controller.go_home()
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("Shutting down controller...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()