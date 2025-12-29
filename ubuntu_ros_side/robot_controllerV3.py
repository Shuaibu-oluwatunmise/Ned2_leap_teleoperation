#!/usr/bin/env python3
"""
ROS2 Optimal Hand-Robot Controller
Best of all versions: Old system smoothness + Intuitive mapping + Gesture activation
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

class OptimalHandRobotController(Node):
    def __init__(self):
        super().__init__('optimal_hand_robot_controller')
        
        # Control state
        self.control_state = ControlState.IDLE
        self.reference_position = None
        self.reference_orientation = None
        self.reference_joints = None
        self.home_complete = False
        
        # Hand tracking subscribers
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
        self.command_rate_limit = 0.1
        self.movement_threshold = 0.012
        
        # Movement scaling factors
        self.scale_factors = {
            'left_right': 3.2,
            'forward_back': 2.0,
            'up_down': 1.8,
            'wrist_roll': 0.8,
            'wrist_pitch': 3.0
        }
        
        # Home position
        self.home_joints = [0.0, 0.0, -0.8, 0.0, 0.5, 0.0]
        self.current_joints = self.home_joints.copy()
        
        # Joint limits
        self.joint_limits = {
            0: (-1.57, 1.57),
            1: (-1.57, 1.2),
            2: (-1.57, 0.6),
            3: (-3.14, 3.14),
            4: (-1.57, 1.57),
            5: (-3.14, 3.14),
        }
        
        # Smoothing
        self.position_history = []
        self.orientation_history = []
        self.history_length = 2
        
        self.get_logger().info("Optimal Hand-Robot Controller - Best of All Versions")
        self.get_logger().info("Features combined:")
        self.get_logger().info("  - Old system smoothness (100ms updates, 0.4s duration)")
        self.get_logger().info("  - Intuitive joint mapping")
        self.get_logger().info("  - Gesture activation")
        self.get_logger().info("  - Extended workspace")
        
        # Timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
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
        
        # Capture reference hand orientation
        self.reference_orientation = np.array([
            self.right_palm.pose.orientation.x,
            self.right_palm.pose.orientation.y,
            self.right_palm.pose.orientation.z,
            self.right_palm.pose.orientation.w
        ])
        
        # Store current joint positions as reference
        self.reference_joints = np.array(self.current_joints)
        
        # Clear history
        self.position_history = []
        self.orientation_history = []
        
        self.get_logger().info("MIRRORING STARTED - Optimal control active")
        self.get_logger().info(f"Reference: [{self.reference_position[0]:.3f}, {self.reference_position[1]:.3f}, {self.reference_position[2]:.3f}]")
    
    def stop_mirroring_mode(self):
        self.control_state = ControlState.IDLE
        self.reference_position = None
        self.reference_orientation = None
        self.reference_joints = None
        self.position_history = []
        self.orientation_history = []
        
        self.get_logger().info("MIRRORING STOPPED")
    
    def quaternion_to_euler(self, quat):
        """Convert quaternion to roll, pitch, yaw"""
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
        """Minimal smoothing"""
        self.position_history.append(new_position.copy())
        self.orientation_history.append(new_orientation.copy())
        
        if len(self.position_history) > self.history_length:
            self.position_history.pop(0)
            self.orientation_history.pop(0)
        
        # Simple averaging
        smoothed_pos = np.mean(self.position_history, axis=0)
        smoothed_orient = np.mean(self.orientation_history, axis=0)
        
        return smoothed_pos, smoothed_orient
    
    def map_hand_to_joints(self, position_delta, orientation_delta):
        """Intuitive mapping"""
        target_joints = self.reference_joints.copy()
        
        # Extract position deltas
        dx = position_delta[0]  # Left/right
        dy = position_delta[1]  # Up/down
        dz = position_delta[2]  # Forward/back
        
        # 1. Base rotation: Hand left/right -> Base rotation
        base_delta = -dx * self.scale_factors['left_right']
        target_joints[0] += base_delta
        
        # 2. Forward/back: Shoulder + Elbow coordination
        if abs(dz) > 0.005:
            shoulder_delta = dz * self.scale_factors['forward_back'] * 0.7
            elbow_delta = -dz * self.scale_factors['forward_back'] * 0.4
            target_joints[1] += shoulder_delta
            target_joints[2] += elbow_delta
        
        # 3. Up/down: Elbow-dominant with shoulder compensation
        if abs(dy) > 0.005:
            elbow_delta = dy * self.scale_factors['up_down'] * 0.8
            shoulder_delta = dy * self.scale_factors['up_down'] * 0.3
            target_joints[2] += elbow_delta
            target_joints[1] += shoulder_delta
        
        # 4. Wrist control with position-aware filtering
        ref_roll, ref_pitch, _ = self.quaternion_to_euler(self.reference_orientation)
        curr_roll, curr_pitch, _ = self.quaternion_to_euler(orientation_delta)
        
        # Calculate position displacement
        position_displacement = np.linalg.norm([dx, dy, dz])
        
        # Dynamic deadband scaling
        base_roll_deadband = 0.15
        base_pitch_deadband = 0.05
        
        displacement_factor = min(position_displacement * 8.0, 3.0)
        
        roll_deadband = base_roll_deadband * (1.0 + displacement_factor)
        pitch_deadband = base_pitch_deadband * (1.0 + displacement_factor * 0.5)
        
        # Joint 4 (Wrist roll)
        roll_delta = curr_roll - ref_roll
        if abs(roll_delta) > roll_deadband:
            target_joints[3] += roll_delta * self.scale_factors['wrist_roll']
        
        # Joint 5 (Wrist pitch)
        pitch_delta = curr_pitch - ref_pitch  
        if abs(pitch_delta) > pitch_deadband:
            target_joints[4] += pitch_delta * self.scale_factors['wrist_pitch']
        
        # Apply joint limits
        for i, (min_val, max_val) in self.joint_limits.items():
            target_joints[i] = np.clip(target_joints[i], min_val, max_val)
        
        return target_joints
    
    def send_robot_command(self, joint_angles: List[float], duration: float = 0.4):
        """Send trajectory"""
        if not self.robot_action_client.wait_for_server(timeout_sec=0.1):
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 
            'joint_4', 'joint_5', 'joint_6'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        future = self.robot_action_client.send_goal_async(goal)
        self.current_joints = joint_angles.copy()
        return True
        
    def control_loop(self):
        """Control loop"""
        current_time = time.time()
        
        if current_time - self.last_command_time < self.command_rate_limit:
            return
        
        if self.control_state != ControlState.MIRRORING:
            return
        
        if (self.right_palm is None or self.reference_position is None or 
            self.reference_orientation is None):
            return
        
        hand_age = current_time - (self.right_palm.header.stamp.sec + 
                                  self.right_palm.header.stamp.nanosec * 1e-9)
        if hand_age > 0.2:
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
            
            # Calculate deltas
            position_delta = current_position - self.reference_position
            
            # Minimal smoothing
            smoothed_pos_delta, smoothed_orient = self.smooth_data(
                position_delta, current_orientation)
            
            # Movement threshold check
            movement_magnitude = np.linalg.norm(smoothed_pos_delta)
            if movement_magnitude < self.movement_threshold:
                return
            
            # Intuitive mapping
            target_joints = self.map_hand_to_joints(smoothed_pos_delta, smoothed_orient)
            
            # Movement threshold
            joint_diff = np.array(target_joints) - np.array(self.current_joints)
            if np.linalg.norm(joint_diff) > 0.04:
                
                if self.send_robot_command(target_joints, duration=0.3):
                    self.last_command_time = current_time
                    
                    if current_time % 2.0 < 0.1:
                        self.get_logger().info(
                            f"Hand: L/R={smoothed_pos_delta[0]:.3f} U/D={smoothed_pos_delta[1]:.3f} F/B={smoothed_pos_delta[2]:.3f} "
                            f"-> Joints: [B={joint_diff[0]:.2f}, S={joint_diff[1]:.2f}, E={joint_diff[2]:.2f}, "
                            f"R={joint_diff[3]:.2f}, P={joint_diff[4]:.2f}] Grab={self.right_grab:.2f}"
                        )
                
        except Exception as e:
            self.get_logger().error(f"Control error: {e}")
    
    def go_home(self):
        """Move to home position and wait"""
        self.get_logger().info("Moving to optimal home position")
        self.control_state = ControlState.HOMING
        
        if self.send_robot_command(self.home_joints, duration=3.0):
            time.sleep(3.5)
            self.home_complete = True
            self.control_state = ControlState.IDLE
            self.get_logger().info("Home position reached - gesture control ready")
        else:
            self.get_logger().error("Failed to reach home position")

def main(args=None):
    print("""
╔══════════════════════════════════════════════════════════════╗
║            OPTIMAL HAND-ROBOT CONTROLLER                     ║
║  Old system smoothness + Intuitive mapping + Gestures       ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    rclpy.init(args=args)
    
    try:
        controller = OptimalHandRobotController()
        
        print("OPTIMAL FEATURES:")
        print("   - Smooth 10Hz control (like old system)")
        print("   - 300ms trajectory duration (smooth)")
        print("   - Intuitive joint-by-joint mapping")
        print("   - Improved Joint 5 wrist pitch control")
        print("   - Gesture activation/deactivation")
        print("   - Extended workspace for table reach")
        print()
        print("CONTROLS:")
        print("   Point 2s = START, Peace 2s = STOP")
        print("   Natural hand movements = robot mirrors")
        print("   Wrist tilt down = robot points to table")
        print()
        
        time.sleep(1)
        controller.go_home()
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("Shutting down optimal controller...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()