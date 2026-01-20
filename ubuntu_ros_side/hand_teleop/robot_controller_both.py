#!/usr/bin/env python3
"""
ROS2 Dual Hand-Robot Mirror Controller
Control TWO Niryo robots simultaneously with left and right hands
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

class DualHandRobotController(Node):
    def __init__(self):
        super().__init__('dual_hand_robot_controller')
        
        # ========== RIGHT HAND / ROBOT 1 (Three) ==========
        self.right_control_state = ControlState.IDLE
        self.right_reference_position = None
        self.right_reference_orientation = None
        self.right_reference_joints = None
        self.right_home_complete = False
        
        # Right hand subscribers
        self.right_palm_sub = self.create_subscription(
            PoseStamped, '/hand_tracking/right_palm', 
            self.right_palm_callback, 10)
        self.right_grab_sub = self.create_subscription(
            Float32, '/hand_tracking/right_grab_strength',
            self.right_grab_callback, 10)
        self.right_gesture_sub = self.create_subscription(
            String, '/hand_tracking/right_gesture',
            self.right_gesture_callback, 10)
        
        # Robot 1 action client
        self.robot1_action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/Three/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Right hand data storage
        self.right_palm = None
        self.right_grab = 0.0
        self.right_gesture = "none"
        self.right_last_command_time = 0
        
        # Right hand smoothing
        self.right_position_history = []
        self.right_orientation_history = []
        
        # Right robot state
        self.right_current_joints = [0.0, 0.0, -0.8, 0.0, 0.5, 0.0]
        self.right_last_wrist_roll = 0.0
        self.right_last_wrist_pitch = 0.0
        
        # ========== LEFT HAND / ROBOT 2 (Four) ==========
        self.left_control_state = ControlState.IDLE
        self.left_reference_position = None
        self.left_reference_orientation = None
        self.left_reference_joints = None
        self.left_home_complete = False
        
        # Left hand subscribers
        self.left_palm_sub = self.create_subscription(
            PoseStamped, '/hand_tracking/left_palm', 
            self.left_palm_callback, 10)
        self.left_grab_sub = self.create_subscription(
            Float32, '/hand_tracking/left_grab_strength',
            self.left_grab_callback, 10)
        self.left_gesture_sub = self.create_subscription(
            String, '/hand_tracking/left_gesture',
            self.left_gesture_callback, 10)
        
        # Robot 2 action client
        self.robot2_action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/Four/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Left hand data storage
        self.left_palm = None
        self.left_grab = 0.0
        self.left_gesture = "none"
        self.left_last_command_time = 0
        
        # Left hand smoothing
        self.left_position_history = []
        self.left_orientation_history = []
        
        # Left robot state
        self.left_current_joints = [0.0, 0.0, -0.8, 0.0, 0.5, 0.0]
        self.left_last_wrist_roll = 0.0
        self.left_last_wrist_pitch = 0.0
        
        # ========== SHARED PARAMETERS ==========
        self.command_rate_limit = 0.04  # 25Hz updates
        self.movement_threshold = 0.012
        self.history_length = 2
        
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
        
        # Joint limits
        self.joint_limits = {
            0: (-1.57, 1.57),
            1: (-1.57, 1.2),
            2: (-1.57, 0.6),
            3: (-3.14, 3.14),
            4: (-1.57, 1.57),
            5: (-3.14, 3.14),
        }
        
        # Wrist control
        self.orientation_deadband = 0.1
        
        self.get_logger().info("Dual Hand-Robot Controller initialized")
        self.get_logger().info("  - Robot 1 (Three): RIGHT HAND control")
        self.get_logger().info("  - Robot 2 (Four): LEFT HAND control")
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.04, self.control_loop)
    
    # ========== RIGHT HAND CALLBACKS ==========
    def right_palm_callback(self, msg: PoseStamped):
        self.right_palm = msg
        
    def right_grab_callback(self, msg: Float32):
        self.right_grab = msg.data
        
    def right_gesture_callback(self, msg: String):
        new_gesture = msg.data
        if new_gesture != self.right_gesture:
            self.right_gesture = new_gesture
            self.handle_right_gesture_change(new_gesture)
    
    def handle_right_gesture_change(self, gesture: str):
        if gesture == "pointing_confirmed" and self.right_home_complete:
            self.start_right_mirroring()
        elif gesture == "peace_confirmed":
            self.stop_right_mirroring()
    
    # ========== LEFT HAND CALLBACKS ==========
    def left_palm_callback(self, msg: PoseStamped):
        self.left_palm = msg
        
    def left_grab_callback(self, msg: Float32):
        self.left_grab = msg.data
        
    def left_gesture_callback(self, msg: String):
        new_gesture = msg.data
        if new_gesture != self.left_gesture:
            self.left_gesture = new_gesture
            self.handle_left_gesture_change(new_gesture)
    
    def handle_left_gesture_change(self, gesture: str):
        if gesture == "pointing_confirmed" and self.left_home_complete:
            self.start_left_mirroring()
        elif gesture == "peace_confirmed":
            self.stop_left_mirroring()
    
    # ========== RIGHT ROBOT CONTROL ==========
    def start_right_mirroring(self):
        if self.right_palm is None:
            self.get_logger().warn("Cannot start right mirroring - no hand data")
            return
            
        self.right_control_state = ControlState.MIRRORING
        
        self.right_reference_position = np.array([
            self.right_palm.pose.position.x,
            self.right_palm.pose.position.y,
            self.right_palm.pose.position.z
        ])
        
        self.right_reference_orientation = np.array([
            self.right_palm.pose.orientation.x,
            self.right_palm.pose.orientation.y,
            self.right_palm.pose.orientation.z,
            self.right_palm.pose.orientation.w
        ])
        
        self.right_reference_joints = np.array(self.right_current_joints)
        
        ref_roll, ref_pitch, _ = self.quaternion_to_euler(self.right_reference_orientation)
        self.right_last_wrist_roll = ref_roll
        self.right_last_wrist_pitch = ref_pitch
        
        self.right_position_history = []
        self.right_orientation_history = []
        
        self.get_logger().info("RIGHT HAND → Robot 1 (Three) MIRRORING STARTED")
    
    def stop_right_mirroring(self):
        self.right_control_state = ControlState.IDLE
        self.right_reference_position = None
        self.right_reference_orientation = None
        self.right_reference_joints = None
        self.right_position_history = []
        self.right_orientation_history = []
        
        self.get_logger().info("RIGHT HAND → Robot 1 (Three) MIRRORING STOPPED")
    
    # ========== LEFT ROBOT CONTROL ==========
    def start_left_mirroring(self):
        if self.left_palm is None:
            self.get_logger().warn("Cannot start left mirroring - no hand data")
            return
            
        self.left_control_state = ControlState.MIRRORING
        
        self.left_reference_position = np.array([
            self.left_palm.pose.position.x,
            self.left_palm.pose.position.y,
            self.left_palm.pose.position.z
        ])
        
        self.left_reference_orientation = np.array([
            self.left_palm.pose.orientation.x,
            self.left_palm.pose.orientation.y,
            self.left_palm.pose.orientation.z,
            self.left_palm.pose.orientation.w
        ])
        
        self.left_reference_joints = np.array(self.left_current_joints)
        
        ref_roll, ref_pitch, _ = self.quaternion_to_euler(self.left_reference_orientation)
        self.left_last_wrist_roll = ref_roll
        self.left_last_wrist_pitch = ref_pitch
        
        self.left_position_history = []
        self.left_orientation_history = []
        
        self.get_logger().info("LEFT HAND → Robot 2 (Four) MIRRORING STARTED")
    
    def stop_left_mirroring(self):
        self.left_control_state = ControlState.IDLE
        self.left_reference_position = None
        self.left_reference_orientation = None
        self.left_reference_joints = None
        self.left_position_history = []
        self.left_orientation_history = []
        
        self.get_logger().info("LEFT HAND → Robot 2 (Four) MIRRORING STOPPED")
    
    # ========== SHARED UTILITY FUNCTIONS ==========
    def quaternion_to_euler(self, quat):
        """Convert quaternion to roll, pitch, yaw"""
        x, y, z, w = quat
        
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm > 0:
            x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sin_pitch = 2 * (w * y - z * x)
        sin_pitch = np.clip(sin_pitch, -1, 1)
        pitch = np.arcsin(sin_pitch)
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        
        return roll, pitch, yaw
    
    def smooth_data(self, new_position, new_orientation, history_pos, history_orient):
        """Apply temporal smoothing"""
        history_pos.append(new_position.copy())
        history_orient.append(new_orientation.copy())
        
        if len(history_pos) > self.history_length:
            history_pos.pop(0)
            history_orient.pop(0)
        
        smoothed_pos = np.mean(history_pos, axis=0)
        smoothed_orient = np.mean(history_orient, axis=0)
        
        return smoothed_pos, smoothed_orient
    
    def map_hand_to_joints(self, position_delta, orientation_delta, reference_joints, last_roll, last_pitch):
        """Map hand movements to joint angles"""
        target_joints = reference_joints.copy()
        
        dx = position_delta[0]
        dy = position_delta[1]
        dz = position_delta[2]
        
        # Base rotation
        base_delta = -dx * self.scale_factors['left_right']
        target_joints[0] += base_delta
        
        # Forward/back
        if abs(dz) > 0.005:
            shoulder_delta = dz * self.scale_factors['forward_back'] * 0.7
            elbow_delta = -dz * self.scale_factors['forward_back'] * 0.4
            target_joints[1] += shoulder_delta
            target_joints[2] += elbow_delta
        
        # Up/down
        if abs(dy) > 0.005:
            elbow_delta = dy * self.scale_factors['up_down'] * 0.8
            shoulder_delta = dy * self.scale_factors['up_down'] * 0.3
            target_joints[2] += elbow_delta
            target_joints[1] += shoulder_delta
        
        # Wrist orientation
        ref_roll, ref_pitch, _ = self.quaternion_to_euler(orientation_delta)
        
        roll_delta = ref_roll - last_roll
        pitch_delta = ref_pitch - last_pitch
        
        if abs(roll_delta) > self.orientation_deadband:
            roll_change = roll_delta * self.scale_factors['wrist_roll']
            roll_change = np.clip(roll_change, -0.1, 0.1)
            target_joints[3] += roll_change
        
        if abs(pitch_delta) > self.orientation_deadband:
            pitch_change = pitch_delta * self.scale_factors['wrist_pitch']
            pitch_change = np.clip(pitch_change, -0.15, 0.15)
            target_joints[4] += pitch_change
        
        # Apply joint limits
        for i, (min_val, max_val) in self.joint_limits.items():
            target_joints[i] = np.clip(target_joints[i], min_val, max_val)
        
        return target_joints
    
    def send_robot_command(self, action_client, joint_angles: List[float], duration: float = 0.08):
        """Send trajectory command to robot"""
        if not action_client.wait_for_server(timeout_sec=0.05):
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
        
        action_client.send_goal_async(goal)
        return True
    
    # ========== MAIN CONTROL LOOP ==========
    def control_loop(self):
        """Main control loop for both robots"""
        current_time = time.time()
        
        # Control right robot
        if self.right_control_state == ControlState.MIRRORING:
            if current_time - self.right_last_command_time >= self.command_rate_limit:
                self.control_right_robot(current_time)
        
        # Control left robot
        if self.left_control_state == ControlState.MIRRORING:
            if current_time - self.left_last_command_time >= self.command_rate_limit:
                self.control_left_robot(current_time)
    
    def control_right_robot(self, current_time):
        """Control right robot with right hand"""
        if (self.right_palm is None or self.right_reference_position is None or 
            self.right_reference_orientation is None):
            return
        
        hand_age = current_time - (self.right_palm.header.stamp.sec + 
                                  self.right_palm.header.stamp.nanosec * 1e-9)
        if hand_age > 0.15:
            return
        
        try:
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
            
            position_delta = current_position - self.right_reference_position
            
            smoothed_pos_delta, smoothed_orient = self.smooth_data(
                position_delta, current_orientation,
                self.right_position_history, self.right_orientation_history)
            
            if np.linalg.norm(smoothed_pos_delta) < self.movement_threshold:
                return
            
            target_joints = self.map_hand_to_joints(
                smoothed_pos_delta, smoothed_orient, 
                self.right_reference_joints,
                self.right_last_wrist_roll, self.right_last_wrist_pitch)
            
            joint_diff = np.array(target_joints) - np.array(self.right_current_joints)
            if np.linalg.norm(joint_diff) < 0.02:
                return
            
            if self.send_robot_command(self.robot1_action_client, target_joints):
                self.right_current_joints = target_joints.copy()
                self.right_last_command_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Right robot control error: {e}")
    
    def control_left_robot(self, current_time):
        """Control left robot with left hand"""
        if (self.left_palm is None or self.left_reference_position is None or 
            self.left_reference_orientation is None):
            return
        
        hand_age = current_time - (self.left_palm.header.stamp.sec + 
                                  self.left_palm.header.stamp.nanosec * 1e-9)
        if hand_age > 0.15:
            return
        
        try:
            current_position = np.array([
                self.left_palm.pose.position.x,
                self.left_palm.pose.position.y,
                self.left_palm.pose.position.z
            ])
            
            current_orientation = np.array([
                self.left_palm.pose.orientation.x,
                self.left_palm.pose.orientation.y,
                self.left_palm.pose.orientation.z,
                self.left_palm.pose.orientation.w
            ])
            
            position_delta = current_position - self.left_reference_position
            
            smoothed_pos_delta, smoothed_orient = self.smooth_data(
                position_delta, current_orientation,
                self.left_position_history, self.left_orientation_history)
            
            if np.linalg.norm(smoothed_pos_delta) < self.movement_threshold:
                return
            
            target_joints = self.map_hand_to_joints(
                smoothed_pos_delta, smoothed_orient,
                self.left_reference_joints,
                self.left_last_wrist_roll, self.left_last_wrist_pitch)
            
            joint_diff = np.array(target_joints) - np.array(self.left_current_joints)
            if np.linalg.norm(joint_diff) < 0.02:
                return
            
            if self.send_robot_command(self.robot2_action_client, target_joints):
                self.left_current_joints = target_joints.copy()
                self.left_last_command_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Left robot control error: {e}")
    
    # ========== HOME POSITION ==========
    def go_home_both(self):
        """Move both robots to home position"""
        self.get_logger().info("Moving both robots to home position...")
        
        # Move right robot
        self.right_control_state = ControlState.HOMING
        if self.send_robot_command(self.robot1_action_client, self.home_joints, duration=3.0):
            self.get_logger().info("Robot 1 (Three) homing...")
        
        # Move left robot
        self.left_control_state = ControlState.HOMING
        if self.send_robot_command(self.robot2_action_client, self.home_joints, duration=3.0):
            self.get_logger().info("Robot 2 (Four) homing...")
        
        # Wait for both to complete
        time.sleep(3.5)
        
        self.right_home_complete = True
        self.left_home_complete = True
        self.right_control_state = ControlState.IDLE
        self.left_control_state = ControlState.IDLE
        
        self.get_logger().info("Both robots at home - ready for gesture control")

def main(args=None):
    print("""
╔══════════════════════════════════════════════════════════════╗
║          DUAL HAND-ROBOT CONTROLLER                      ║
║      Control 2 robots with left and right hands          ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    rclpy.init(args=args)
    
    try:
        controller = DualHandRobotController()
        
        print("DUAL ROBOT CONTROL:")
        print("   RIGHT HAND → Robot 1 (Three)")
        print("   LEFT HAND  → Robot 2 (Four)")
        print()
        print("CONTROLS:")
        print("   Right hand pointing 2s → START Robot 1")
        print("   Left hand pointing 2s  → START Robot 2")
        print("   Right hand peace 2s    → STOP Robot 1")
        print("   Left hand peace 2s     → STOP Robot 2")
        print()
        print("Moving both robots to home position...")
        print()
        
        time.sleep(1)
        controller.go_home_both()
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("Shutting down dual controller...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
