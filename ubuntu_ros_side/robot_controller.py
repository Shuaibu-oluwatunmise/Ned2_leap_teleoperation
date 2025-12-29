#!/usr/bin/env python3
"""
ROS2 Hand-to-Robot Controller Node
Maps hand tracking data to Niryo Ned2 robot arm movements
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Header
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
import numpy as np
import time
from typing import Optional, List

class HandRobotController(Node):
    def __init__(self):
        super().__init__('hand_robot_controller')
        
        # Hand tracking subscribers
        self.left_palm_sub = self.create_subscription(
            PoseStamped, '/hand_tracking/left_palm', 
            self.left_palm_callback, 10)
        self.right_palm_sub = self.create_subscription(
            PoseStamped, '/hand_tracking/right_palm', 
            self.right_palm_callback, 10)
        self.left_grab_sub = self.create_subscription(
            Float32, '/hand_tracking/left_grab_strength',
            self.left_grab_callback, 10)
        self.right_grab_sub = self.create_subscription(
            Float32, '/hand_tracking/right_grab_strength',
            self.right_grab_callback, 10)
        
        # Robot action client
        self.robot_action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/Three/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Hand data storage
        self.left_palm = None
        self.right_palm = None
        self.left_grab = 0.0
        self.right_grab = 0.0
        self.last_command_time = 0
        
        # Control parameters
        self.command_rate_limit = 0.1
        self.workspace_scale = 2.0
        self.safety_limits = {
            'x': (-0.3, 0.3),
            'y': (-0.3, 0.3),
            'z': (0.1, 0.4)
        }
        
        # Robot home position
        self.home_joints = [0.0, 0.3, -0.3, 0.0, 0.0, 0.0]
        self.current_joints = self.home_joints.copy()
        
        self.get_logger().info("Hand-Robot Controller initialized")
        self.get_logger().info("Use your RIGHT HAND to control the robot")
        
        # Timer for periodic robot updates
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
    def left_palm_callback(self, msg: PoseStamped):
        self.left_palm = msg
        
    def right_palm_callback(self, msg: PoseStamped):
        self.right_palm = msg
        
    def left_grab_callback(self, msg: Float32):
        self.left_grab = msg.data
        
    def right_grab_callback(self, msg: Float32):
        self.right_grab = msg.data
    
    def hand_to_robot_coordinates(self, hand_pos) -> tuple:
        """Convert hand coordinates to robot workspace coordinates"""
        robot_x = hand_pos.x * self.workspace_scale - 0.2
        robot_y = hand_pos.y * self.workspace_scale - 0.1
        robot_z = hand_pos.z * self.workspace_scale + 0.2
        
        robot_x = np.clip(robot_x, *self.safety_limits['x'])
        robot_y = np.clip(robot_y, *self.safety_limits['y'])
        robot_z = np.clip(robot_z, *self.safety_limits['z'])
        
        return robot_x, robot_y, robot_z
    
    def position_to_joint_angles(self, x: float, y: float, z: float) -> List[float]:
        """Simplified inverse kinematics"""
        joint_1 = np.arctan2(y, x)
        
        reach = np.sqrt(x*x + y*y)
        joint_2 = np.clip(reach * 2.0 - 0.3, -1.5, 1.5)
        joint_3 = np.clip(-reach * 1.5 + z * 2.0, -1.5, 0.5)
        
        joint_4 = 0.0
        joint_5 = -joint_2 - joint_3
        joint_6 = 0.0
        
        return [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    
    def send_robot_command(self, joint_angles: List[float], duration: float = 0.5):
        """Send trajectory command to robot"""
        if not self.robot_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Robot action server not available")
            return
        
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
        
    def control_loop(self):
        """Main control loop - runs at 20Hz"""
        current_time = time.time()
        
        if current_time - self.last_command_time < self.command_rate_limit:
            return
        
        if self.right_palm is None:
            return
        
        hand_age = current_time - (self.right_palm.header.stamp.sec + 
                                  self.right_palm.header.stamp.nanosec * 1e-9)
        if hand_age > 0.2:
            self.get_logger().warn("Hand tracking data is stale")
            return
        
        try:
            robot_x, robot_y, robot_z = self.hand_to_robot_coordinates(
                self.right_palm.pose.position
            )
            
            target_joints = self.position_to_joint_angles(robot_x, robot_y, robot_z)
            
            joint_diff = np.array(target_joints) - np.array(self.current_joints)
            if np.linalg.norm(joint_diff) > 0.05:
                
                self.send_robot_command(target_joints, duration=0.2)
                self.last_command_time = current_time
                
                self.get_logger().info(
                    f"Moving to: X={robot_x:.3f}, Y={robot_y:.3f}, Z={robot_z:.3f} "
                    f"Grab={self.right_grab:.2f}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
    
    def go_home(self):
        """Move robot to home position"""
        self.get_logger().info("Moving to home position")
        self.send_robot_command(self.home_joints, duration=2.0)

def main(args=None):
    print("""
╔══════════════════════════════════════════════════════════════╗
║               HAND-TO-ROBOT CONTROLLER NODE                  ║
║           Control Ned2 robot with hand movements            ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    rclpy.init(args=args)
    
    try:
        controller = HandRobotController()
        
        print("🚀 Controller started!")
        print("📋 Instructions:")
        print("   - Move your RIGHT HAND to control the robot")
        print("   - Press Ctrl+C to stop")
        print()
        
        time.sleep(2)
        controller.go_home()
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nℹ️ Shutting down controller...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()