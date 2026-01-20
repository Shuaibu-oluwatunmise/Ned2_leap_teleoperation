#!/usr/bin/env python3
"""
Leader-Follower Robot Control System
Robot Four (Leader) -> Robot Three (Follower)

The leader robot is moved manually in FreeMotion mode,
and the follower robot mimics the movements in real-time.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from niryo_ned_ros2_interfaces.srv import SetBool
import numpy as np
import sys
import signal
from collections import deque


class LeaderFollowerController(Node):
    def __init__(self):
        super().__init__('leader_follower_controller')
        
        # Robot namespaces
        self.leader_ns = 'Four'
        self.follower_ns = 'Three'
        
        # Control parameters (ULTRA-LOW-LATENCY MODE)
        self.control_rate = 200.0  # Hz - maximum responsiveness
        self.trajectory_duration = 0.03  # seconds - 30ms for near-instant response
        self.smoothing_alpha = 0.7  # EMA smoothing factor (0-1, higher = more responsive)
        
        # State variables
        self.leader_joint_positions = None
        self.smoothed_positions = None
        self.leader_joint_names = None
        self.last_command_time = self.get_clock().now()
        self.is_ready = False
        
        # Create subscriber for leader joint states
        self.leader_sub = self.create_subscription(
            JointState,
            f'/{self.leader_ns}/joint_states',
            self.leader_joint_callback,
            10
        )
        
        # Create action client for follower trajectory control
        self.follower_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.follower_ns}/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Create service clients for learning mode control
        self.leader_learning_mode_client = self.create_client(
            SetBool,
            f'/{self.leader_ns}/niryo_robot/learning_mode/activate'
        )
        
        self.follower_learning_mode_client = self.create_client(
            SetBool,
            f'/{self.follower_ns}/niryo_robot/learning_mode/activate'
        )
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        # Statistics
        self.command_count = 0
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('Leader-Follower Controller initialized')
        self.get_logger().info(f'Leader: Robot {self.leader_ns}')
        self.get_logger().info(f'Follower: Robot {self.follower_ns}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        
    def leader_joint_callback(self, msg):
        """Callback for leader robot joint states"""
        # Store joint positions and names
        self.leader_joint_positions = np.array(msg.position)
        self.leader_joint_names = list(msg.name)
        
        # Initialize smoothed positions on first message
        if self.smoothed_positions is None:
            self.smoothed_positions = self.leader_joint_positions.copy()
            self.get_logger().info(f'Received first joint state from leader: {len(self.leader_joint_names)} joints')
            self.is_ready = True
        else:
            # Apply exponential moving average smoothing
            self.smoothed_positions = (
                self.smoothing_alpha * self.leader_joint_positions +
                (1 - self.smoothing_alpha) * self.smoothed_positions
            )
    
    def control_loop(self):
        """Main control loop - sends follower commands based on leader position"""
        if not self.is_ready or self.leader_joint_positions is None:
            return
        
        # Check if action server is available
        if not self.follower_action_client.server_is_ready():
            if self.command_count == 0:
                self.get_logger().warn('Waiting for follower action server...', throttle_duration_sec=5.0)
            return
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.leader_joint_names
        
        # Create trajectory point with smoothed positions
        point = JointTrajectoryPoint()
        point.positions = self.smoothed_positions.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.trajectory_duration * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        # Send goal (non-blocking)
        self.follower_action_client.send_goal_async(goal_msg)
        self.command_count += 1
        self.last_command_time = self.get_clock().now()
    
    def print_statistics(self):
        """Print control statistics"""
        if self.is_ready:
            self.get_logger().info(
                f'Stats - Commands sent: {self.command_count}, '
                f'Current joints: {len(self.leader_joint_names) if self.leader_joint_names else 0}'
            )
    
    def activate_learning_mode(self, robot_name, client, enable=True):
        """Activate or deactivate learning mode for a robot"""
        action = "Activating" if enable else "Deactivating"
        self.get_logger().info(f'{action} learning mode for {robot_name}...')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Learning mode service not available for {robot_name}')
            return False
        
        request = SetBool.Request()
        request.value = enable
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'{action} learning mode for {robot_name}: {result.message}')
            return result.status >= 0  # Niryo returns status code
        else:
            self.get_logger().error(f'Failed to {action.lower()} learning mode for {robot_name}')
            return False
    
    def setup_robots(self):
        """Setup both robots - enable learning mode on leader, disable on follower"""
        self.get_logger().info('Setting up robots...')
        
        # Enable learning mode on leader (Robot Four)
        self.get_logger().info(f'Enabling FreeMotion on leader ({self.leader_ns})...')
        success_leader = self.activate_learning_mode(self.leader_ns, self.leader_learning_mode_client)
        
        if success_leader:
            self.get_logger().info(f'✓ Leader ({self.leader_ns}) is now in FreeMotion - you can move it by hand!')
        else:
            self.get_logger().warn(f'Could not activate learning mode on leader - it may already be active')
        
        # Note: We don't explicitly disable learning mode on follower
        # The trajectory commands will automatically control it
        self.get_logger().info(f'Follower ({self.follower_ns}) will be controlled via trajectory commands')
        
        self.get_logger().info('Robot setup complete!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('READY! Move the leader robot by hand to control the follower.')
        self.get_logger().info('Press Ctrl+C to stop.')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    controller = LeaderFollowerController()
    
    # Setup robots
    controller.setup_robots()
    
    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        controller.get_logger().info('Shutting down gracefully...')
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.get_logger().info('Leader-Follower control stopped')
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
