#!/usr/bin/env python3
"""
Button-Triggered Bilateral Leader-Follower Robot Control
Uses physical FreeMotion buttons on robots to determine leader

Press the FreeMotion button on either robot to make it the leader.
The other robot automatically becomes the follower.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from niryo_ned_ros2_interfaces.srv import SetBool
import numpy as np
import sys
import signal


class ButtonBilateralController(Node):
    def __init__(self):
        super().__init__('button_bilateral_controller')
        
        # Robot namespaces
        self.robot_three_ns = 'Three'
        self.robot_four_ns = 'Four'
        
        # Control parameters (ULTRA-LOW-LATENCY MODE)
        self.control_rate = 200.0  # Hz
        self.trajectory_duration = 0.03  # seconds
        self.smoothing_alpha = 0.7  # EMA smoothing factor
        
        # State variables for Robot Three
        self.three_joint_positions = None
        self.three_joint_names = None
        self.three_smoothed_positions = None
        self.three_learning_mode = False  # FreeMotion state
        
        # State variables for Robot Four
        self.four_joint_positions = None
        self.four_joint_names = None
        self.four_smoothed_positions = None
        self.four_learning_mode = False  # FreeMotion state
        
        # Leader tracking
        self.current_leader = None  # "Three" or "Four" or None
        self.is_ready = False
        
        # Create subscribers for joint states
        self.three_joint_sub = self.create_subscription(
            JointState,
            f'/{self.robot_three_ns}/joint_states',
            self.three_joint_callback,
            10
        )
        
        self.four_joint_sub = self.create_subscription(
            JointState,
            f'/{self.robot_four_ns}/joint_states',
            self.four_joint_callback,
            10
        )
        
        # Create subscribers for learning mode state (FreeMotion button)
        self.three_learning_sub = self.create_subscription(
            Bool,
            f'/{self.robot_three_ns}/niryo_robot/learning_mode/state',
            self.three_learning_callback,
            10
        )
        
        self.four_learning_sub = self.create_subscription(
            Bool,
            f'/{self.robot_four_ns}/niryo_robot/learning_mode/state',
            self.four_learning_callback,
            10
        )
        
        # Create action clients for both robots
        self.three_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.robot_three_ns}/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.four_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.robot_four_ns}/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Create service clients for learning mode control
        self.three_learning_mode_client = self.create_client(
            SetBool,
            f'/{self.robot_three_ns}/niryo_robot/learning_mode/activate'
        )
        
        self.four_learning_mode_client = self.create_client(
            SetBool,
            f'/{self.robot_four_ns}/niryo_robot/learning_mode/activate'
        )
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        # Statistics
        self.command_count = 0
        self.switch_count = 0
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('Button-Triggered Bilateral Controller initialized')
        self.get_logger().info(f'Robot Three: {self.robot_three_ns}')
        self.get_logger().info(f'Robot Four: {self.robot_four_ns}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info('Press FreeMotion button on either robot to make it the leader!')
        
    def three_joint_callback(self, msg):
        """Callback for Robot Three joint states"""
        self.three_joint_positions = np.array(msg.position)
        self.three_joint_names = list(msg.name)
        
        if self.three_smoothed_positions is None:
            self.three_smoothed_positions = self.three_joint_positions.copy()
            self.get_logger().info(f'Robot Three ready: {len(self.three_joint_names)} joints')
            self._check_ready()
        else:
            self.three_smoothed_positions = (
                self.smoothing_alpha * self.three_joint_positions +
                (1 - self.smoothing_alpha) * self.three_smoothed_positions
            )
    
    def four_joint_callback(self, msg):
        """Callback for Robot Four joint states"""
        self.four_joint_positions = np.array(msg.position)
        self.four_joint_names = list(msg.name)
        
        if self.four_smoothed_positions is None:
            self.four_smoothed_positions = self.four_joint_positions.copy()
            self.get_logger().info(f'Robot Four ready: {len(self.four_joint_names)} joints')
            self._check_ready()
        else:
            self.four_smoothed_positions = (
                self.smoothing_alpha * self.four_joint_positions +
                (1 - self.smoothing_alpha) * self.four_smoothed_positions
            )
    
    def three_learning_callback(self, msg):
        """Callback for Robot Three learning mode state"""
        old_state = self.three_learning_mode
        self.three_learning_mode = msg.data
        
        # Detect button press (transition to FreeMotion)
        if not old_state and self.three_learning_mode:
            self.get_logger().info('🔘 Robot Three FreeMotion button pressed!')
            self._set_leader("Three")
    
    def four_learning_callback(self, msg):
        """Callback for Robot Four learning mode state"""
        old_state = self.four_learning_mode
        self.four_learning_mode = msg.data
        
        # Detect button press (transition to FreeMotion)
        if not old_state and self.four_learning_mode:
            self.get_logger().info('🔘 Robot Four FreeMotion button pressed!')
            self._set_leader("Four")
    
    def _check_ready(self):
        """Check if both robots are ready"""
        if (self.three_smoothed_positions is not None and 
            self.four_smoothed_positions is not None and 
            not self.is_ready):
            self.is_ready = True
            self.get_logger().info('Both robots ready! Waiting for FreeMotion button press...')
    
    def _set_leader(self, new_leader):
        """Set which robot is the leader"""
        if self.current_leader == new_leader:
            return  # Already the leader
        
        old_leader = self.current_leader
        self.current_leader = new_leader
        
        if old_leader is not None:
            self.switch_count += 1
            self.get_logger().info(f'🔄 Leader switched: {old_leader} → {new_leader}')
        else:
            self.get_logger().info(f'✓ Leader set: {new_leader}')
        
        # Ensure follower is NOT in FreeMotion
        if new_leader == "Four":
            # Four is leader, make sure Three is controlled
            self.activate_learning_mode(self.robot_three_ns, self.three_learning_mode_client, enable=False)
        else:  # Three is leader
            # Three is leader, make sure Four is controlled
            self.activate_learning_mode(self.robot_four_ns, self.four_learning_mode_client, enable=False)
    
    def control_loop(self):
        """Main control loop - sends follower commands based on leader position"""
        if not self.is_ready or self.current_leader is None:
            return
        
        # Get leader positions and follower action client
        if self.current_leader == "Four":
            leader_positions = self.four_smoothed_positions
            leader_joint_names = self.four_joint_names
            follower_client = self.three_action_client
        else:  # leader == "Three"
            leader_positions = self.three_smoothed_positions
            leader_joint_names = self.three_joint_names
            follower_client = self.four_action_client
        
        # Check if follower action server is available
        if not follower_client.server_is_ready():
            if self.command_count == 0:
                self.get_logger().warn('Waiting for follower action server...', throttle_duration_sec=5.0)
            return
        
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = leader_joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = leader_positions.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.trajectory_duration * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        # Send goal (non-blocking)
        follower_client.send_goal_async(goal_msg)
        self.command_count += 1
    
    def print_statistics(self):
        """Print control statistics"""
        if self.is_ready:
            leader_str = self.current_leader if self.current_leader else "None (press button!)"
            self.get_logger().info(
                f'Stats - Leader: {leader_str}, '
                f'Commands: {self.command_count}, '
                f'Switches: {self.switch_count}'
            )
    
    def activate_learning_mode(self, robot_name, client, enable=True):
        """Activate or deactivate learning mode for a robot"""
        action = "Activating" if enable else "Deactivating"
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Learning mode service not available for {robot_name}', throttle_duration_sec=5.0)
            return False
        
        request = SetBool.Request()
        request.value = enable
        # Send async without waiting for result to avoid blocking
        client.call_async(request)
        return True
    
    def setup_robots(self):
        """Setup both robots - disable FreeMotion on both initially"""
        self.get_logger().info('Setting up robots for button-triggered bilateral control...')
        
        # Disable FreeMotion on BOTH robots initially
        self.get_logger().info(f'Disabling FreeMotion on both robots...')
        self.get_logger().info(f'Press the FreeMotion button on either robot to start!')
        
        success_three = self.activate_learning_mode(self.robot_three_ns, self.three_learning_mode_client, enable=False)
        success_four = self.activate_learning_mode(self.robot_four_ns, self.four_learning_mode_client, enable=False)
        
        if success_three and success_four:
            self.get_logger().info(f'✓ Both robots controlled (FreeMotion OFF)')
        else:
            self.get_logger().warn(f'Could not set up learning modes properly')
        
        self.get_logger().info('Robot setup complete!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('BUTTON-TRIGGERED BILATERAL CONTROL ACTIVE!')
        self.get_logger().info('Press FreeMotion button on either robot to make it the leader.')
        self.get_logger().info('The other robot will automatically follow.')
        self.get_logger().info('Press Ctrl+C to stop.')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    controller = ButtonBilateralController()
    
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
        controller.get_logger().info('Button-triggered bilateral control stopped')
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
