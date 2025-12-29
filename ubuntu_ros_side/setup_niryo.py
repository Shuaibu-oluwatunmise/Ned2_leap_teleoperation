#!/usr/bin/env python3
"""
Niryo NED2 ROS2 Driver Setup Script
This script automates the complete setup process for controlling a Niryo NED2 robot with ROS2
"""

import subprocess
import os
import sys
from pathlib import Path

def run_command(cmd, description="", check=True, shell=True):
    """Run a shell command and handle errors"""
    print(f"\n{'='*60}")
    print(f"STEP: {description}")
    print(f"Running: {cmd}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(cmd, shell=shell, check=check, capture_output=False)
        print(f"✅ SUCCESS: {description}")
        return result
    except subprocess.CalledProcessError as e:
        print(f"❌ FAILED: {description}")
        print(f"Error: {e}")
        if check:
            sys.exit(1)
        return e

def create_robot_config():
    """Create the robot configuration file"""
    config_content = """rosbridge_port: 9090
robot_namespaces:
  - "Three"
robot_ips:
  - "192.168.8.143"
"""
    
    config_path = Path("~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml").expanduser()
    
    print(f"\n{'='*60}")
    print("STEP: Creating robot configuration file")
    print(f"File: {config_path}")
    print(f"{'='*60}")
    
    try:
        with open(config_path, 'w') as f:
            f.write(config_content)
        print(f"✅ SUCCESS: Robot config created")
        print("Configuration:")
        print(config_content)
    except Exception as e:
        print(f"❌ FAILED: Could not create config file")
        print(f"Error: {e}")
        print("\nMANUAL STEP REQUIRED:")
        print("Please run this command to edit the config file:")
        print(f"nano {config_path}")
        print("And paste this content:")
        print(config_content)

def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║                NIRYO NED2 ROS2 SETUP SCRIPT              ║
║          Automated setup for robotic arm control         ║
╚══════════════════════════════════════════════════════════════╝
""")

    # Step 1: Initial workspace setup
    run_command("mkdir -p ~/ros2_drivers_ws/src", "Creating ROS2 workspace")
    
    os.chdir(Path("~/ros2_drivers_ws/src").expanduser())
    run_command("git clone https://github.com/NiryoRobotics/ned-ros2-driver.git", 
                "Cloning Niryo driver repository")
    
    os.chdir(Path("~/ros2_drivers_ws").expanduser())
    
    # Step 2: Initialize rosdep
    run_command("sudo rosdep init", "Initializing rosdep", check=False)
    run_command("rosdep update", "Updating rosdep database")
    
    # Step 3: Fix ROS repository issues
    run_command("sudo sed -i 's/ros2-testing/ros2/g' /etc/apt/sources.list.d/ros2.list",
                "Switching from ROS2 testing to main repository")
    
    run_command("sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg",
                "Removing old ROS GPG key")
    
    run_command("curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key",
                "Downloading new ROS GPG key")
    
    run_command("sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key",
                "Installing new ROS GPG key")
    
    run_command("sudo apt update", "Updating package lists")
    
    # Step 4: Install MoveIt and all dependencies
    moveit_packages = [
        "ros-jazzy-moveit",
        "ros-jazzy-moveit-msgs", 
        "ros-jazzy-moveit-core",
        "ros-jazzy-moveit-common",
        "ros-jazzy-moveit-configs-utils",
        "ros-jazzy-moveit-visual-tools",
        "ros-jazzy-moveit-ros-visualization",
        "ros-jazzy-moveit-ros-move-group",
        "ros-jazzy-moveit-kinematics",
        "ros-jazzy-moveit-planners",
        "ros-jazzy-moveit-simple-controller-manager"
    ]
    
    other_packages = [
        "ros-jazzy-joint-state-publisher-gui",
        "ros-jazzy-launch-pytest",
        "ros-jazzy-geometric-shapes", 
        "ros-jazzy-random-numbers",
        "ros-jazzy-urdfdom-py",
        "libompl-dev",
        "libompl16t64",
        "ros-jazzy-ompl",
        "ros-jazzy-hpp-fcl",
        "python3.12-venv",
        "python3-pip",
        "ros-jazzy-topic-tools"
    ]
    
    all_packages = " ".join(moveit_packages + other_packages)
    run_command(f"sudo apt install -y {all_packages}", "Installing MoveIt2 and all dependencies")
    
    # Step 5: Install rosdep dependencies
    run_command("rosdep install --from-paths src --ignore-src -r -y", "Installing project dependencies")
    
    # Step 6: Create Python virtual environment
    run_command("python3 -m venv venv --system-site-packages", "Creating Python virtual environment")
    
    # Step 7: Build the workspace
    print(f"\n{'='*60}")
    print("STEP: Building ROS2 workspace")
    print("This may take several minutes...")
    print(f"{'='*60}")
    
    # Activate virtual environment and install Python requirements
    run_command("bash -c 'source venv/bin/activate && pip install -r src/ned-ros2-driver/requirements.txt'", 
                "Installing Python requirements in virtual environment")
    
    # Build the workspace
    run_command("bash -c 'source /opt/ros/jazzy/setup.bash && colcon build'", 
                "Building ROS2 workspace")
    
    # Step 8: Create robot configuration
    create_robot_config()
    
    # Step 9: Final instructions
    print(f"""
╔══════════════════════════════════════════════════════════════╗
║                    SETUP COMPLETE!                       ║
╚══════════════════════════════════════════════════════════════╝

TO START USING YOUR ROBOT:

1. Open 3 terminals and run these commands in each:

   Terminal 1 (Robot Driver):
   cd ~/ros2_drivers_ws
   source venv/bin/activate
   source /opt/ros/jazzy/setup.bash  
   source install/setup.bash
   ros2 launch niryo_ned_ros2_driver driver.launch.py drivers_list_file:=src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml

   Terminal 2 (Joint State Relay):
   source /opt/ros/jazzy/setup.bash
   ros2 run topic_tools relay /Three/joint_states /joint_states

   Terminal 3 (Robot Control):
   cd ~/ros2_drivers_ws
   source venv/bin/activate
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

2. Calibrate the robot:
   ros2 service call /Three/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{{value: 1}}"

3. Move the robot:
   ros2 action send_goal /Three/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{{
     trajectory: {{
       joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
       points: [{{
         positions: [0.0, 0.5, -0.5, 0.0, 0.0, 0.0],
         time_from_start: {{sec: 3}}
       }}]
     }}
   }}"

ROBOT CONFIGURATION:
- IP Address: 192.168.8.143  
- Namespace: Three
- Make sure your robot is powered on and connected to the network!

Happy robotics! 🤖
""")

if __name__ == "__main__":
    main()