#!/usr/bin/env python3
"""
Niryo NED2 Dual Robot ROS2 Driver Setup Script
This script sets up TWO Niryo NED2 robots for simultaneous control
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
    """Create robot configuration file (supports single or dual robots)"""
    print("\n" + "="*60)
    print("ROBOT CONFIGURATION")
    print("="*60)
    print("This setup supports controlling 1 or 2 Niryo robots.")
    print()
    
    # Ask for number of robots
    while True:
        num_robots = input("How many robots do you want to control? (1 or 2) [default: 1]: ").strip() or "1"
        if num_robots in ["1", "2"]:
            num_robots = int(num_robots)
            break
        print("Please enter 1 or 2")
    
    # Get robot 1 IP
    robot1_ip = input("\nRobot 1 IP address [default: 192.168.8.143]: ").strip() or "192.168.8.143"
    
    if num_robots == 1:
        # Single robot configuration
        config_content = f"""rosbridge_port: 9090
robot_namespaces:
  - "Three"
robot_ips:
  - "{robot1_ip}"
"""
        print(f"\n✅ Configured for SINGLE robot control")
        print(f"   Robot: Three @ {robot1_ip}")
        print(f"   Control: RIGHT HAND only")
        
    else:
        # Dual robot configuration
        robot2_ip = input("Robot 2 IP address [default: 192.168.8.144]: ").strip() or "192.168.8.144"
        
        config_content = f"""rosbridge_port: 9090
robot_namespaces:
  - "Three"
  - "Four"
robot_ips:
  - "{robot1_ip}"
  - "{robot2_ip}"
"""
        print(f"\n✅ Configured for DUAL robot control")
        print(f"   Robot 1 (Three): {robot1_ip} - RIGHT HAND")
        print(f"   Robot 2 (Four):  {robot2_ip} - LEFT HAND")
    
    config_path = Path("~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml").expanduser()
    
    print(f"\n{'='*60}")
    print("STEP: Creating dual robot configuration file")
    print(f"File: {config_path}")
    print(f"{'='*60}")
    
    try:
        with open(config_path, 'w') as f:
            f.write(config_content)
        print(f"✅ SUCCESS: Dual robot config created")
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
║            NIRYO NED2 ROS2 SETUP SCRIPT                  ║
║       Setup for controlling 1 or 2 robots with hands     ║
╚══════════════════════════════════════════════════════════════╝
""")

    # Check if workspace already exists
    workspace_path = Path("~/ros2_drivers_ws").expanduser()
    
    if workspace_path.exists():
        print(f"\n⚠️  Workspace already exists at {workspace_path}")
        response = input("Do you want to reconfigure for dual robots? (y/n): ").strip().lower()
        
        if response != 'y':
            print("Setup cancelled.")
            sys.exit(0)
        
        print("\n✅ Reconfiguring existing workspace for dual robots...")
        os.chdir(workspace_path)
        
        # Just update the config file
        create_robot_config()
        
    else:
        # Full setup (same as original)
        run_command("mkdir -p ~/ros2_drivers_ws/src", "Creating ROS2 workspace")
        
        os.chdir(Path("~/ros2_drivers_ws/src").expanduser())
        run_command("git clone https://github.com/NiryoRobotics/ned-ros2-driver.git", 
                    "Cloning Niryo driver repository")
        
        os.chdir(Path("~/ros2_drivers_ws").expanduser())
        
        # Step 2: Initialize rosdep
        run_command("sudo rosdep init", "Initializing rosdep", check=False)
        run_command("rosdep update", "Updating rosdep database")
        
        # Step 3: Update packages
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
        
        # Step 8: Create dual robot configuration
        create_robot_config()
    
    # Final instructions
    print(f"""
╔══════════════════════════════════════════════════════════════╗
║                   SETUP COMPLETE!                        ║
╚══════════════════════════════════════════════════════════════╝

Check your configuration file to see if you're using 1 or 2 robots:
cat ~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml

TO START USING YOUR ROBOT SYSTEM:

1. Terminal 1 (Dual Robot Driver):
   cd ~/ros2_drivers_ws
   ./launch_driver.sh

2. Terminal 2 (Hand Receiver):
   cd ~/ros2_drivers_ws
   unset ROS_DISCOVERY_SERVER
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=1
   python3 hand_receiverV2.py

3. Terminal 3 (Robot Controller):
   cd ~/ros2_drivers_ws
   unset ROS_DISCOVERY_SERVER
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   export ROS_DOMAIN_ID=1
   
   For SINGLE robot (right hand only):
   python3 robot_controllerV2.py
   
   For DUAL robots (both hands):
   python3 robot_controller_both.py

CALIBRATE ROBOT(S):
Single robot:
  ros2 service call /Three/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{{value: 1}}"

Dual robots:
  ros2 service call /Three/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{{value: 1}}"
  ros2 service call /Four/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt "{{value: 1}}"

VERIFY SETUP:
ros2 node list
Single robot: Should show /ros2_driver_Three
Dual robots: Should show /ros2_driver_Three AND /ros2_driver_Four

Happy robot control! 🤖
""")

if __name__ == "__main__":
    main()
