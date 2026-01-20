#!/usr/bin/env bash
# Helper script to run the leader-follower control system

echo "=========================================="
echo "Leader-Follower Robot Control System"
echo "=========================================="
echo ""
echo "Leader: Robot Four (will be in FreeMotion)"
echo "Follower: Robot Three (will mimic leader)"
echo ""
echo "Make sure:"
echo "  1. Both robots are powered on and connected"
echo "  2. The driver is running (./launch_driver.sh)"
echo "  3. Both robots are calibrated"
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Unset the discovery server to use default multicast discovery
unset ROS_DISCOVERY_SERVER

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source workspace
source install/setup.bash

# Set domain ID
export ROS_DOMAIN_ID=1

echo ""
echo "=========================================="
echo "Starting Leader-Follower Control..."
echo "=========================================="
echo ""

# Run the control script
python3 leader_follower_control.py
