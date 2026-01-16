#!/usr/bin/env bash

# Helper script to set up ROS 2 environment for Niryo driver
# This script disables discovery server mode and sets the correct domain ID

# Unset the discovery server to use default multicast discovery
unset ROS_DISCOVERY_SERVER

# Set domain ID
export ROS_DOMAIN_ID=1

# Restart the ROS 2 daemon with the new settings
echo "Restarting ROS 2 daemon..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start

echo "ROS 2 environment configured:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_DISCOVERY_SERVER: (disabled)"
echo ""
echo "You can now use ros2 commands like:"
echo "  ros2 node list"
echo "  ros2 topic list"
echo "  ros2 service list"
