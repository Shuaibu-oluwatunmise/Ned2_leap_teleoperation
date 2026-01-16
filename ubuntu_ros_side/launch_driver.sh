#!/usr/bin/env bash

# Unset the discovery server to use default multicast discovery
unset ROS_DISCOVERY_SERVER

# Activate virtual environment
source venv/bin/activate

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source workspace
source install/setup.bash

# Set domain ID
export ROS_DOMAIN_ID=1

echo "=== ROS 2 Environment ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_DISCOVERY_SERVER: $ROS_DISCOVERY_SERVER"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "========================="

# Launch the driver
ros2 launch niryo_ned_ros2_driver driver.launch.py \
  drivers_list_file:=src/ned-ros2-driver/niryo_ned_ros2_driver/config/drivers_list.yaml
