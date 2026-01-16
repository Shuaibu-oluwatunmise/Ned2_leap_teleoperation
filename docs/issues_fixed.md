# Issues Fixed - Niryo NED ROS2 Driver

## Date: 2026-01-16

## Summary
Fixed critical ROS 2 node discovery issues that prevented the Niryo NED ROS2 driver from being visible in the ROS 2 graph, causing `ros2 node list` to return empty results and service calls to hang indefinitely.

---

## Problem Description

### Symptoms
1. **Node Discovery Failure**: Running `ros2 node list` returned no results, even though the driver process was running
2. **Service Calls Hanging**: Attempting to call services (e.g., `/Three/niryo_robot/joints_interface/calibrate_motors`) would hang indefinitely with "waiting for service to become available..."
3. **Misleading Success Messages**: The driver logged "Bridge node initialized" but wasn't actually participating in the ROS 2 graph

### Initial Observations
- Driver process was running (PID visible in `ps aux`)
- Log messages showed successful initialization:
  ```
  [INFO] [ros2_driver_Three]: Creating driver for robot with IP: 192.168.8.143 and port: 9090
  [INFO] [ros2_driver_Three]: Bridge node initialized for robot with ip: 192.168.8.143
  ```
- Default ROS 2 topics (`/parameter_events`, `/rosout`) were visible, but no driver-specific topics/services appeared

---

## Root Cause Analysis

### Investigation Steps

1. **Domain ID Mismatch (Initial Hypothesis)**
   - Checked `ROS_DOMAIN_ID` in different terminals
   - Found driver was using `ROS_DOMAIN_ID=1` while some terminals defaulted to `0`
   - **Result**: Fixing domain ID alone didn't solve the problem

2. **Missing Dependencies**
   - Discovered `ModuleNotFoundError: No module named 'roslibpy'` when launching without virtual environment
   - **Result**: Virtual environment (`venv`) was required but not the root cause of discovery issues

3. **DDS Discovery Configuration (Root Cause)**
   - Examined environment variables: `cat /proc/<PID>/environ | tr '\0' '\n' | grep ROS`
   - **Found**: `ROS_DISCOVERY_SERVER=;192.168.8.201:11811;`
   - **Impact**: ROS 2 was configured to use Discovery Server mode instead of default multicast discovery
   - The discovery server at `192.168.8.201:11811` was not accessible
   - This prevented all node-to-node discovery, even though nodes were running

4. **ROS 2 Daemon State**
   - The ROS 2 daemon was caching the discovery server configuration
   - Simply unsetting `ROS_DISCOVERY_SERVER` in new terminals wasn't sufficient
   - **Solution**: Daemon needed to be restarted with the new configuration

### Root Cause
**ROS 2 Discovery Server mode was enabled but the discovery server was unreachable**, preventing DDS-based node discovery. The ROS 2 daemon was caching this configuration, requiring a full daemon restart to apply changes.

---

## Solution Implemented

### 1. Environment Configuration
Created helper scripts to properly configure the ROS 2 environment:

#### `launch_driver.sh`
- Unsets `ROS_DISCOVERY_SERVER` to disable Discovery Server mode
- Activates the Python virtual environment (required for `roslibpy` dependency)
- Sources ROS 2 and workspace setup files
- Sets `ROS_DOMAIN_ID=1`
- Launches the Niryo driver with proper configuration

#### `setup_ros2_env.sh`
- Configures terminal environment for ROS 2 CLI commands
- Unsets `ROS_DISCOVERY_SERVER`
- Sets `ROS_DOMAIN_ID=1`
- Restarts the ROS 2 daemon to apply new settings
- Provides usage instructions

### 2. Workflow Changes

**Before (Broken):**
```bash
# Terminal 1: Launch driver
cd ~/ros2_drivers_ws
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch niryo_ned_ros2_driver driver.launch.py drivers_list_file:=...

# Terminal 2: Try to use ROS 2 commands
ros2 node list  # Returns nothing!
```

**After (Fixed):**
```bash
# Terminal 1: Launch driver
cd ~/ros2_drivers_ws
./launch_driver.sh

# Terminal 2: Use ROS 2 commands
cd ~/ros2_drivers_ws
source setup_ros2_env.sh
source install/setup.bash
ros2 node list  # Works! Shows /ros2_driver_Three
ros2 service call /Three/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt '{value: 1}'
```

---

## Technical Details

### Environment Variables
The following environment variables are critical for proper operation:

| Variable | Required Value | Purpose |
|----------|---------------|---------|
| `ROS_DOMAIN_ID` | `1` | Ensures all nodes communicate on the same DDS domain |
| `ROS_DISCOVERY_SERVER` | `(unset)` | Disables Discovery Server mode, enables multicast discovery |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS middleware (default, no change needed) |

### Discovery Server vs Multicast Discovery

**Discovery Server Mode** (was enabled):
- Nodes must connect to a central discovery server
- Requires `ROS_DISCOVERY_SERVER` to point to an accessible server
- Useful for complex network topologies or WAN deployments
- **Problem**: Server at `192.168.8.201:11811` was not accessible

**Multicast Discovery** (now enabled):
- Default ROS 2 discovery mechanism
- Nodes discover each other via UDP multicast
- Works on local networks without additional infrastructure
- **Solution**: Disabled Discovery Server to use this mode

### Dependencies
The driver requires the following Python dependencies:
- `roslibpy<2.0.0` - ROSBridge client library
- `pyyaml` - YAML configuration parsing

These are installed in the virtual environment at `venv/`.

---

## Files Created/Modified

### New Files
1. **`launch_driver.sh`** - Driver launch script with proper environment setup
2. **`setup_ros2_env.sh`** - Terminal environment configuration script
3. **`test_node.py`** - Minimal test node for debugging (can be removed)
4. **`issues_fixed.md`** - This documentation file

### Modified Files
None - all fixes were implemented via environment configuration scripts.

---

## Verification

### Test Results
After implementing the fix:

1. **Node Discovery**:
   ```bash
   $ ros2 node list
   /ros2_driver_Three
   ```
   ✅ Success

2. **Service Discovery**:
   ```bash
   $ ros2 service list | grep calibrate
   /Three/niryo_robot/joints_interface/calibrate_motors
   /Three/niryo_robot/joints_interface/factory_calibrate_motors
   ```
   ✅ Success

3. **Service Call**:
   ```bash
   $ ros2 service call /Three/niryo_robot/joints_interface/calibrate_motors niryo_ned_ros2_interfaces/srv/SetInt '{value: 1}'
   response:
   niryo_ned_ros2_interfaces.srv.SetInt_Response(status=1, message='JointHardwareInterface::calibrateJoints - Calibration already done')
   ```
   ✅ Success

---

## Recommendations

### For Future Development

1. **Document Environment Requirements**
   - Add environment setup instructions to the main README
   - Document the requirement for `ROS_DOMAIN_ID=1`
   - Explain why Discovery Server mode is disabled

2. **Dependency Management**
   - Consider adding `roslibpy` to `rosdep` dependencies
   - Document the virtual environment requirement
   - Or install `roslibpy` system-wide if appropriate

3. **Launch File Improvements**
   - Consider adding environment variable checks to the launch file
   - Warn users if `ROS_DISCOVERY_SERVER` is set
   - Automatically set `ROS_DOMAIN_ID` in the launch file

4. **Testing**
   - Add integration tests that verify node discovery
   - Test in both Discovery Server and multicast modes
   - Document expected behavior in different network configurations

### For Users

1. **Always use the provided scripts**:
   - Use `./launch_driver.sh` to start the driver
   - Use `source setup_ros2_env.sh` before running ROS 2 commands

2. **Verify environment**:
   ```bash
   echo $ROS_DOMAIN_ID  # Should be 1
   echo $ROS_DISCOVERY_SERVER  # Should be empty
   ```

3. **If issues persist**:
   - Restart the ROS 2 daemon: `ros2 daemon stop && ros2 daemon start`
   - Check firewall settings (multicast UDP must be allowed)
   - Verify network connectivity to robot at `192.168.8.143:9090`

---

## Lessons Learned

1. **Environment variables matter**: ROS 2's behavior is heavily influenced by environment variables, especially for DDS discovery
2. **Daemon state is persistent**: The ROS 2 daemon caches configuration and must be restarted when environment changes
3. **Discovery Server is not always appropriate**: For local development, multicast discovery is simpler and more reliable
4. **Misleading success messages**: A node can initialize successfully but still not participate in the graph due to discovery issues
5. **Systematic debugging**: Checking environment variables early would have saved significant debugging time

---

## References

- [ROS 2 DDS Discovery](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Discovery.html)
- [ROS 2 Discovery Server](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)
- [ROS 2 Environment Variables](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [Fast DDS Discovery Mechanisms](https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery.html)

---

## Contact

For questions or issues related to this fix, please refer to the commit history or open an issue in the repository.
