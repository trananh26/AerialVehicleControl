# MAVROS Migration Guide

## Overview
This project has been ported from xrce-dds (micro_ros_agent) to MAVROS for ArduPilot SITL communication.

## Key Changes

### Architecture
**Before (xrce-dds):**
```
ArduPilot SITL ←→ micro_ros_agent (xrce-dds bridge) ←→ ROS 2 Topics/Services
  (ardupilot_msgs)
```

**After (MAVROS):**
```
ArduPilot SITL ←→ MAVROS (MAVLink to ROS bridge) ←→ ROS 2 Topics/Services
  (mavros_msgs)
```

### Launch Files Modified
1. **New:** `ardupilot/Tools/ros2/ardupilot_sitl/launch/sitl_mavros_udp.launch.py`
   - Replaces `sitl_dds_udp.launch.py`
   - Launches SITL, MAVProxy (optional), and MAVROS node
   - MAVROS connects to ArduPilot SITL via UDP (127.0.0.1:14550)

2. **Updated:** `ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py`
   - Changed from `sitl_dds_udp.launch.py` to `sitl_mavros_udp.launch.py`
   - Removed `dds_udp.parm` from default parameters

### Topics/Services Migration

#### Services
| Old (ardupilot_msgs) | New (MAVROS) | Command |
|---|---|---|
| `/ap/mode_switch` | `/mavros/set_mode` | Switch flight mode (e.g., "GUIDED") |
| `/ap/arm_motors` | `/mavros/cmd/arming` | Arm/disarm vehicle |
| `/ap/experimental/takeoff` | `/mavros/cmd/command` | MAV_CMD_NAV_TAKEOFF (command 22) |

#### Topics
| Old | New | Purpose |
|---|---|---|
| `/ap/cmd_vel` | `/mavros/setpoint_velocity/cmd_vel` | Velocity commands |
| `/odometry` | `/mavros/local_position/pose` | Position/pose (needs conversion) |
| N/A | `/mavros/state` | Vehicle state (armed, mode, etc.) |
| N/A | `/mavros/battery` | Battery information |
| N/A | `/mavros/imu/data` | IMU data |

### Code Changes

#### Package Dependencies
- **drone_control/package.xml**
  - Added: `mavros_msgs`
  - Added: `nav_msgs`
  - Removed: `ardupilot_msgs` (implicit, no longer needed)

- **ardupilot_gz_bringup/package.xml**
  - Added: `mavros` package dependency

#### drone_control/circle_mission.py
- Changed imports from `ardupilot_msgs.srv` to `mavros_msgs.srv`
- Updated service clients:
  - `ModeSwitch` → `SetMode`
  - `ArmMotors` → `CommandBool`
  - `Takeoff` → `CommandLong`
- Updated subscriptions:
  - `/odometry` (Odometry) → `/mavros/local_position/pose` (PoseStamped)
  - Added `/mavros/state` subscription for vehicle state monitoring
- Updated publisher:
  - `/ap/cmd_vel` → `/mavros/setpoint_velocity/cmd_vel`
- Frame reference updated: `"odom"` → `"map"` (MAVROS convention)

## MAVROS Configuration

MAVROS connects to ArduPilot SITL via:**UDP:** `127.0.0.1:14550` (receive from SITL's MAVLink output)
- **MAVROS will bind to:** `127.0.0.1:14551` (send commands)

### Default MAVROS Plugins Enabled
- ardupilot_msgs (for ArduPilot-specific messages)
- diagnostic_aggregator
- image_common
- local_position
- sys_status
- global_position
- imu_pub
- home_position

## Running the Updated System

### Launch SITL + MAVROS + Gazebo
```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### Launch Just SITL + MAVROS (without Gazebo)
```bash
ros2 launch ardupilot_sitl sitl_mavros_udp.launch.py
```

### Build the drone_control package
```bash
cd mavros_sw_ws
colcon build --packages-select drone_control
source install/setup.bash
```

### Run circle mission
```bash
ros2 run drone_control circle_mission
```

## Connection Debugging

Check MAVROS connection status:
```bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/system_status/heartbeat
```

Monitor MAVLink traffic (with MAVProxy running):
```bash
# In MAVProxy shell:
status
```

## Key Differences from DDS

1. **MAVLink Protocol:** MAVROS uses MAVLink v2.0, while xrce-dds uses FastDDS
2. **Network:** MAVROS uses UDP, xrce-dds can use both serial and UDP
3. **Plugin System:** MAVROS loads plugins; xrce-dds has a fixed set of topics
4. **Framerate:** MAVROS typically updates at ~10-50Hz depending on SITL output
5. **Messages:** Different message types and service definitions

## Troubleshooting

### MAVROS not connecting
- Ensure ArduPilot SITL is running: `ps aux | grep sim_vehicle`
- Check ports: `netstat -an | grep 1455`
- Verify MAVROS parameters in `sitl_mavros_udp.launch.py`

### Vehicle not arming
- Check `/mavros/state` for current mode
- Ensure in correct mode for arming (usually GUIDED)
- Check MAVROS system_status heartbeat

### Commands not working
- Verify services are available: `ros2 service list | grep mavros`
- Check ROS 2 namespace: services are under `/mavros`
- Monitor MAVROS console output for errors

## References
- [MAVROS Documentation](https://github.com/mavlink/mavros)
- [ArduPilot ROS Integration](https://ardupilot.org/dev/docs/ros-sitl.html)
- [MAVLink Protocol](https://mavlink.io/)
