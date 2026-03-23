# ArduPilot + MAVROS Migration Summary

## Migration Complete ✓

This document summarizes the successful migration of the ArduPilot SITL integration from xrce-dds (micro_ros_agent) to MAVROS.

## Files Modified/Created

### 1. **New Launch File**
- **Path:** `ardupilot/Tools/ros2/ardupilot_sitl/launch/sitl_mavros_udp.launch.py`
- **Purpose:** Replaces `sitl_dds_udp.launch.py`
- **Components:** SITL + MAVProxy + MAVROS node
- **Connection:** UDP 127.0.0.1:14550 (SITL) ↔ MAVROS

### 2. **Updated Launch Files**
- **Path:** `ardupilot_gz/ardupilot_gz_bringup/launch/robots/iris.launch.py`
- **Changes:**
  - Replaced `sitl_dds_udp.launch.py` with `sitl_mavros_udp.launch.py`
  - Removed `dds_udp.parm` from defaults parameter
  - Updated comments and descriptions

### 3. **Updated Application Code**
- **Path:** `drone_control/drone_control/circle_mission.py`
- **Changes:**
  - Replaced `ardupilot_msgs.srv` imports with `mavros_msgs.srv`
  - Updated service clients:
    - `ModeSwitch` → `SetMode`
    - `ArmMotors` → `CommandBool`
    - `Takeoff` → `CommandLong`
  - Updated topic subscriptions:
    - `/odometry` → `/mavros/local_position/pose`
    - Added `/mavros/state`
  - Updated topic publishers:
    - `/ap/cmd_vel` → `/mavros/setpoint_velocity/cmd_vel`

### 4. **Updated Package Dependencies**
- **drone_control/package.xml**
  - Added: `mavros_msgs`, `nav_msgs`
  - Removed dependency on `ardupilot_msgs`

- **ardupilot_gz_bringup/package.xml**
  - Added: `mavros` package

## Communication Flow

### Before (xrce-dds):
```
ArduPilot SITL 
    ↓ DDS/FastDDS
micro_ros_agent
    ↓ ROS 2
Your Application (drone_control)
    └─ Uses ardupilot_msgs topics/services
```

### After (MAVROS):
```
ArduPilot SITL
    ↓ MAVLink (UDP)
MAVROS Node
    ↓ ROS 2 Topics/Services
Your Application (drone_control)
    └─ Uses mavros_msgs topics/services
```

## Key API Changes Quick Reference

| Feature | Old API | New API |
|---------|---------|---------|
| Arm Vehicle | `/ap/arm_motors` service | `/mavros/cmd/arming` service ✓|
| Flight Mode | `/ap/mode_switch` service | `/mavros/set_mode` service |
| Takeoff | `/ap/experimental/takeoff` service | `/mavros/cmd/command` + MAV_CMD_NAV_TAKEOFF |
| Velocity Command | `/ap/cmd_vel` topic | `/mavros/setpoint_velocity/cmd_vel` |
| Position | `/odometry` (Odometry msg) | `/mavros/local_position/pose` (PoseStamped) |
| Vehicle State | (implicit) | `/mavros/state` topic |

## How to Use

### 1. Full System (SITL + Gazebo + MAVROS + Application)
```bash
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### 2. Just SITL + MAVROS (no Gazebo)
```bash
ros2 launch ardupilot_sitl sitl_mavros_udp.launch.py
```

### 3. Build and Run Drone Control
```bash
colcon build --packages-select drone_control
source install/setup.bash
ros2 run drone_control circle_mission
```

## Validation Checklist

- [x] Syntax validation: All Python files compile successfully
- [x] Import statements updated correctly
- [x] Service calls adapted to MAVROS API
- [x] Topic remappings configured
- [x] Package dependencies declared
- [x] Documentation created
- [ ] Functional testing (requires running system)
- [ ] Integration testing (requires running full stack)

## Testing Steps

1. **Check MAVROS is serving topics:**
   ```bash
   ros2 topic list | grep mavros
   ```

2. **Monitor vehicle state:**
   ```bash
   ros2 topic echo /mavros/state
   ```

3. **Verify connection:**
   ```bash
   ros2 service list | grep mavros
   ```

4. **Run circle mission:**
   ```bash
   ros2 run drone_control circle_mission
   ```

## Troubleshooting

- **MAVROS not connecting:** Check UDP port 14550/14551 are available
- **Services not found:** Ensure MAVROS node is running (`ros2 node list`)
- **Vehicle state not updating:** Check ArduPilot SITL is running
- **Command failures:** Monitor `/mavros/state` - may need to be in correct mode

## Notes

- MAVROS typically operates at 10-50Hz depending on SITL output rate
- Position frame conventions changed from `odom` to `map` (MAVROS standard)
- Service calls are now asynchronous (`call_async`) - add callback handling as needed
- MAVLink provides more robust error checking than xrce-dds DDS bridge
- MAVROS plugin system allows selective enabling of features (see `sitl_mavros_udp.launch.py`)

## References

- [MAVROS GitHub](https://github.com/mavlink/mavros)
- [ArduPilot ROS2 Integration](https://ardupilot.org/dev/docs/ros-sitl.html)
- [MAVLink Protocol](https://mavlink.io/)
- Migration guide: See `MAVROS_MIGRATION.md`
