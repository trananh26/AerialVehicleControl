# MAVROS - SITL vs Real Board Configuration Guide

## Overview
You now have two separate MAVROS configurations:
1. **SITL Mode**: For simulation with ArduPilot SITL
2. **Real Board Mode**: For connecting to actual ArduPilot hardware

## Configuration Comparison

| Aspect | SITL | Real Board |
|--------|------|-----------|
| **Autopilot** | Simulated (JSON), running locally | Physical hardware (Pixhawk, F450, etc.) |
| **Connection** | UDP localhost | USB/Serial connection |
| **URL Format** | `udp://127.0.0.1:14550@127.0.0.1:14551` | `/dev/ttyACM0:115200` |
| **Launch File** | `iris_runway.launch.py` | `apm.launch.py` |
| **Extra Components** | Includes SITL + MAVProxy | Just MAVROS |
| **Port Setup** | No setup needed | USB cable required |

## SITL Mode (Simulation)

### Launch Command
```bash
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start SITL with MAVROS
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### Verify Connection
```bash
# Check MAVROS is connected
ros2 echo /mavros/state

# Check topics are being published
ros2 topic list | grep mavros | wc -l  # Should show 70+ topics
```

---

## Real Board Mode (Hardware)

### Prerequisites
1. **Hardware Requirements**:
   - ArduPilot flight controller (Pixhawk, F450, etc.)
   - USB cable or serial adapter
   - Pre-configured autopilot firmware (ArduCopter, ArduPlane, etc.)

2. **Find Your Serial Port**:
   ```bash
   # List available serial ports
   ls /dev/tty* | grep -E "(ACM|USB)"
   
   # Common ports:
   # - /dev/ttyACM0 (USB native)
   # - /dev/ttyUSB0 (FTDI adapter)
   # - /dev/ttyS0 (UART)
   ```

3. **Determine Baudrate**:
   ```bash
   # Most common baudrates for ArduPilot:
   # - 115200 (default, most boards)
   # - 57600 (older boards)
   # - 921600 (higher speed for faster telemetry)
   ```

### Launch Command - Basic (USB)

**Default settings (most common):**
```bash
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mavros apm.launch.py

# Recommended for your board
ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyACM0:57600
```

### Launch Command - Custom Serial Port

**With custom port and baudrate:**
```bash
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyACM0:115200
```

**Other baudrates:**
```bash
# 57600 baud
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyUSB0:57600

# 921600 baud (faster)
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyUSB0:921600

# FTDI USB adapter
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyUSB0:115200
```

### Launch Command - Network Connection

**TCP connection (Ethernet):**
```bash
ros2 launch mavros apm.launch.py \
  fcu_url:=tcp://192.168.1.100:5760
```

**UDP connection (Telemetry radio):**
```bash
ros2 launch mavros apm.launch.py \
  fcu_url:=udp://0.0.0.0:14550@192.168.1.100:14550
```

### Verify Real Board Connection

```bash
# Check MAVROS connected to board
ros2 echo /mavros/state
# Should show: armed: false, mode: STABILIZE (or last mode)

# Check battery voltage
ros2 echo /mavros/sys_status

# Check IMU data
ros2 echo /mavros/imu/data

# Monitor connection heartbeat
ros2 topic hz /mavros/state  # Should show >10 Hz
```

---

## Troubleshooting Real Board Connection

### "Cannot open port" Error
```bash
# Check port permissions
ls -la /dev/ttyACM0

# Add user to dialout group (one-time setup)
sudo usermod -a -G dialout $USER
newgrp dialout

# Or use sudo
sudo ros2 launch mavros apm.launch.py
```

### "Connection timeout" Error

**Check USB connection:**
```bash
# Verify device appears
dmesg | tail -20  # Check for USB device messages
lsusb  # List USB devices
```

**Try different baudrate:**
```bash
# Common issue: wrong baudrate
# Try 57600 if 115200 doesn't work
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyACM0:57600
```

**Check if port is in use:**
```bash
# See what's using the port
lsof /dev/ttyACM0
```

### Board Not Responding

**Verify autopilot is armed in the device:**
- Check battery is connected
- Check firmware is loaded (use QGroundControl or Mission Planner)
- Check no other application is using the port

**Manually test connection:**
```bash
# Option 1: Use screen
screen /dev/ttyACM0 115200

# Option 2: Use minicom
minicom -D /dev/ttyACM0

# Type Ctrl-A then X to exit
```

---

## Running Your Application with Real Board

### Takeoff/Land Mission on Real Board
```bash
# Terminal 1: Connect MAVROS to real board
ros2 launch mavros apm.launch.py

# Terminal 2: Run takeoff/land mission
source install/setup.bash
ros2 run drone_control takeoff_land_mission
```

### Monitor in Real Time (Terminal 3)
```bash
# Watch position
ros2 echo /mavros/local_position/pose

# Watch state
ros2 echo /mavros/state

# Watch battery
ros2 echo /mavros/sys_status
```

---

## Quick Reference Table

```
┌─────────────────┬────────────────────────────┬─────────────────────────────┐
│ Use Case        │ SITL Command               │ Real Board Command          │
├─────────────────┼────────────────────────────┼─────────────────────────────┤
│ SITL (default)  │ iris_runway.launch.py      │ -                           │
│ USB Pixhawk     │ -                          │ apm.launch.py               │
│ Custom port     │ -                          │ apm.launch.py               │
│                 │                            │ fcu_url:=/dev/ttyUSB0:57600 │
│ Telemetry radio │ -                          │ apm.launch.py               │
│                 │                            │ fcu_url:=udp://IP:PORT      │
└─────────────────┴────────────────────────────┴─────────────────────────────┘
```

---

## MAVROS Topics - Same in Both Modes

### Useful Topics for Real Board
```bash
# Position and attitude
/mavros/local_position/pose          # Local position
/mavros/local_position/odom          # Odometry
/mavros/global_position/global       # GPS position

# Vehicle state
/mavros/state                         # Armed, mode, connected
/mavros/sys_status                    # Battery, CPU, etc.
/mavros/battery                       # Detailed battery info
/mavros/imu/data                      # IMU measurements

# Sensors
/mavros/imu/data_raw                 # Raw IMU data
/mavros/imu/mag                      # Magnetometer
/mavros/altitude                     # Barometer altitude

# Control (for takeoff_land_mission)
/mavros/cmd/command                  # MAV_CMD_NAV_TAKEOFF backend
/mavros/local_position/pose          # Position feedback (subscribe)
```

### Useful Services for Real Board
```bash
# Check all available services
ros2 service list | grep mavros | head -20

# Common services
/mavros/set_mode                     # Set flight mode
/mavros/cmd/arming                   # Arm/disarm
/mavros/cmd/takeoff                  # Takeoff
/mavros/cmd/land                     # Land
/mavros/cmd/set_home                 # Set home location
```

---

## Example Workflows

### Workflow 1: Test with SITL First, Then Real Board
```bash
# Step 1: Test everything with SITL
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
# (in another terminal)
ros2 run drone_control takeoff_land_mission

# Step 2: When ready, switch to real board
ros2 launch mavros apm.launch.py
# (same takeoff_land_mission command works without changes)
ros2 run drone_control takeoff_land_mission
```

### Current Situation Note
If the node logs `Waiting for set_mode service...`, the mission node is running correctly but MAVROS services are not available yet. Start or fix MAVROS connection first, then rerun:

```bash
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run drone_control takeoff_land_mission
```

### Workflow 2: Multiple Boards
```bash
# Terminal 1: Board 1
ros2 launch mavros apm.launch.py \
  --namespace board1 \
  fcu_url:=/dev/ttyACM0:115200

# Terminal 2: Board 2
ros2 launch mavros apm.launch.py \
  --namespace board2 \
  fcu_url:=/dev/ttyUSB0:115200

# Topics now under /board1/mavros and /board2/mavros
```

---

## Safety Precautions for Real Board

⚠️ **Before Flying:**
1. Remove propellers initially (for testing)
2. Test all motor directions with M1, M2, M3, M4 commands
3. Verify stabilize mode is working
4. Check battery voltage
5. Verify GPS lock (if using GPS)
6. Test in MANUAL mode first, then STABILIZE
7. Only after full testing, attempt autonomous modes (GUIDED, AUTO)

```bash
# Safe testing order:
# 1. Arm in MANUAL mode
# 2. Test in STABILIZE mode
# 3. Test in MODE GUIDED (with low altitude)
# 4. Only then run autonomous missions
```

---

## Files Created/Modified

```
✅ New Launch Files:
  - Use MAVROS built-in launch: `mavros/launch/apm.launch.py`

✅ Existing SITL Launch (unchanged):
  - ardupilot_gz_bringup/launch/iris_runway.launch.py
```

Both configurations use the same MAVROS topics and your `takeoff_land_mission` node - just different connection methods.
