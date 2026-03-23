# MAVROS Quick Start - SITL vs Real Board

## 📋 Setup (One Time)
```bash
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 🔵 SIMULATION MODE (SITL)

### Launch Command
```bash
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### What It Starts
- ArduCopter simulator (JSON) running locally
- MAVProxy bridge for debugging
- MAVROS connected via UDP
- No hardware needed!

### Test It
```bash
# Terminal 2
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run drone_control takeoff_land_mission
or
ros2 run drone_control circle_mission
```

---

## 🔴 REAL BOARD MODE (Hardware)

### Step 1: Connect Hardware
- Plug in USB cable to flight controller
- Verify it appears: `ls /dev/tty*`
- Common ports: `/dev/ttyACM0` or `/dev/ttyUSB0`

### Step 2: Launch Command (Default USB)
```bash
ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyACM0:57600
```

### Step 2B: Custom Port or Baudrate
```bash
# Different USB port
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyUSB0:115200

# Faster baudrate
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyACM0:921600

# Telemetry radio
ros2 launch mavros apm.launch.py \
  fcu_url:=udp://192.168.1.157:14550
```

### Step 3: Verify Connection
```bash
# Terminal 2
ros2 echo /mavros/state
# Should show: connected: true, armed: false
```

### Step 4: Run Your Mission
```bash
# Terminal 2
ros2 run drone_control takeoff_land_mission
or
ros2 run drone_control circle_mission
```

---

## 📊 Comparison

| | SITL | Real Board |
|---|------|-----------|
| **Hardware** | None | Flight controller |
| **Connection** | UDP localhost | USB/Serial/Network |
| **Launch** | `iris_runway.launch.py` | `apm.launch.py` |
| **Topics** | Same ✓ | Same ✓ |
| **Your Code** | Works ✓ | Works ✓ (no changes) |
| **Risk** | None | Medium (propellers!) |

---

## 🔧 Troubleshooting

### "Cannot open port"
```bash
# Fix permissions
sudo usermod -a -G dialout $USER
# Then logout and login, or:
newgrp dialout
```

### "Connection timeout"
```bash
# Try different baudrate
ros2 launch mavros apm.launch.py \
  fcu_url:=/dev/ttyACM0:57600
```

### Check what port is being used
```bash
# See all tty devices
ls -la /dev/tty* | grep -E "(ACM|USB)"

# See what's using a port
lsof /dev/ttyACM0
```

---

## 📍 Topics (Same in Both Modes)

```bash
# Control commands
/mavros/cmd/command                   # Takeoff command service backend
/mavros/set_mode                      # Set flight mode

# Feedback
/mavros/local_position/pose           # Where are we
/mavros/state                         # Armed? Connected?
/mavros/sys_status                    # Battery, health
/mavros/imu/data                      # Sensor data
```

---

## 🚀 Full Workflow

### Option A: Test then Deploy
```bash
# Terminal 1: Start SITL
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# Terminal 2: Test code
ros2 run drone_control takeoff_land_mission

# ✅ If it works, switch to real board:

# Terminal 1 (press Ctrl+C first): Start real board
ros2 launch mavros apm.launch.py

# Terminal 2: Same command, works with real board!
ros2 run drone_control takeoff_land_mission
```

### Option B: Parallel Testing
```bash
# Terminal 1: SITL on default ROS_DOMAIN_ID
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# Terminal 2: Real board on different domain
export ROS_DOMAIN_ID=1
ros2 launch mavros apm.launch.py

# Now topics are separate, can test both simultaneously!
```

---

## ⚠️ Safety for Real Board

Before flying with real hardware:

1. **Remove propellers** for initial testing
2. **Test arm/disarm** in MANUAL mode
3. **Verify battery** is charged
4. **Check motor directions** (use QGroundControl)
5. **Test STABILIZE** mode first
6. **Only then test GUIDED** mode with low altitude

```bash
# Safe command sequence:
# 1. Arm in MANUAL
# 2. Switch to STABILIZE
# 3. Switch to GUIDED
# 4. Command takeoff to 1m
# 5. Verify mission climbs then switches to LAND
# 6. Land before battery critical
```

### Current Run Sequence (Important)
```bash
# Terminal 1: bring up MAVROS (SITL or real board)
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
# OR
ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyACM0:57600

# Terminal 2: run mission node
cd /home/an/ros2_project/ardupilot/mavros_sw_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run drone_control takeoff_land_mission
```

If Terminal 2 prints `Waiting for set_mode service...`, MAVROS is not up yet or not connected to FCU.

---

## 📚 Full Documentation
See `REAL_BOARD_CONNECTION.md` for detailed troubleshooting and advanced configurations.
