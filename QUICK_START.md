# QUICK START - From Fresh Ubuntu to SITL/Real Board

This guide is written to be reproducible on another machine, without hardcoded personal paths.

## 0. Scope and Assumptions

- Ubuntu: `22.04` (recommended for ROS 2 Humble)
- Shell: `bash`
- Goals:
  - Run SITL simulation with MAVROS
  - Run with a real flight controller through MAVROS


## 1. Install System Dependencies (One Time)

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y \
  curl gnupg2 lsb-release ca-certificates \
  build-essential git \
  python3-pip python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep python3-vcstool
```

## 2. Install ROS 2 Humble

Official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### 2.1 Add ROS 2 apt repository (run once on every machine)

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 2.2 Install ROS package by device role

Laptop / development machine (Desktop):

```bash
sudo apt install -y ros-humble-desktop
```

Companion Computer (Raspberry Pi) (ROS-Base):

```bash
sudo apt install -y ros-humble-ros-base
```

### 2.3 Install MAVROS

If you want MAVROS from apt packages:

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
```

Install GeographicLib datasets (required by MAVROS global position features):

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Note: this repository also includes `mavros` in `workspace.repos`, so MAVROS can be built from source together with the workspace.

### 2.4 Install Gazebo and Required Plugins

Use Gazebo Harmonic only (do not install Gazebo Classic for this setup).

For ROS 2 Humble + Gazebo Harmonic, add Gazebo APT source first:

```bash
sudo apt install -y wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
```

Add Gazebo sources to rosdep for this non-default pairing:

```bash
sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update
```

Install Gazebo Harmonic from official Gazebo docs:

- https://gazebosim.org/docs/harmonic/install

After installation, verify Gazebo is working:

```bash
gz sim -v4 -r shapes.sdf
```

Set Gazebo version environment variable (required by ArduPilot Gazebo plugin build scripts):

```bash
export GZ_VERSION=harmonic
```

Persist it in `~/.bashrc`:

```bash
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
```

Install ROS-GZ integration packages on laptop/dev machine:

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image
```

Install ArduPilot Gazebo plugin dependencies for Harmonic:

```bash
sudo apt install -y \
  libgz-sim8-dev rapidjson-dev \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```



## 3. Create Workspace and Fetch Source Code

```bash
mkdir -p $HOME/ros2_project
cd $HOME/ros2_project

# Clone your main repository (replace with your repository URL)
git clone https://github.com/trananh26/AerialVehicleControl
cd AerialVehicleControl

# Import dependencies listed in workspace.repos
vcs import src < workspace.repos
```

## 4. Install ROS Package Dependencies with rosdep

```bash
sudo rosdep init || true
rosdep update

cd $HOME/ros2_project/AerialVehicleControl
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Build the Workspace

```bash
cd $HOME/ros2_project/AerialVehicleControl
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 6. Source Environment (Every New Terminal)

```bash
source /opt/ros/humble/setup.bash
source $HOME/ros2_project/AerialVehicleControl/install/setup.bash
```

Optional: add to `~/.bashrc`:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source $HOME/ros2_project/AerialVehicleControl/install/setup.bash' >> ~/.bashrc
```

## 7. Run SITL Simulation (No Hardware)

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source $HOME/ros2_project/AerialVehicleControl/install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source $HOME/ros2_project/AerialVehicleControl/install/setup.bash

# Choose one mission
ros2 run drone_control takeoff_land_mission
# or
ros2 run drone_control circle_mission
```

Check MAVROS connection:

```bash
ros2 topic echo /mavros/state
```

Expected: `connected: true`.

## 8. Run with Real Board (Physical Flight Controller)

### 8.1 Preparation

- Connect the flight controller over USB
- Find the serial device:

```bash
ls /dev/tty* | grep -E "(ACM|USB)"
```

Common devices: `/dev/ttyACM0`, `/dev/ttyUSB0`

If serial permission is denied:

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

### 8.2 Launch MAVROS for Real Board

Terminal 1 (default):

```bash
source /opt/ros/humble/setup.bash
source $HOME/ros2_project/AerialVehicleControl/install/setup.bash
ros2 launch ardupilot_gz_bringup mavros_real_board.launch.py
```

Terminal 1 (custom port/baud):

```bash
ros2 launch ardupilot_gz_bringup mavros_real_board.launch.py \
  fcu_url:=/dev/ttyACM0:57600
```

Terminal 2 (run mission):

```bash
source /opt/ros/humble/setup.bash
source $HOME/ros2_project/AerialVehicleControl/install/setup.bash
ros2 run drone_control takeoff_land_mission
# or
ros2 run drone_control circle_mission
```

### 8.3 Quick Validation

```bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/sys_status
```

## 9. Quick Troubleshooting

### 9.1 Build fails due to missing packages

```bash
cd $HOME/ros2_project/AerialVehicleControl
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 9.2 MAVROS cannot connect (`connected: false`)

- Re-check `fcu_url` (correct serial port and baudrate)
- Try another baudrate: `57600`, `115200`, `921600`
- Check if another process is using the port:

```bash
lsof /dev/ttyACM0
```

### 9.3 Mission waits for services

If mission node prints `Waiting for set_mode service...`, MAVROS is either not running yet or not connected to FCU.

## 10. Real Hardware Safety Notes

1. Remove propellers for first-time tests.
2. Check battery and motor direction.
3. Test arm/disarm in a safe mode first.
4. Only fly real hardware after SITL tests pass.

## 11. Command Summary

```bash
# Terminal A: bringup
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
# or
ros2 launch ardupilot_gz_bringup mavros_real_board.launch.py fcu_url:=/dev/ttyACM0:57600

# Terminal B: mission
ros2 run drone_control takeoff_land_mission
# or
ros2 run drone_control circle_mission
```
