# ROS 2 Humble + Gazebo Harmonic + ArduPilot + MAVROS Setup Guide (Ubuntu 22.04)

This guide is rewritten from your step list, reordered for a clean end-to-end installation.

## 1. System update

```bash
sudo apt update
sudo apt upgrade -y
```

## 2. Configure locale (UTF-8)

```bash
locale  # check current locale

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify
```

## 3. Install ROS 2 Humble (Debian packages)

Reference: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade -y

sudo apt install -y ros-humble-desktop ros-dev-tools python3-vcstool
```

Add ROS to shell startup:

```bash
grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. Install Gazebo Harmonic

Reference: <https://gazebosim.org/docs/harmonic/install_ubuntu/>

```bash
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg wget

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y gz-harmonic
```

Set Gazebo version in shell:

```bash
grep -qxF "export GZ_VERSION=harmonic" ~/.bashrc || echo "export GZ_VERSION=harmonic" >> ~/.bashrc
source ~/.bashrc
```

## 5. Add Gazebo rosdep source and install extra dependencies

Reference: <https://ardupilot.org/dev/docs/ros2-gazebo.html>

```bash
sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
  -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
```

Initialize rosdep (run only once per machine):

```bash
sudo rosdep init || true
rosdep update
```

Install additional development libraries:

```bash
sudo apt update
sudo apt install -y \
  libgz-sim8-dev rapidjson-dev \
  libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

## 6. Install MAVROS

```bash
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo apt install -y geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

## 7. Create workspace and import source code

Install `vcstool` if not already installed:

```bash
sudo apt install -y python3-vcstool
```

Clone workspace và import tất cả repos:

```bash
# Clone workspace
git clone --recurse-submodules -b feat/x500-arm-submodules \
  https://github.com/trananh26/AerialVehicleControl.git
```

Install ArduPilot prerequisites:

```bash
cd ~/AerialVehicleControl/src/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y

rm -f ~/AerialVehicleControl/src/ardupilot/.lock-waf_linux_build

# Reload environment
. ~/.profile
```

## 8. Install ROS dependencies and build with colcon

Reference: <https://ardupilot.org/dev/docs/ros2.html#ros2>

```bash
cd ~/AerialVehicleControl

source ~/.bashrc
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-up-to ardupilot_gz_bringup drone_control
```

Source workspace:

```bash
source ~/AerialVehicleControl/install/setup.bash
```

## 9. Launch simulation

```bash
ros2 launch ardupilot_gz_bringup x500_runway.launch.py
```

## 10. Run `drone_control` circle mission

After the simulator is running, open a new terminal and source the environments:

```bash
source /opt/ros/humble/setup.bash
source ~/AerialVehicleControl/install/setup.bash
```

Run the mission node:

```bash
ros2 run drone_control circle_mission
```

Note:
1. Keep the launch terminal (`x500_runway.launch.py`) running while this node is active.

## Notes

1. `rosdep init` should be run once per machine.
2. If `ros2` or `gz` commands are missing in a new terminal, run `source ~/.bashrc`.
3. The original `GZ_SIM_*` path examples using `~/gz_ws/...` were replaced with `~/AerialVehicleControl/...` to match this workspace.
