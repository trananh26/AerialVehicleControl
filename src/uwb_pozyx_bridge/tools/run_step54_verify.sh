#!/usr/bin/env bash
# Step 5.4 gate: compares the three position estimates.
#
#   truth        the vehicle's real position in Gazebo
#   trilateration the bridge node's own solution
#   ekf          ArduPilot's EKF estimate
#
# All three must agree, otherwise the EKF is fusing the beacon data but
# placing the vehicle in the wrong spot, which means a frame mapping error.

# Note: no `set -u`; the ROS setup scripts reference unbound variables.
set -o pipefail

DURATION="${1:-40}"
ROOT="/home/an/ros2_project/AerialVehicleControl"
AP="$ROOT/src/ardupilot"
PLUGIN="$ROOT/src/uwb_gz_plugin"
BRIDGE="$ROOT/src/uwb_pozyx_bridge"

SITL_BIN="$AP/build/sitl/bin/arducopter"
PARM="$BRIDGE/config/beacon.parm"

cleanup() {
    pkill -f "gz sim.*uwb_runway" 2>/dev/null
    pkill -f uwb_gz_to_ros_bridge 2>/dev/null
    pkill -f uwb_pozyx_bridge.pozyx_node 2>/dev/null
    pkill -f compare_positions 2>/dev/null
    pkill -f sitl_beacon_monitor 2>/dev/null
    pkill -f "arducopter --model x500 --defaults" 2>/dev/null
    sleep 1
}
trap cleanup EXIT

cleanup
rm -f /tmp/step54_*.log

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$ROOT/install/uwb_gz_plugin/share/uwb_gz_plugin:$ROOT/src/ardupilot_gazebo/models:$ROOT/src/ardupilot_gz/ardupilot_gz_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== 1/6 starting Gazebo ==="
stdbuf -oL -eL timeout $((DURATION + 90)) gz sim -r -s uwb_runway.sdf \
    > /tmp/step54_gz.log 2>&1 &

for _ in $(seq 1 40); do
    if gz topic -l 2>/dev/null | grep -q "/uwb/ranging/tag"; then
        break
    fi
    sleep 0.5
done

echo "=== 2/6 spawning the tag vehicle at (5, 5, 0.1) ==="
gz service -s /world/uwb_runway/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'name: "x500", sdf: "<sdf version=\"1.9\"><model name=\"x500\"><include><uri>model://x500_base</uri><pose>5 5 0.1 0 0 0</pose></include></model></sdf>"'

sleep 2

echo "=== 3/6 starting the gz -> ROS 2 bridge ==="
stdbuf -oL -eL timeout $((DURATION + 70)) \
    ros2 run uwb_gz_plugin uwb_gz_to_ros_bridge \
    > /tmp/step54_bridge.log 2>&1 &

echo "=== 4/6 starting ArduPilot SITL ==="
"$SITL_BIN" --model x500 --defaults "$PARM" --serial1=tcp:5761:wait \
    > /tmp/step54_sitl.log 2>&1 &

for _ in $(seq 1 40); do
    if grep -q "SERIAL0 on TCP port 5760" /tmp/step54_sitl.log 2>/dev/null; then
        break
    fi
    sleep 0.5
done

echo "=== 5/6 setting the beacon parameters ==="
timeout $((DURATION + 50)) python3 -u "$BRIDGE/tools/sitl_beacon_monitor.py" \
    --mavlink-port 5760 --duration "$((DURATION + 30))" --set-params \
    > /tmp/step54_monitor.log 2>&1 &
MONITOR_PID=$!

sleep 8

echo "=== 6/6 starting the Pozyx bridge node ==="
stdbuf -oL -eL timeout $((DURATION + 60)) \
    python3 -u -m uwb_pozyx_bridge.pozyx_node \
    --ros-args \
    -p input_topic:=/uwb/ranging_ros \
    -p output_pose_topic:=/uwb/pose \
    -p serial_host:=127.0.0.1 \
    -p serial_port:=5761 \
    -p send_rate:=20.0 \
    -p position_accuracy_m:=0.05 \
    > /tmp/step54_node.log 2>&1 &

# Give the EKF time to align before comparing.
sleep 20

echo
echo "=== comparing the three position sources ==="
python3 -u "$BRIDGE/tools/compare_positions.py" \
    --mavlink-port 5760 --duration "$DURATION"
COMPARE_STATUS=$?

echo
echo "############ NODE LOG ############"
tail -6 /tmp/step54_node.log

exit $COMPARE_STATUS
