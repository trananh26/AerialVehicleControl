#!/usr/bin/env bash
# Step 5.5 gate: run the whole pipeline from a single `ros2 launch`.
#
# Nothing is started by hand here; the launch file has to bring up the world,
# the vehicle, SITL, the gz -> ROS 2 bridge and the Pozyx node on its own.
#
# The gate then checks the same things the earlier steps checked, so a
# regression in any single component still shows up.

# Note: no `set -u`; the ROS setup scripts reference unbound variables.
set -o pipefail

DURATION="${1:-45}"
ROOT="/home/an/ros2_project/AerialVehicleControl"

cleanup() {
    pkill -f "uwb_pozyx_sitl.launch.py" 2>/dev/null
    pkill -f "gz sim.*uwb_runway" 2>/dev/null
    pkill -f uwb_gz_to_ros_bridge 2>/dev/null
    pkill -f uwb_pozyx_bridge.pozyx_node 2>/dev/null
    pkill -f sitl_beacon_monitor 2>/dev/null
    pkill -f compare_positions 2>/dev/null
    pkill -f "arducopter --model x500 --defaults" 2>/dev/null
    sleep 2
}
trap cleanup EXIT

cleanup
rm -f /tmp/step55_launch.log

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

echo "=== launching the whole pipeline ==="
stdbuf -oL -eL timeout $((DURATION + 150)) \
    ros2 launch uwb_pozyx_bridge uwb_pozyx_sitl.launch.py \
    monitor_duration:=$((DURATION - 15)) \
    > /tmp/step55_launch.log 2>&1 &
LAUNCH_PID=$!

# Give everything time to come up and the EKF time to align.
sleep $((DURATION - 15))

# SITL's MAVLink port serves a single TCP client, so the monitor has to
# release it before the comparison tool can read the EKF position.
echo "=== waiting for the MAVLink monitor to release port 5760 ==="
for _ in $(seq 1 40); do
    if ! pgrep -f sitl_beacon_monitor > /dev/null 2>&1; then
        break
    fi
    sleep 1
done
sleep 3

echo
echo "=== comparing the three position sources ==="
python3 -u "$ROOT/src/uwb_pozyx_bridge/tools/compare_positions.py" \
    --mavlink-port 5760 --duration 15
COMPARE_STATUS=$?

echo
echo "############ LAUNCH LOG (beacon and error lines) ############"
grep -E "origin set|range beacons|initial pos|initial beacon|error|Error|ERROR|Traceback" \
    /tmp/step55_launch.log | tail -25

echo
echo "############ spawn check ############"
grep -E "spawning|entity|x500" /tmp/step55_launch.log | head -8

exit $COMPARE_STATUS
