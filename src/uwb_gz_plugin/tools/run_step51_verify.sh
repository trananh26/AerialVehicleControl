#!/usr/bin/env bash
# Step 5.1 gate: verify the Gazebo -> ROS 2 side of the UWB pipeline.
#
# Checks, in order:
#   1. the bridge node publishes /uwb/ranging_ros and /uwb/tag_pose_ros
#   2. the ranging topic runs at roughly 20 Hz
#   3. every message carries a plausible range, an anchor id and a LOS code
#   4. the tag pose topic reports the real vehicle position
#   5. the tag pose does not leak into the ranging topic
#
# ArduPilot is NOT involved in this step.

# Note: no `set -u`; the ROS setup scripts reference unbound variables.
set -o pipefail

ROOT="/home/an/ros2_project/AerialVehicleControl"
PKG="$ROOT/src/uwb_gz_plugin"

GZ_LOG=/tmp/gz_step51.log

cleanup() {
    pkill -f "gz sim.*uwb_runway" 2>/dev/null
    pkill -f uwb_gz_to_ros_bridge 2>/dev/null
    sleep 1
}
trap cleanup EXIT

cleanup
rm -f "$GZ_LOG"

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$ROOT/install/uwb_gz_plugin/share/uwb_gz_plugin:$ROOT/src/ardupilot_gazebo/models:$ROOT/src/ardupilot_gz/ardupilot_gz_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== starting Gazebo (uwb_runway.sdf) ==="
stdbuf -oL -eL timeout 120 gz sim -r -s uwb_runway.sdf > "$GZ_LOG" 2>&1 &

for _ in $(seq 1 40); do
    if gz topic -l 2>/dev/null | grep -q "/uwb/ranging/tag"; then
        break
    fi
    sleep 0.5
done

echo "=== spawning the tag vehicle at the centre, (5, 5, 0.1) ==="
# The pose must sit on the <include> element; see docs R14.
gz service -s /world/uwb_runway/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'name: "x500", sdf: "<sdf version=\"1.9\"><model name=\"x500\"><include><uri>model://x500_base</uri><pose>5 5 0.1 0 0 0</pose></include></model></sdf>"'

sleep 3

echo "=== starting the ROS 2 bridge ==="
stdbuf -oL -eL timeout 90 ros2 run uwb_gz_plugin uwb_gz_to_ros_bridge \
    > /tmp/step51_bridge.log 2>&1 &

sleep 6

echo
echo "=== ROS 2 topics ==="
ros2 topic list | grep uwb

echo
echo "=== running the ranging stream gate checker ==="
python3 -u "$PKG/tools/check_ranging_stream.py" --duration 10
GATE_STATUS=$?

echo
echo "=== sample ranging messages ==="
timeout 6 ros2 topic echo /uwb/ranging_ros --qos-reliability best_effort 2>&1 \
    | head -32

echo
echo "=== sample tag pose ==="
timeout 6 ros2 topic echo /uwb/tag_pose_ros --qos-reliability best_effort 2>&1 \
    | head -10

echo
echo "=== bridge log ==="
cat /tmp/step51_bridge.log

exit $GATE_STATUS
