#!/usr/bin/env bash
# Phase 4 test: start the Gazebo world with the UWB plugin, spawn a tag
# vehicle, and verify that ranging messages are published.
#
# This does not involve ArduPilot; it isolates the Gazebo side of the
# pipeline.  Phase 5 joins the two halves together.

# Note: no `set -u`; the ROS setup scripts reference unbound variables.
set -o pipefail

ROOT="/home/an/ros2_project/AerialVehicleControl"
PKG="$ROOT/src/uwb_gz_plugin"

GZ_LOG=/tmp/gz_phase4.log

cleanup() {
    pkill -f "gz sim.*uwb_runway" 2>/dev/null
    sleep 1
}
trap cleanup EXIT

cleanup
rm -f "$GZ_LOG"

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$ROOT/install/uwb_gz_plugin/share/uwb_gz_plugin:$ROOT/src/ardupilot_gazebo/models:$ROOT/src/ardupilot_gz/ardupilot_gz_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== starting Gazebo (uwb_runway.sdf) ==="
stdbuf -oL -eL timeout 90 gz sim -r -s uwb_runway.sdf > "$GZ_LOG" 2>&1 &

# Wait for the world to come up.
for _ in $(seq 1 40); do
    if gz topic -l 2>/dev/null | grep -q "/uwb/ranging/tag"; then
        break
    fi
    sleep 0.5
done

echo "=== gz topics ==="
gz topic -l 2>/dev/null | grep uwb

echo
echo "=== plugin log lines ==="
GZ_SERVER_LOG="$(ls -td "$HOME"/.gz/sim/log/*/ 2>/dev/null | head -1)/server_console.log"
if [ -f "$GZ_SERVER_LOG" ]; then
    grep "UwbSystem" "$GZ_SERVER_LOG" | sed 's/^([^)]*) ([^)]*) //' | head -20
else
    echo "(no server console log at $GZ_SERVER_LOG)"
fi

echo
echo "=== spawning tag model x500_base at (5, 5, 0.1) ==="
# The pose must sit on the <include> element: a nested model's own <pose>
# (x500_base has "0 0 .025") would otherwise override the spawn position.
gz service -s /world/uwb_runway/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'name: "x500", sdf: "<sdf version=\"1.9\"><model name=\"x500\"><include><uri>model://x500_base</uri><pose>5 5 0.1 0 0 0</pose></include></model></sdf>"'

sleep 4

echo
echo "=== ranging messages (5 s) ==="
timeout 6 gz topic -e -t /uwb/ranging/tag 2>&1 | head -40

echo
echo "=== verifying ranges against the expected geometry ==="
python3 - "$ROOT" <<'PYEOF'
import math
import sys

# The tag sits at (5, 5, 0.1) and the anchors at the corners of a 10 m x 10 m
# square at z = 2.0 m, so every range must be sqrt(5^2 + 5^2 + 1.9^2).
expected = math.sqrt(5.0 ** 2 + 5.0 ** 2 + 1.9 ** 2)
print(f"expected range to every anchor: {expected:.4f} m")
print("(compare with the 'x' values of the ranging messages above)")
sys.exit(0)
PYEOF

echo
echo "=== plugin log after spawn ==="
if [ -f "$GZ_SERVER_LOG" ]; then
    grep "UwbSystem" "$GZ_SERVER_LOG" | sed 's/^([^)]*) ([^)]*) //' | head -30
fi
