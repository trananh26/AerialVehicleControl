#!/usr/bin/env bash
# Verify the whole x500 UWB + Pozyx pipeline.
#
# The pipeline is brought up by a single launch file, which is the same one
# that is used for interactive work:
#
#     ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py
#
# Three gates are checked:
#
#   1. FDM coupling   - SITL actually drives the Gazebo simulation.  The
#                        vehicle has to climb in Gazebo, which is only
#                        possible when SITL is connected over the FDM port.
#   2. Position       - the Gazebo truth, the trilateration and the EKF
#                        estimate have to agree.
#   3. GCS link       - MAVProxy has to forward the telemetry to the UDP
#                        port Mission Planner connects to.
#
# Everything is read from MAVProxy's UDP output.  SITL's MAVLink port
# serves a single TCP client and MAVProxy already holds it, so connecting
# to port 5760 directly only works when MAVProxy is not running.

# Note: no `set -u`; the ROS setup scripts reference unbound variables.
set -o pipefail

SETTLE="${1:-30}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG="/tmp/uwb_pipeline_verify.log"

cleanup() {
    pkill -f "x500_uwb_runway.launch.py" 2>/dev/null
    pkill -f "gz sim.*uwb_runway" 2>/dev/null
    pkill -f uwb_gz_to_ros_bridge 2>/dev/null
    pkill -f uwb_pozyx_bridge.pozyx_node 2>/dev/null
    pkill -f compare_positions 2>/dev/null
    pkill -f verify_fdm_coupling 2>/dev/null
    pkill -f "arducopter --model json" 2>/dev/null
    pkill -f mavproxy.py 2>/dev/null
    sleep 2
}
trap cleanup EXIT

cleanup
rm -f "$LOG"

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

echo "=== launching the pipeline ==="
stdbuf -oL -eL timeout $((SETTLE + 300)) \
    ros2 launch uwb_pozyx_bridge x500_uwb_runway.launch.py \
    > "$LOG" 2>&1 &
LAUNCH_PID=$!

# Wait for the EKF to have a position estimate and for MAVProxy to be
# forwarding.  The beacon frames start flowing once the Pozyx node has
# connected to SITL's serial port.
echo "=== waiting for the stack to come up ==="
for _ in $(seq 1 60); do
    if grep -q "is using range beacons" "$LOG" 2>/dev/null; then
        break
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "the launch died, see $LOG"
        exit 1
    fi
    sleep 2
done

if ! grep -q "is using range beacons" "$LOG" 2>/dev/null; then
    echo "the EKF never switched to the range beacons, see $LOG"
    tail -40 "$LOG"
    exit 1
fi
echo "the EKF is using the range beacons"

# ---------------------------------------------------------------------
# Gate 1: the three position sources agree.
#
# This runs first, while the vehicle is still on the ground.  The FDM
# gate flies the vehicle away afterwards, and a moving vehicle is a much
# worse case for the comparison than a parked one.
# ---------------------------------------------------------------------
echo
echo "=== gate 1: the three position sources agree ==="
python3 -u "$ROOT/src/uwb_pozyx_bridge/tools/compare_positions.py" \
    --udp-port 14550 --duration 15
COMPARE_STATUS=$?

# ---------------------------------------------------------------------
# Gate 2: FDM coupling.
#
# This one flies the vehicle, so it runs last.  The vehicle is left
# disarmed and back on the ground when it finishes.
# ---------------------------------------------------------------------
echo
echo "=== gate 2: SITL drives the Gazebo simulation ==="
python3 -u "$ROOT/src/uwb_pozyx_bridge/tools/verify_fdm_coupling.py" \
    --udp-port 14550 --settle "$SETTLE"
FDM_STATUS=$?

# ---------------------------------------------------------------------
# Gate 3: the GCS link.
# ---------------------------------------------------------------------
echo
echo "=== gate 3: MAVProxy forwards the telemetry to the GCS port ==="
python3 -u - <<'PYEOF'
import sys
from pymavlink import mavutil

status = 0
for port in (14550, 14551):
    mav = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{port}", source_system=255, source_component=0
    )
    msg = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    mav.close()
    if msg is None:
        print(f"  UDP {port}: NO DATA")
        status = 1
    else:
        print(f"  UDP {port}: heartbeat from system {msg.get_srcSystem()}")
sys.exit(status)
PYEOF
GCS_STATUS=$?

echo
echo "############ LAUNCH LOG (beacon and error lines) ############"
grep -E "origin set|range beacons|initial pos|initial beacon|error|Error|ERROR|Traceback" \
    "$LOG" | tail -25

echo
echo "=== summary ==="
[ $FDM_STATUS -eq 0 ] && echo "  FDM coupling : PASSED" || echo "  FDM coupling : FAILED"
[ $COMPARE_STATUS -eq 0 ] && echo "  positions    : PASSED" || echo "  positions    : FAILED"
[ $GCS_STATUS -eq 0 ] && echo "  GCS link     : PASSED" || echo "  GCS link     : FAILED"

if [ $FDM_STATUS -ne 0 ] || [ $COMPARE_STATUS -ne 0 ] || [ $GCS_STATUS -ne 0 ]; then
    exit 1
fi
echo
echo "PIPELINE: PASSED"
