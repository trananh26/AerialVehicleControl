#!/usr/bin/env bash
# Step 5.3 gate: Gazebo -> ROS 2 -> Pozyx -> ArduPilot SITL.
#
# This is the first step where ArduPilot is involved.  The vehicle still sits
# on the ground; what is being checked is that the EKF accepts the beacon
# data and starts fusing it.
#
# Processes started, in order:
#   1. Gazebo with the UWB world
#   2. the tag vehicle, spawned at the centre of the anchor square
#   3. the gz -> ROS 2 bridge
#   4. ArduPilot SITL, with the beacon parameters
#   5. the MAVLink monitor, which sets the parameters
#   6. the Pozyx bridge node, which drives the serial port

# Note: no `set -u`; the ROS setup scripts reference unbound variables.
set -o pipefail

DURATION="${1:-50}"
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
    pkill -f sitl_beacon_monitor 2>/dev/null
    pkill -f "arducopter --model x500 --defaults" 2>/dev/null
    sleep 1
}
trap cleanup EXIT

cleanup
rm -f /tmp/step53_sitl.log /tmp/step53_node.log /tmp/step53_monitor.log

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="$ROOT/install/uwb_gz_plugin/share/uwb_gz_plugin:$ROOT/src/ardupilot_gazebo/models:$ROOT/src/ardupilot_gz/ardupilot_gz_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "=== 1/6 starting Gazebo ==="
stdbuf -oL -eL timeout $((DURATION + 60)) gz sim -r -s uwb_runway.sdf \
    > /tmp/step53_gz.log 2>&1 &

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
stdbuf -oL -eL timeout $((DURATION + 40)) \
    ros2 run uwb_gz_plugin uwb_gz_to_ros_bridge \
    > /tmp/step53_bridge.log 2>&1 &

echo "=== 4/6 starting ArduPilot SITL ==="
"$SITL_BIN" --model x500 --defaults "$PARM" --serial1=tcp:5761:wait \
    > /tmp/step53_sitl.log 2>&1 &

for _ in $(seq 1 40); do
    if grep -q "SERIAL0 on TCP port 5760" /tmp/step53_sitl.log 2>/dev/null; then
        break
    fi
    sleep 0.5
done

echo "=== 5/6 setting the beacon parameters ==="
timeout $((DURATION + 40)) python3 -u "$BRIDGE/tools/sitl_beacon_monitor.py" \
    --mavlink-port 5760 --duration "$DURATION" --set-params \
    > /tmp/step53_monitor.log 2>&1 &
MONITOR_PID=$!

# Let the monitor set the parameters and SITL bind serial1.
sleep 8

echo "=== 6/6 starting the Pozyx bridge node ==="
stdbuf -oL -eL timeout $((DURATION + 30)) \
    python3 -u -m uwb_pozyx_bridge.pozyx_node \
    --ros-args \
    -p input_topic:=/uwb/ranging_ros \
    -p output_pose_topic:=/uwb/pose \
    -p serial_host:=127.0.0.1 \
    -p serial_port:=5761 \
    -p send_rate:=20.0 \
    -p position_accuracy_m:=0.05 \
    > /tmp/step53_node.log 2>&1 &

# The monitor runs for DURATION seconds and prints its summary at the end;
# wait for it so that the log is complete before the gate check.
wait $MONITOR_PID 2>/dev/null
sleep 2

echo
echo "############ GATE CHECK ############"
# The beacon status messages arrive at the very end of the run, so the gate
# retries for a while rather than reading the log a single time.
python3 - <<'PYEOF'
import re
import sys
import time


def read(path):
    try:
        with open(path, errors="replace") as handle:
            return handle.read()
    except OSError:
        return ""


NEEDED = [
    ("EKF3 origin set", r"origin set"),
    ("EKF3 is using range beacons", r"is using range beacons"),
    ("initial pos NE is ~0", r"initial pos NE = -?0\.0,-?0\.0"),
    ("beacon pos D offset is 0.1", r"beacon pos D offset = 0\.1"),
]

blob = ""
for _ in range(30):
    blob = read("/tmp/step53_sitl.log") + read("/tmp/step53_monitor.log")
    if all(re.search(pattern, blob) for _, pattern in NEEDED):
        break
    time.sleep(2)

failed = []
for name, pattern in NEEDED:
    ok = re.search(pattern, blob) is not None
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}")
    if not ok:
        failed.append(name)

monitor = read("/tmp/step53_monitor.log")
node = read("/tmp/step53_node.log")

# The node must not be dropping frames.
counters = re.findall(r"frames_sent=(\d+) dropped=(\d+)", node)
if counters:
    sent, dropped = counters[-1]
    print(f"  [{'OK ' if dropped == '0' else 'FAIL'}] "
          f"node frames_sent={sent} dropped={dropped}")
    if dropped != "0":
        failed.append("node dropped frames")
else:
    print("  [??  ] no frame counters in the node log")
    failed.append("the node did not report its frame counters")

# The EKF must have left the uninitialised state.  Note that
# EKF_UNINITIALIZED is 1024, i.e. bit 10, not bit 8.
EKF_UNINITIALIZED = 1024
flags = re.findall(r"flags=0x([0-9a-f]+)", monitor)
if flags:
    last = int(flags[-1], 16)
    uninitialised = bool(last & EKF_UNINITIALIZED)
    print(f"  [{'OK ' if not uninitialised else 'FAIL'}] "
          f"final EKF flags 0x{flags[-1]} "
          f"({'healthy' if not uninitialised else 'still uninitialised'})")
    if uninitialised:
        failed.append("the EKF is still uninitialised")
else:
    print("  [??  ] no EKF status reports in the monitor log")
    failed.append("no EKF status reports")

print()
if failed:
    for name in failed:
        print(f"FAILED: {name}")
    print("STEP 5.3 GATE: FAILED")
    sys.exit(1)

print("STEP 5.3 GATE: PASSED")
PYEOF
GATE_STATUS=$?

echo
echo "############ SITL LOG ############"
tail -20 /tmp/step53_sitl.log
echo
echo "############ NODE LOG ############"
tail -12 /tmp/step53_node.log
echo
echo "############ MONITOR LOG (beacon lines) ############"
grep -E "statustext|ekf\]" /tmp/step53_monitor.log | tail -25

exit $GATE_STATUS
