#!/usr/bin/env bash
# Phase 3 integration test: the Pozyx bridge node driving a real ArduPilot
# SITL instance, with a fake ranging source standing in for Gazebo.
#
# This isolates the node from the Gazebo plugin, so a failure here points at
# the node, the protocol or the parameter set rather than at the simulation.
#
# Usage:  ./run_tier3_verify.sh [duration_seconds]

set -uo pipefail

DURATION="${1:-40}"
ROOT="/home/an/ros2_project/AerialVehicleControl"
AP="$ROOT/src/ardupilot"
PKG="$ROOT/src/uwb_pozyx_bridge"

SITL_BIN="$AP/build/sitl/bin/arducopter"
PARM="$PKG/config/beacon.parm"

SITL_LOG=/tmp/sitl_tier3.log
SENDER_LOG=/tmp/tier3_node.log
MONITOR_LOG=/tmp/tier3_monitor.log
SOURCE_LOG=/tmp/tier3_source.log

cleanup() {
    pkill -f fake_ranging_source 2>/dev/null
    pkill -f uwb_pozyx_bridge.pozyx_node 2>/dev/null
    pkill -f sitl_beacon_monitor 2>/dev/null
    pkill -f "arducopter --model x500 --defaults" 2>/dev/null
    sleep 1
}
trap cleanup EXIT

cleanup
rm -f "$SITL_LOG" "$SENDER_LOG" "$MONITOR_LOG" "$SOURCE_LOG"

export PYTHONPATH="$PKG:${PYTHONPATH:-}"

echo "=== starting SITL ==="
"$SITL_BIN" --model x500 --defaults "$PARM" --serial1=tcp:5761:wait \
    > "$SITL_LOG" 2>&1 &

# Wait for the MAVLink port to be listening.
for _ in $(seq 1 40); do
    if grep -q "SERIAL0 on TCP port 5760" "$SITL_LOG" 2>/dev/null; then
        break
    fi
    sleep 0.5
done

echo "=== starting MAVLink monitor (sets beacon params) ==="
python3 -u "$PKG/tools/sitl_beacon_monitor.py" \
    --mavlink-port 5760 --duration "$DURATION" --set-params \
    > "$MONITOR_LOG" 2>&1 &

# Give the monitor time to set the parameters and let SITL bind serial1.
sleep 6

echo "=== starting fake ranging source ==="
python3 -u "$PKG/tools/fake_ranging_source.py" \
    --topic /uwb/ranging_ros --rate 20 --static --position 0 0 \
    > "$SOURCE_LOG" 2>&1 &

sleep 2

echo "=== starting Pozyx bridge node ==="
python3 -u -m uwb_pozyx_bridge.pozyx_node \
    --ros-args \
    -p input_topic:=/uwb/ranging_ros \
    -p output_pose_topic:=/uwb/pose \
    -p serial_host:=127.0.0.1 \
    -p serial_port:=5761 \
    -p send_rate:=20.0 \
    -p position_accuracy_m:=0.05 \
    > "$SENDER_LOG" 2>&1 &

# Let the pipeline run.
sleep "$DURATION"

echo
echo "############ SITL LOG ############"
tail -30 "$SITL_LOG"
echo
echo "############ NODE LOG ############"
tail -30 "$SENDER_LOG"
echo
echo "############ RANGING SOURCE LOG ############"
tail -10 "$SOURCE_LOG"
echo
echo "############ MONITOR LOG ############"
cat "$MONITOR_LOG"
