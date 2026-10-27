#!/usr/bin/env bash
# End-to-end Tier 2 verification of the Pozyx UWB beacon pipeline.
#
# Starts ArduPilot SITL with a beacon serial port, connects a MAVLink client to
# release SITL's start-up wait, then feeds the Pozyx wire protocol from a fake
# Arduino so that the EKF3 range-beacon fusion can be observed.
#
# Usage:  ./run_tier2_verify.sh [duration_seconds]

set -uo pipefail

DURATION="${1:-40}"
ROOT="/home/an/ros2_project/AerialVehicleControl"
AP="$ROOT/src/ardupilot"
PKG="$ROOT/src/uwb_pozyx_bridge"

SITL_BIN="$AP/build/sitl/bin/arducopter"
PARM="$PKG/config/beacon.parm"

SITL_LOG=/tmp/sitl_beacon.log
SENDER_LOG=/tmp/fake_sender.log
MONITOR_LOG=/tmp/monitor.log

cleanup() {
    pkill -f fake_pozyx_sender 2>/dev/null
    pkill -f sitl_beacon_monitor 2>/dev/null
    pkill -f "arducopter --model x500 --defaults" 2>/dev/null
    sleep 1
}
trap cleanup EXIT

cleanup
rm -f "$SITL_LOG" "$SENDER_LOG" "$MONITOR_LOG"

echo "=== starting SITL ==="
"$SITL_BIN" --model x500 --defaults "$PARM" --serial1=tcp:5761:wait \
    > "$SITL_LOG" 2>&1 &
SITL_PID=$!

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
MONITOR_PID=$!

# Give the monitor time to set the parameters and let SITL bind serial1.
sleep 6

echo "=== starting fake Pozyx sender ==="
python3 -u "$PKG/tools/fake_pozyx_sender.py" \
    --port 5761 --duration "$DURATION" --position-error "${POS_ERR:-50}" ${SENDER_EXTRA:-} \
    > "$SENDER_LOG" 2>&1 &
SENDER_PID=$!

# Let the pipeline run.
sleep "$DURATION"

echo
echo "############ SITL LOG ############"
tail -40 "$SITL_LOG"
echo
echo "############ SENDER LOG ############"
tail -20 "$SENDER_LOG"
echo
echo "############ MONITOR LOG ############"
cat "$MONITOR_LOG"
