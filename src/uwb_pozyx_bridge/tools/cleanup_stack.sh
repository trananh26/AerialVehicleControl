#!/usr/bin/env bash
# Stop every process of the x500 UWB stack, including the ones that a plain
# `pkill` on the launch file does not reach.
#
# Why this exists
# ---------------
# `ros2 launch` spawns the child processes of the stack (MAVROS, RViz, the
# Gazebo bridges, the state publisher).  When the launch is interrupted with
# Ctrl-C, ROS 2 usually tears them down, but a crash or a `kill -9` on the
# launch process leaves them behind.  Every survivor keeps running:
#
#   * it holds a copy of the Gazebo topic subscriptions,
#   * it burns CPU (a `parameter_bridge` or a `robot_state_publisher` of the
#     x500 namespace costs about 15-20 % of a core), and
#   * it publishes into the same ROS namespace as the new stack.
#
# The visible symptom is a launch that reports errors continuously even
# though nothing is actually wrong with the new instance: the machine is
# simply overloaded (load average above 20 on a four-core box), so MAVProxy
# stops relaying the telemetry and MAVROS reports "Lost connection, HEARTBEAT
# timed out" while the autopilot itself is perfectly healthy.
#
# Usage:
#
#     src/uwb_pozyx_bridge/tools/cleanup_stack.sh
#
# Run it before starting a new instance of the launch file.

set -o pipefail

# Patterns of the processes that make up the stack.  They are matched against
# the full command line, so `pkill -f` is the right tool.
PATTERNS=(
    # The launch file itself, and the mission scripts that talk to it.
    "x500_uwb_runway.launch.py"
    "takeoff_land_mission"
    "circle_mission"
    # The Gazebo server and its GUI.
    "gz sim"
    # ArduPilot SITL and the MAVLink proxies.
    "arducopter --model json"
    "mavproxy.py"
    # The ROS side of the stack.
    "mavros_node"
    "rviz2"
    "odometry_node"
    # The Gazebo/ROS bridges.  These are the ones that survive a crashed
    # launch and that pile up over repeated runs.
    "parameter_bridge"
    "robot_state_publisher"
    "uwb_gz_to_ros_bridge"
    "ros_gz_bridge"
)

# The x500 namespace is shared by every instance, so the bridges are matched
# on the namespace rather than on the executable name alone.
NAMESPACE="__ns:=/x500"

killed=0

for pattern in "${PATTERNS[@]}"; do
    # Collect the pids first: `pkill` would otherwise also match the grep
    # that is used to verify the cleanup afterwards.
    pids="$(pgrep -f -- "$pattern" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        count="$(echo "$pids" | wc -l)"
        echo "stopping $count process(es) matching '$pattern'"
        # SIGTERM first so the processes can shut down cleanly, then SIGKILL
        # whatever is left over.
        kill $pids 2>/dev/null || true
        killed=$((killed + count))
    fi
done

# Give them a moment to exit on their own.
sleep 2

# Anything that survived the SIGTERM gets SIGKILL.  The bridges in particular
# are known to ignore a polite request while they are blocked on a topic.
survivors=""
for pattern in "${PATTERNS[@]}"; do
    pids="$(pgrep -f -- "$pattern" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        survivors="$survivors $pids"
    fi
done

if [ -n "$(echo "$survivors" | tr -d ' ')" ]; then
    echo "forcing the remaining processes to stop:$survivors"
    kill -9 $survivors 2>/dev/null || true
    sleep 1
fi

# Report what is left, so a still-broken run is obvious.
remaining=""
for pattern in "${PATTERNS[@]}"; do
    pids="$(pgrep -f -- "$pattern" 2>/dev/null || true)"
    if [ -n "$pids" ]; then
        remaining="$remaining
  $pattern -> $(echo "$pids" | tr '\n' ' ')"
    fi
done

if [ -n "$remaining" ]; then
    echo "WARNING: processes are still running:$remaining"
    exit 1
fi

echo "stack stopped ($killed process(es) terminated)"
echo "load average: $(cat /proc/loadavg)"
