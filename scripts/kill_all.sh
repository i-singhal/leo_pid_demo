#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# kill_all.sh — Clean shutdown of all ROS 2 / Gazebo processes
# ═══════════════════════════════════════════════════════════════
# Usage:
#   bash kill_all.sh
#   OR (if made executable):
#   ./kill_all.sh
#
# What it does:
#   1. Sends SIGTERM (polite "please stop") to all ROS/Gazebo processes
#   2. Waits 3 seconds for them to shut down gracefully
#   3. Sends SIGKILL (forced kill) to anything still alive
#   4. Clears leftover shared memory from DDS
# ═══════════════════════════════════════════════════════════════

echo "=== Stopping all ROS 2 and Gazebo processes ==="

# ── Phase 1: SIGTERM (polite shutdown) ───────────────────────
# pkill searches for running processes by name and sends them
# a signal. -f means "match against the full command line, not
# just the process name." Without -f, it might miss processes
# whose name doesn't exactly match (like "gz sim server").
#
# The "2>/dev/null" hides error messages when no matching
# process is found (otherwise you'd see ugly "no process"
# warnings for things that aren't running).

echo "Sending SIGTERM (polite stop)..."

pkill -f "gz sim" 2>/dev/null
pkill -f "gazebo" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "pid_waypoint_follower" 2>/dev/null
pkill -f "ros2" 2>/dev/null
pkill -f "xterm" 2>/dev/null

# ── Phase 2: Wait ───────────────────────────────────────────
# Give processes 3 seconds to shut down gracefully. Some
# processes (especially Gazebo's rendering engine) need time
# to release GPU resources and close shared memory.

echo "Waiting 3 seconds for graceful shutdown..."
sleep 3

# ── Phase 3: SIGKILL (force kill) ───────────────────────────
# -9 means send signal 9 (SIGKILL). Any process that ignored
# SIGTERM gets forcefully terminated by the operating system.
# The process cannot catch or ignore SIGKILL.

echo "Sending SIGKILL (force kill) to survivors..."

pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gazebo" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "pid_waypoint_follower" 2>/dev/null
pkill -9 -f "xterm" 2>/dev/null

# ── Phase 4: Clean up shared memory ─────────────────────────
# DDS (the communication layer ROS 2 uses) creates shared
# memory files in /dev/shm/. If processes die without cleaning
# up, these files linger and can cause issues on next launch.
# This removes any leftover DDS shared memory segments owned
# by your user.

echo "Cleaning up DDS shared memory..."
rm -f /dev/shm/fastrtps_* 2>/dev/null
rm -f /dev/shm/Fast* 2>/dev/null

# ── Done ─────────────────────────────────────────────────────
echo "=== All clean ==="
echo ""

# Show if anything is still running (should be empty)
remaining=$(ps aux | grep -E "gz|gazebo|rviz|parameter_bridge|pid_waypoint" | grep -v grep | grep -v kill_all)
if [ -z "$remaining" ]; then
    echo "No ROS/Gazebo processes remaining."
else
    echo "WARNING — these processes survived:"
    echo "$remaining"
    echo "You may need to kill them manually with: kill -9 <PID>"
fi