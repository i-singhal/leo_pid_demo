# Leo PID Demo

PID waypoint follower for the Leo Rover in Gazebo Harmonic simulation. Uses ground truth pose from Gazebo (not odometry) to avoid skid-steer drift, with RViz visualization of waypoints, path trail, and goal markers. Includes an obstacle world with a 3D overhead LiDAR providing terrain point cloud data.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     master_launch.py                            │
│  Starts all processes with timed delays, coordinated shutdown   │
└────┬──────────┬──────────────┬─────────────┬───────────────┬────┘
     │          │              │             │               │
     ▼          ▼              ▼             ▼               ▼
 Gazebo    GT Bridge     PID Node      RViz (nav)     RViz (lidar)
 (sim)     (pose)     (controller)    (waypoints)    (point cloud)
     │          │              │             ▲               ▲
     │          │              │             │               │
     │          └──► TFMessage ┘             │               │
     │               (robot pose)            │               │
     │                         │             │               │
     │                    ┌────┘             │               │
     │                    ▼                  │               │
     │              PID control              │               │
     │                    │                  │               │
     │         ┌──────────┼──────────┐       │               │
     │         ▼          ▼          ▼       │               │
     │    /cmd_vel   /waypoint   /robot      │               │
     │         │     _markers    _path       │               │
     │         │         └──────────┘────────┘               │
     ◄─────────┘                                             │
     │                                                       │
     └──► /overhead_lidar/points ──► LiDAR Bridge ──────────┘
          (obstacle world only)
```

The system runs a sense-think-act control loop at 20Hz:
1. **Sense** — ground truth bridge translates Gazebo's internal pose data into a ROS 2 TFMessage
2. **Think** — PID node calculates distance and heading errors, runs dual PID controllers
3. **Act** — publishes velocity commands on /cmd_vel, Gazebo moves the robot

### Why ground truth instead of odometry?

The Leo Rover uses skid-steer drive (4 wheels, no steering mechanism). When it turns, the wheels drag sideways on the ground. Odometry calculates position from wheel encoder ticks but cannot measure this sideways slip, causing yaw drift that accumulates with every turn. Ground truth from Gazebo's physics engine gives the robot's exact position with zero drift.

### PID Control

Two independent PID controllers run in parallel:

- **Linear PID** — controls forward speed based on distance to the current waypoint. Gains: Kp=0.6, Ki=0.0, Kd=0.05
- **Angular PID** — controls turning speed based on heading error to the waypoint. Gains: Kp=1.2, Ki=0.0, Kd=0.15

Additional behaviors:
- **Turn-in-place** — if heading error exceeds 20 degrees, forward motion stops and the robot rotates on the spot
- **Final approach slowdown** — within 20cm of the goal, speed scales linearly to zero
- **Derivative filtering** — low-pass filter (alpha=0.3) smooths noisy derivative terms
- **Goal preemption** — a new goal cancels the current path and generates fresh waypoints

### Overhead LiDAR (obstacle world)

A simulated 3D LiDAR mounted at 12m height (simulating a stationary drone), pointed downward with 90 degree pitch rotation. Produces a dense point cloud of the terrain and obstacles below.

- 720 horizontal samples x 32 vertical layers = 23,040 points per scan
- Plus/minus 29 degree vertical spread, 25m max range
- 5Hz update rate
- Bridged from Gazebo's PointCloudPacked to ROS 2's PointCloud2 via ros_gz_bridge

## Prerequisites

- **Ubuntu 24.04**
- **ROS 2 Jazzy** — [install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- **Gazebo Harmonic** — installed with the Leo Simulator packages
- **xterm** — for the interactive waypoint input terminal

### Install dependencies

```bash
# ROS 2 Jazzy (if not already installed)
sudo apt update
sudo apt install ros-jazzy-desktop

# Leo Rover simulator
sudo apt install ros-jazzy-leo-simulator

# xterm (used by the launch file for interactive input)
sudo apt install xterm
```

## Setup

```bash
# Create a workspace
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src

# Clone the Leo Rover packages
git clone https://github.com/LeoRover/leo_common-ros2.git
git clone https://github.com/LeoRover/leo_simulator-ros2.git

# Clone this package
git clone https://github.com/i-singhal/leo_pid_demo.git

# Install any missing ROS dependencies
cd ~/rover_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

## Recommended: set ROS 2 to localhost only

Since everything runs on one machine, restrict ROS 2 communication to localhost. This prevents DDS multicast issues on Wi-Fi:

```bash
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
source ~/.bashrc
```

## Usage

### Launch with empty world (default)

```bash
ros2 launch leo_pid_demo master_launch.py
```

Starts: Gazebo (empty world) then ground truth bridge (5s) then PID node in xterm (7s) then RViz navigation view (8s).

### Launch with obstacle world

```bash
ros2 launch leo_pid_demo master_launch.py world:=obstacles
```

Starts everything above plus: LiDAR bridge (5s), static tf for LiDAR frame (5s), LiDAR cloud node (5s), and a second RViz window for the LiDAR point cloud view (9s). Two RViz windows open — one for navigation, one for the overhead point cloud.

### Send waypoints

Two ways to set goals:

- **Type coordinates** in the xterm window: `2.0 1.0`
- **Click in RViz** using the "2D Goal Pose" button

The robot generates intermediate waypoints spaced 0.5m apart and follows them in sequence, aligning heading between each waypoint.

### Clean shutdown

Press **Ctrl+C** in the launch terminal. The launch system sends SIGTERM to all child processes, waits, then SIGKILL to survivors.

If things get stuck, use the kill script:

```bash
bash ~/rover_ws/src/leo_pid_demo/scripts/kill_all.sh
```

This finds and kills all ROS 2 and Gazebo processes by name, then cleans up DDS shared memory.

### Tune PID parameters at runtime

```bash
ros2 param set /pid_waypoint_follower kp_lin 0.8
ros2 param set /pid_waypoint_follower kp_ang 1.5
```

Or use the graphical tuner:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

All PID gains, speed limits, tolerances, and waypoint spacing are declared as ROS parameters and can be changed without restarting the node.

## Package Structure

```
leo_pid_demo/
├── launch/
│   └── master_launch.py          # One-command launch for everything
├── leo_pid_demo/
│   ├── __init__.py
│   ├── pid_controller.py         # PID waypoint follower node
│   └── lidar_cloud_node.py       # Republishes LiDAR with correct frame_id
├── worlds/
│   └── leo_obstacles.sdf         # Obstacle world with 3D overhead LiDAR
├── scripts/
│   └── kill_all.sh               # Emergency kill for all ROS/Gazebo processes
├── waypoint_follower.rviz        # RViz config — navigation view
├── lidar_view.rviz               # RViz config — LiDAR point cloud view
├── package.xml                   # ROS 2 package manifest with dependencies
├── setup.py                      # Build configuration (entry points, data files)
├── setup.cfg
├── README.md
└── .gitignore
```

## ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| /cmd_vel | Twist | Node to Gazebo | Velocity commands |
| /world/name/dynamic_pose/info | TFMessage | Gazebo to Node | Ground truth pose |
| /odom | Odometry | Gazebo to Node | Wheel odometry (fallback) |
| /goal_pose | PoseStamped | RViz to Node | Goal from RViz click |
| /waypoint_markers | MarkerArray | Node to RViz | Colored waypoint spheres |
| /robot_path | Path | Node to RViz | Trail behind robot |
| /goal_marker | Marker | Node to RViz | Green arrow at goal |
| /overhead_lidar/points | PointCloud2 | Gazebo to RViz | LiDAR point cloud |

## Troubleshooting

**Gazebo opens but robot doesn't move:** Check if ground truth is publishing with `ros2 topic hz /world/leo_empty/dynamic_pose/info`. If no data, the ground truth bridge failed. Run `killros` and relaunch.

**Robot invisible in Gazebo (obstacle world):** Known Gazebo Harmonic rendering issue. The robot exists in physics and responds to commands. Check with `gz model --list` — if leo_rover appears, it's there.

**RViz shows fixed frame does not exist:** The PID node hasn't started yet or ground truth isn't flowing. Wait a few seconds or check the xterm window for errors.

**Zombie processes after crash:** Run `killros` or `bash scripts/kill_all.sh` to force-kill everything and clean up DDS shared memory.

**Point cloud not visible in LiDAR RViz:** Check the topic is publishing with `ros2 topic hz /overhead_lidar/points`. If no data, the LiDAR bridge may have failed. Ensure you launched with `world:=obstacles`. Also check the fixed frame is set to world

## Version History

- **v1.0** — Original PID waypoint follower with odometry-based control
- **v2.0** — Ground truth pose via Gazebo, RViz visualization, goal preemption
- **v3.0** — Master launch script, kill-all cleanup, DDS localhost fix
- **v3.1** — Fixed package.xml dependencies, dynamic ground truth topic
- **v4.0** — Obstacle world, 3D overhead LiDAR, dual RViz windows, LiDAR bridge