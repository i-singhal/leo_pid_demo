# Leo PID Demo

PID waypoint follower for the Leo Rover in Gazebo Harmonic simulation. Uses ground truth pose from Gazebo (not odometry) to avoid skid-steer drift, with RViz visualization of waypoints, path trail, and goal markers.

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

### Launch everything (one command)

```bash
ros2 launch leo_pid_demo master_launch.py
```

This starts Gazebo, the ground truth bridge, the PID waypoint follower (in its own xterm window), and RViz — all with timed delays so each component starts after its dependencies are ready.

### Send waypoints

Two ways to set goals:

- **Type coordinates** in the xterm window: `2.0 1.0`
- **Click in RViz** using the "2D Goal Pose" button (the green arrow in the toolbar)

### Launch with a custom world

```bash
ros2 launch leo_pid_demo master_launch.py world:=obstacles
```

This looks for `worlds/leo_obstacles.sdf` in the package. Default is the built-in `leo_empty` world.

### Clean shutdown

Press **Ctrl+C** in the launch terminal. All processes shut down together.

If things get stuck, use the kill script:

```bash
bash ~/rover_ws/src/leo_pid_demo/scripts/kill_all.sh
```

### Tune PID parameters at runtime

While the node is running:

```bash
ros2 param set /pid_waypoint_follower kp_lin 0.8
ros2 param set /pid_waypoint_follower kp_ang 1.5
```

Or use the graphical tuner:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

## Version history

- **v1.0** — Original PID waypoint follower with odometry-based control
- **v2.0** — Ground truth pose via Gazebo, RViz visualization, goal preemption
- **v3.0** — Master launch script, kill-all cleanup, DDS localhost fix
- **v3.1** — Fixed package.xml dependencies, dynamic ground truth topic