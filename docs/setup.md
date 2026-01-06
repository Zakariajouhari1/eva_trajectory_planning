# Setup Guide

## Quick Start

### 1. Install ROS 2 Foxy

If you don't have ROS 2:

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Foxy
sudo apt update
sudo apt install ros-foxy-desktop
```

### 2. Install Dependencies

```bash
sudo apt install -y \
  ros-foxy-nav-msgs \
  ros-foxy-geometry-msgs \
  ros-foxy-sensor-msgs \
  ros-foxy-visualization-msgs \
  ros-foxy-tf2 \
  ros-foxy-tf2-geometry-msgs \
  libeigen3-dev \
  python3-numpy \
  python3-colcon-common-extensions
```

### 3. Build Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/eva_trajectory_planning.git

# Build
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select eva_planning

# Source workspace
source install/setup.bash
```

### 4. Test It

```bash
# Terminal 1: Launch system
ros2 launch eva_planning test_planning.launch.py

# Terminal 2: Monitor (optional)
ros2 run eva_planning planning_monitor.py
```

You should see:
```
[eva_planning_node]: EVA Planning Node initialized
[eva_planning_node]: Map received: 200x200
[eva_planning_node]: Global path: X points in Y ms
```

## Docker Setup

### Build Container

```bash
cd eva_trajectory_planning
docker build -t eva_planning:latest .
```

### Run Container

```bash
docker run -it \
  --network=host \
  -v $(pwd):/workspace \
  eva_planning:latest
```

Inside container:
```bash
cd /workspace
colcon build --packages-select eva_planning
source install/setup.bash
ros2 launch eva_planning test_planning.launch.py
```

## Common Issues

### "colcon: command not found"
```bash
sudo apt install python3-colcon-common-extensions
```

### "package 'eva_planning' not found"
```bash
# Make sure you're in workspace root
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select eva_planning
```

### Build errors about missing headers
```bash
sudo apt install libeigen3-dev
```

### "Start or goal invalid" errors
This happens when vehicle position exceeds map bounds. The test generator uses a 40m radius circle, map is 100m×100m. If vehicle somehow gets outside bounds, the planner rejects it.

Fix: Restart the test generator or send a goal within bounds.

## Performance Tuning

Edit `eva_planning/config/planning_params.yaml`:

- **Slower computer?** Reduce `local_planner_rate` to 5.0 Hz
- **Need faster reaction?** Increase `local_planner_rate` to 20.0 Hz
- **Larger environment?** Increase `lookahead_distance`
- **Tight spaces?** Reduce `obstacle_avoidance_radius`

## Verification Commands

```bash
# Check nodes running
ros2 node list

# Check topics
ros2 topic list

# Monitor path publication rate
ros2 topic hz /planning/global_path
ros2 topic hz /planning/local_path

# See current vehicle position
ros2 topic echo /vehicle/odom

# See planned path
ros2 topic echo /planning/global_path
```
