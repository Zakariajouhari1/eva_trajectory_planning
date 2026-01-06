# EVA Trajectory Planning

ROS 2 trajectory planning system combining A* global planning with cubic spline local path generation.

## Features

- **Global Planning**: A* algorithm for optimal pathfinding
- **Local Planning**: Cubic spline interpolation for smooth trajectories
- **Obstacle Avoidance**: Real-time reactive behavior
- **ROS 2 Integration**: Works with Foxy and Humble

## Requirements

- Ubuntu 20.04 or 22.04
- ROS 2 Foxy or Humble
- C++14 compiler
- Eigen3

## Installation

```bash
# Install dependencies
sudo apt install ros-foxy-nav-msgs ros-foxy-geometry-msgs \
                 ros-foxy-visualization-msgs libeigen3-dev

# Clone and build
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/eva_trajectory_planning.git
cd ~/ros2_ws
colcon build --packages-select eva_planning
source install/setup.bash
```

## Usage

**Launch the system:**
```bash
ros2 launch eva_planning test_planning.launch.py
```

**Monitor performance:**
```bash
ros2 run eva_planning planning_monitor.py
```

**Set custom goal:**
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, \
    pose: {position: {x: 30.0, y: 30.0, z: 0.0}, \
           orientation: {w: 1.0}}}" --once
```

## Configuration

Edit `eva_planning/config/planning_params.yaml`:

```yaml
eva_planning_node:
  ros__parameters:
    global_planner_rate: 1.0        # Hz
    local_planner_rate: 10.0        # Hz
    map_resolution: 0.5             # meters
    robot_radius: 1.0               # meters
    max_speed: 5.0                  # m/s
    lookahead_distance: 10.0        # meters
    obstacle_avoidance_radius: 2.0  # meters
```

## Topics

**Subscribed:**
- `/vehicle/odom` (nav_msgs/Odometry) - Vehicle state
- `/goal_pose` (geometry_msgs/PoseStamped) - Target position
- `/map` (nav_msgs/OccupancyGrid) - Environment map

**Published:**
- `/planning/global_path` (nav_msgs/Path) - A* planned path
- `/planning/local_path` (nav_msgs/Path) - Spline trajectory
- `/planning/status` (std_msgs/String) - Planning status

## Troubleshooting

**Build fails:**
```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select eva_planning
```

**Planning fails ("Start or goal invalid"):**
- Check vehicle is within map bounds (-50m to +50m)
- Verify map is being published: `ros2 topic echo /map --once`
- Ensure coordinates don't exceed grid limits

**No visualization:**
```bash
# Check topics are publishing
ros2 topic list
ros2 topic hz /planning/global_path
```

## License

MIT

## Contact

Issues: https://github.com/YOUR_USERNAME/eva_trajectory_planning/issues
