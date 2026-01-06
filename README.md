# 🚗 EVA Trajectory Planning System

[![ROS 2](https://img.shields.io/badge/ROS_2-Foxy-blue.svg)](https://docs.ros.org/en/foxy/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Documentation](https://img.shields.io/badge/docs-passing-brightgreen.svg)](./docs/)

**Professional trajectory planning system for autonomous vehicles** combining global path planning (A*) and local trajectory generation (Cubic Splines) with real-time obstacle avoidance.

> 🎓 **Academic Project** | Part of EVA Autonomous Vehicle R&D Platform - Unit 5: Global and Local Trajectory Planning

---

## 📑 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Configuration](#configuration)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [Citation](#citation)
- [License](#license)

---

## 🎯 Overview

The EVA Trajectory Planning System implements a **hybrid planning approach** for autonomous vehicle navigation:

- **Global Planning**: A* algorithm for optimal path generation on occupancy grids
- **Local Planning**: Cubic spline interpolation for smooth, dynamically feasible trajectories
- **Obstacle Avoidance**: Real-time reactive behavior using potential field methods
- **ROS 2 Integration**: Full integration with Robot Operating System 2 (Foxy)

### Key Capabilities

✅ **Real-time path planning** at 1 Hz (global) and 10 Hz (local)  
✅ **Obstacle detection and avoidance** with configurable safety margins  
✅ **Smooth trajectory generation** respecting vehicle kinematic constraints  
✅ **Comprehensive testing framework** with simulation scenarios  
✅ **Production-ready code** with extensive documentation and error handling

---

## ⭐ Features

### Planning Algorithms

- **A\* Global Planner**
  - Configurable heuristics (Euclidean, Manhattan, Octile)
  - Path smoothing and optimization
  - Efficient obstacle avoidance
  - Grid map support with customizable resolution

- **Cubic Spline Local Planner**
  - Real-time trajectory adjustment
  - Lookahead distance control
  - Dynamic obstacle avoidance with potential fields
  - Continuous path generation

### ROS 2 Integration

- **Publisher Topics**:
  - `/planning/global_path` - Global waypoint path
  - `/planning/local_path` - Local smooth trajectory
  - `/planning/status` - Planning status messages

- **Subscriber Topics**:
  - `/vehicle/odom` - Vehicle odometry
  - `/goal_pose` - Target destination
  - `/map` - Occupancy grid map

- **Parameters**: Fully configurable via YAML files

### Testing & Simulation

- **Test Data Generator**: Simulates vehicle dynamics and environment
- **Real-time Monitor**: Performance metrics and visualization
- **Multiple Scenarios**: Circular track, obstacle courses, urban grids
- **Comprehensive Logging**: Full data recording for analysis

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    EVA Planning Node                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐         ┌──────────────┐                │
│  │   A* Global  │────────▶│    Path      │                │
│  │   Planner    │         │  Smoothing   │                │
│  └──────────────┘         └──────────────┘                │
│         │                         │                         │
│         ▼                         ▼                         │
│  ┌──────────────────────────────────────┐                 │
│  │      Global Path Waypoints           │                 │
│  └──────────────────────────────────────┘                 │
│                    │                                        │
│                    ▼                                        │
│  ┌──────────────────────────────────────┐                 │
│  │   Cubic Spline Local Planner         │                 │
│  │   + Potential Field Avoidance        │                 │
│  └──────────────────────────────────────┘                 │
│                    │                                        │
│                    ▼                                        │
│         Local Smooth Trajectory                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
         │                            │
         ▼                            ▼
   Control Module            Visualization (RViz2)
```

**Data Flow**:
1. Map → Global Planner → Waypoints
2. Waypoints + Odometry → Local Planner → Trajectory
3. Trajectory → Vehicle Control

For detailed architecture, see [docs/architecture.md](docs/architecture.md)

---

## 📦 Installation

### Prerequisites

- **OS**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- **ROS 2**: Foxy Fitzroy or Humble Hawksbill
- **Compiler**: GCC 9.3+ with C++14 support
- **Dependencies**:
  ```bash
  sudo apt install -y \
    ros-foxy-nav-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-visualization-msgs \
    ros-foxy-tf2 \
    ros-foxy-tf2-geometry-msgs \
    libeigen3-dev \
    python3-numpy
  ```

### Build from Source

```bash
# Create workspace
mkdir -p ~/eva_ws/src
cd ~/eva_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/eva_trajectory_planning.git

# Build
cd ~/eva_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select eva_planning --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

### Docker Installation

```bash
# Build Docker image
docker build -t eva_planning:latest -f docker/Dockerfile .

# Run container
docker-compose -f docker/docker-compose.yml up
```

For detailed installation instructions, see [docs/installation.md](docs/installation.md)

---

## 🚀 Quick Start

### 1. Launch Complete System

```bash
# Terminal 1: Launch planning system with test generator
ros2 launch eva_planning test_planning.launch.py

# Terminal 2: Monitor performance
ros2 run eva_planning planning_monitor.py
```

### 2. Send Manual Goal

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, 
    pose: {position: {x: 30.0, y: 30.0, z: 0.0}, 
           orientation: {w: 1.0}}}" --once
```

### 3. Visualize in RViz2

```bash
rviz2 -d eva_planning/rviz/planning.rviz
```

### 4. View Topics

```bash
# List all topics
ros2 topic list

# Echo global path
ros2 topic echo /planning/global_path

# Check planning rate
ros2 topic hz /planning/local_path
```

---

## 🎮 Usage

### Running Different Scenarios

#### Circular Track (Default)
```bash
ros2 launch eva_planning test_planning.launch.py \
  scenario:=circular_track \
  vehicle_speed:=3.0
```

#### Obstacle Course
```bash
ros2 launch eva_planning test_planning.launch.py \
  scenario:=obstacle_course \
  vehicle_speed:=2.0
```

#### Urban Grid
```bash
ros2 launch eva_planning test_planning.launch.py \
  scenario:=urban_grid \
  vehicle_speed:=4.0
```

### Parameter Tuning

Edit `eva_planning/config/planning_params.yaml`:

```yaml
eva_planning_node:
  ros__parameters:
    # Planning rates
    global_planner_rate: 1.0      # Hz
    local_planner_rate: 10.0      # Hz
    
    # Map parameters
    map_resolution: 0.5            # meters
    robot_radius: 1.0              # meters
    
    # Global planner (A*)
    heuristic_type: "euclidean"    # euclidean/manhattan/octile
    
    # Local planner
    max_speed: 5.0                 # m/s
    max_acceleration: 2.0          # m/s²
    lookahead_distance: 10.0       # meters
    
    # Obstacle avoidance
    obstacle_avoidance_radius: 2.0 # meters
```

---

## 📚 Documentation

Comprehensive documentation is available in the `docs/` directory:

- **[Installation Guide](docs/installation.md)** - Detailed setup instructions
- **[Architecture Overview](docs/architecture.md)** - System design and components
- **[Algorithm Details](docs/algorithms.md)** - A* and Spline implementation
- **[API Reference](docs/api_reference.md)** - Class and function documentation
- **[Troubleshooting](docs/troubleshooting.md)** - Common issues and solutions

---

## 🧪 Testing

### Run Unit Tests

```bash
cd ~/eva_ws
colcon test --packages-select eva_planning
colcon test-result --verbose
```

### Performance Benchmarks

```bash
# Run benchmark scenarios
./scripts/run_tests.sh

# Results saved in: eva_planning/test_results/
```

### Code Coverage

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Coverage
colcon test --packages-select eva_planning
```

---

## 🔧 Configuration

### Planning Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `global_planner_rate` | double | 1.0 | Global planning frequency (Hz) |
| `local_planner_rate` | double | 10.0 | Local planning frequency (Hz) |
| `map_resolution` | double | 0.5 | Grid cell size (meters) |
| `robot_radius` | double | 1.0 | Vehicle radius for collision (m) |
| `max_speed` | double | 5.0 | Maximum vehicle speed (m/s) |
| `lookahead_distance` | double | 10.0 | Local planner horizon (m) |
| `obstacle_avoidance_radius` | double | 2.0 | Obstacle influence range (m) |

### Topic Configuration

All topic names can be remapped using ROS 2 launch files or command line:

```bash
ros2 run eva_planning eva_planning_node \
  --ros-args \
  --remap /vehicle/odom:=/my_odom \
  --remap /goal_pose:=/my_goal
```

---

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Development Setup

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style

- C++: Follow [ROS 2 C++ Style Guide](https://docs.ros.org/en/foxy/Contributing/Code-Style-Language-Versions.html)
- Python: Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/)
- Use `clang-format` for C++ and `black` for Python

---

## 📖 Citation

If you use this work in your research, please cite:

```bibtex
@software{eva_trajectory_planning2025,
  author = {Your Name},
  title = {EVA Trajectory Planning System},
  year = {2025},
  publisher = {GitHub},
  url = {https://github.com/YOUR_USERNAME/eva_trajectory_planning}
}
```

### Related Publications

- Abd-alrasool et al. (2023). "Global and Local Path Planning for Self-Driving Car"
- Abbadi & Matousek (2014). "Path Planning Implementation Using MATLAB"

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **ROS 2 Community** for the excellent robotics framework
- **Research Papers** cited in `docs/algorithms.md`
- **EVA Project Team** for collaboration and support

---

## 📧 Contact

**Author**: Your Name  
**Email**: your.email@example.com  
**Project**: EVA Autonomous Vehicle Platform - Unit 5

**Issues**: Please report bugs via [GitHub Issues](https://github.com/YOUR_USERNAME/eva_trajectory_planning/issues)

---

## 🗺️ Roadmap

- [x] A* Global Planner
- [x] Cubic Spline Local Planner
- [x] Real-time Obstacle Avoidance
- [x] ROS 2 Integration
- [ ] Hybrid A* Implementation
- [ ] MPC Local Planner
- [ ] Dynamic Obstacle Prediction
- [ ] Multi-vehicle Coordination
- [ ] Real Hardware Integration

---

<div align="center">

**⭐ Star this repository if you find it helpful!**

[Report Bug](https://github.com/YOUR_USERNAME/eva_trajectory_planning/issues) · 
[Request Feature](https://github.com/YOUR_USERNAME/eva_trajectory_planning/issues) · 
[Documentation](./docs/)

</div>
