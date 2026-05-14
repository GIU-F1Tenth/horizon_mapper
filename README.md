# Horizon Mapper

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
![MPC](https://img.shields.io/badge/MPC-R.svg)
![LQR](https://img.shields.io/badge/LQR-R.svg)

A ROS2 package for trajectory processing and horizon publishing built for F1TENTH racing. Horizon Mapper turns optimal trajectories into rolling reference horizons for optimal control loops (LQR, MPC, and beyond), so the controller can stay focused on the OCP instead of data plumbing.

Think of it as the pit crew for your controller: it delivers clean, timely horizons so the OCP can focus on winning the lap.

### Key Capabilities

- **Trajectory Processing**: Loads and preprocesses optimal racing trajectories from CSV files
- **Predictive Horizons**: Generates rolling trajectory horizons for LQR, MPC, and other optimal control policies
- **State Estimation**: Integrates vehicle odometry and pose estimates for accurate reference tracking
- **Real-time Publishing**: Provides continuous trajectory updates at configurable frequencies
- **Validation**: Ensures trajectory data integrity with comprehensive validation checks

## Installation


1. **Clone the repository** into your ROS2 workspace:
   ```bash
   git clone https://github.com/GIU-F1Tenth/horizon_mapper.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/horizon_mapper
   pip install -r requirement.txt
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select horizon_mapper
   source install/setup.bash
   ```

## Usage

### Basic Launch

Launch the Horizon Mapper with config file parameters:

```bash
ros2 launch horizon_mapper horizon_mapper.launch.py
```

### Integration Example

The Horizon Mapper is used as part of a larger racing stack:

```bash
# Launch the full stack (example)
ros2 launch race_stack autonomous_racing.launch.py \
  map_name:=your_track \
  enable_horizon_mapper:=true
```

## Configuration

### Trajectory Files

The HM expects trajectory files in CSV format with the following columns:

```csv
# x,y,v
2.0,4.0,1.0
2.2,5.1,1.6
2.4,4.2,2.3
...
```
- `x`, `y`: Position coordinates (meters)
- `v`: Velocity (m/s)

The HM produces x,y,v,theta,delta,kappa based on the Bicycle-model.

### Configuration File

Edit `config/horizon_mapper.yaml` to customize behavior:

```yaml
horizon_mapper:
  ros__parameters:
    horizon_N: 8              # Prediction horizon steps
    horizon_T: 0.5            # Time per horizon step (seconds)
    wheelbase: 0.33           # Vehicle wheelbase (meters)
    max_steering_angle: 0.5   # Maximum steering angle (radians)
    min_speed: 0.1            # Minimum vehicle speed (m/s)
    enable_logging: false     # Enable detailed logging
```

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/car_state/odom` | `nav_msgs/Odometry` | Vehicle odometry data |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 2D pose estimates |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/horizon_mapper/reference_trajectory` | `giu_f1t_interfaces/VehicleStateArray` | Predictive trajectory horizon for LQR/MPC |
| `/horizon_mapper/reference_path` | `nav_msgs/Path` | Full reference path for visualization |
| `/horizon_mapper/path_ready` | `std_msgs/Bool` | Status indicator for trajectory readiness |

## Dependencies

### ROS2 Packages
- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard ROS2 message types
- `geometry_msgs` - Geometry-related message types
- `nav_msgs` - Navigation message types
- `tf_transformations` - Coordinate transformation utilities
- `giu_f1t_interfaces` - Custom F1TENTH interface definitions

### Python Packages
- `numpy` - Numerical computing
- `csv` - CSV file processing
- `math` - Mathematical functions

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Odometry      │───▶│  Horizon        │───▶│   Controller    │
│   /car_state/   │    │  Mapper         │    │  (LQR / MPC)    │
│   odom          │    │                 │    │                 │
└─────────────────┘    │  ┌───────────┐  │    └─────────────────┘
                       │  │Trajectory │  │
┌─────────────────┐    │  │Processing │  │    ┌─────────────────┐
│   RViz Pose     │───▶│  └───────────┘  │───▶│   Visualization │
│   /initialpose  │    │                 │    │   /rviz         │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Contributing

We welcome contributions! Please follow these steps:

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Make your changes** and ensure tests pass
4. **Commit your changes**: `git commit -m 'Add amazing feature'`
5. **Push to the branch**: `git push origin feature/amazing-feature`
6. **Open a Pull Request**

### Development Guidelines

- Follow PEP 8 Python style guidelines
- Add docstrings to all functions and classes
- Include unit tests for new functionality
- Update documentation as needed

### Testing

Run the test suite:

```bash
cd ~/ros2_ws
colcon test --packages-select horizon_mapper
colcon test-result --verbose
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **F1TENTH Community**: For the racing platform and ecosystem
- **ROS2 Team**: For the excellent robotics middleware
- **ETH Zurich PBL**: For the ForzaETH race stack inspiration
- **Contributors**: Thanks to all who have contributed to this project

## Citation

If you use this package in your research, please cite:

```bibtex
@software{horizon_mapper_2025,
  title={Horizon Mapper: Trajectory Processing for F1TENTH Autonomous Racing},
  author={Mohammed Abdelazim},
  year={2025},
  url={https://github.com/GIU-F1Tenth/horizon_mapper}
}
```
---

**Happy Racing! 🏁**
