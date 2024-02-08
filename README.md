# LidarSORT-3D-Object-Tracking

# Overview
This project aims to develop components for an autonomous driving system for cars or robots. It encompasses sensor data processing, object tracking, and autonomous driving control algorithms.

# File Descriptions

## Configuration Files
- **`params.yaml`**: Contains all the major configuration parameters for the system. This includes settings for sensor accuracy, tracking algorithm parameters, control gains, etc.

## ROS Launch Files
- **`mission.launch`**: Launches the necessary ROS nodes for actual mission execution. This includes sensor nodes, data processing nodes, etc.
- **`test.launch`**: Sets up a testing environment to evaluate the system's performance. It is used for testing specific sensors or algorithms.

## C++ Header and Source Files
- **`KalmanTracker.h`, `Hungarian.h`**: Contains implementations for object tracking algorithms.
- **`lidar_lib.hpp`, `camera_lib.hpp`**: Libraries for sensor data processing.
- **`car_struct.h`**: Defines the vehicle data structure.
- **`acc_controller.cpp`**: Implementation of the Adaptive Cruise Controller.

# Getting Started

## Installing Dependencies
This project is developed using ROS (Robot Operating System) and C++17. Ensure that the appropriate version of ROS and a C++ compiler are installed.

## Setting Configuration Parameters
1. Open the `params.yaml` file and adjust the parameters as needed.
2. These settings will directly affect the performance and operation of the system.

## Execution
To execute a mission, use `mission.launch`, and for system testing, use `test.launch`.

### Launch system
```bash
roslaunch acc_controller mission.launch
```
### Launch system with visualizing
```bash
roslaunch acc_controller test.launch
```

## Contributing
If you would like to contribute to this project, please submit issues or send Pull Requests. All contributions are welcome.
