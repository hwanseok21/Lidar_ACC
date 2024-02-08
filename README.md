# Vehicle Control System Library

This library provides core functionalities for the development of vehicle control systems. It includes algorithms for sensor data processing, object tracking, and the control of vehicle speed and direction.

## Main Components

- `acc_controller.cpp`: Implementation of vehicle speed and distance control algorithms.
- `Hungarian.h`: Class for solving the optimal assignment problem using the Hungarian algorithm.
- `lidar_lib.hpp`: LiDAR sensor data processing and object recognition functionalities.
- `KalmanTracker.h`: Object tracking algorithm using Kalman filter.
- `struct.hpp~`: Basic data structures and utility functions.
- `camera_lib.hpp`: Camera sensor data processing and image analysis functionalities.
- `car_struct.h`: Definitions of data structures related to vehicles.

## Installation

This library is written in C++ and requires a compiler to be installed before use. To use the library, compile the source code and install necessary dependencies as follows:

```bash
g++ -std=c++11 -o your_program_name your_program.cpp -lneeded_libraries
