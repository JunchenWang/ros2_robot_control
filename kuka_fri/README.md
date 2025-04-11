# ROS2 Package for KUKA FRI Med7 Communication

This repository contains a ROS2 package built upon the official **FRI-Client-SDK_Cpp (version 2.5)**, adapted for use with the **KUKA Med7 robot**. It is developed and maintained by the [Surgical Robot Navigation and Control Laboratory](https://mrs.buaa.edu.cn/).

## Overview

This package provides:

- A class `Med7Client` for real-time communication with the KUKA Med7 robot using the FRI 2.5 protocol.
- Custom ROS2 packages:
  - `hardware_interface`: A hardware interface implementation for integrating with our ROS2 control frameworks.
  - `robot_math`: A mathematical utility package used in robot kinematics and control algorithms.

## Modifications to FRI SDK

The following modifications were made to the original `external/libfri-2.5` source code in order to support integration with the Med7Client class and enable ROS2 compatibility:

1. **Access Modifier Change in `friLBRClient.h`**
   The method `virtual ClientData* createData();` in the file `include/friLBRClient.h` was originally declared as `protected`.
   It was changed to `public` to allow external access from the `Med7Client` class.
2. **Friend Class Declarations**In order to grant the `Med7Client` class access to internal members of the robot command and state classes, the following friend class declarations were added:

   - In `include/friLBRCommand.h`:
     ```cpp
     friend class Med7Client;
     ```
   - In `include/friLBRState.h`:
     ```cpp
     friend class Med7Client;
     ```

These modifications were necessary to enable low-level access to FRI internal data structures and implement a customized client for KUKA Med7 robots within a ROS2 environment.

## Requirements

- ROS2 Jazzy
- C++17 compiler
- Official [KUKA FRI SDK 2.5](https://www.kuka.com/)
- Linux system (recommended Ubuntu 24.04 with PREEMPT_RT kernel)

## Build Instructions

Clone the repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/JunchenWang/ros2_robot_control
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
