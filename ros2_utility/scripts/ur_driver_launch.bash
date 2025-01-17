#!/bin/bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.31.201 launch_rviz:=true

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release