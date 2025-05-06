# ros2_robot_control

This is our lab code repository for robot control using ros2. Of course you have to install ROS2  first. The codes are developed under **Ubuntu 24** with **ROS2 Jazzy**.

# package dependencies

Please clone the following packages into the src folder of your ros2's workspace. (e.g., ~/ros2_ws)

1. franka_description (https://github.com/frankaemika/franka_description.git)

# build from source

1. install dependencies

   ```bash
   sudo apt install qt6-base-dev qt6-charts-dev libjsoncpp-dev
   ```
2. go to your workspace's src folder and clone & build the source

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/JunchenWang/ros2_robot_control.git
   rosdep install --from-paths src -y --ignore-src
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
   ```
3. launch the test

   ```bash
   source ~/ros2_ws/install/local_setup.bash
   ros2 launch applications simulation_ur.launch.py
   ```
   you can activate the ForwardController in the robot gui and change the joint position of the robot.
   ![screenshot1](screenshot3.png)
   ![screenshot1](screenshot2.png)

   press ctrl+c to terminate.

# package list

This repository contains the following packages:

1. applications
   
   Each py file in the launch dir is a concrete control task using the framework.

2. robot_monitor

   It is a gui interface which can display the real-time curves of joint states (position, velocity, effort, acceleration) published on the topic of "joint_states".

   ![screenshot1](screenshot1.png)

   You can run **robot_monitor** as a standalone node to visualize the robot's states.

   ```bash
   ros2 run robot_monitor robot_monitor
   ```

   before that, do not forget to source the path

   ```bash
   source ~/ros2_ws/install/local_setup.bash
   ```

   Now, if some other node on the ROS2 network published messages into the "joint_states" topic, the **robot_monitor** can capture the signals.
3. robot_math

   It is a libray for robot mathematics, including dynamics and kinematics using twist and wrench representation. 

4. control_node

   It is a light-weight framework for robot control. 

5. robot_control_msgs

   customized msgs used in this framework.

6. robot_controller_interface

   controller interface.

7. robot_hardware_interface

   hardware interface.

8. controllers

   various controllers implementation.

9.  hardwares

    various hardwares implementation.

10. robot_gui

    a robot gui to facilitate using the framework.

11. task_node
   
    each cpp file under the src folder is an individual control task.
    > * franka_rcm.cpp: the source code for our article ''Dynamic Control with a Remote Center-of-Motion Constraint for Human-Robot Collaboration'' submitted to R-CIM.
