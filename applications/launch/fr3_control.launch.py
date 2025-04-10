# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.actions import RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription

from launch.actions import RegisterEventHandler
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart


def generate_launch_description():

    # rviz_file = PathJoinSubstitution(
    #             [
    #                 FindPackageShare("ur_description"),
    #                 "rviz",
    #                 "view_robot.rviz"
    #             ]
    #         )

    # 使用hardwares包中的ur5e配置文件
    rviz_file = PathJoinSubstitution(
            [
                FindPackageShare("franka_description"),
                "rviz",
                "visualize_franka.rviz"
            ]
        )

    robot_xacro_filepath = PathJoinSubstitution(
        [
            FindPackageShare("franka_description"),
            "robots",
            "fr3",
            "fr3.urdf.xacro"
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_xacro_filepath,
            " ",
            "hand:=",
            "false",
            " ",
            "ee_id:=",
            "frank_hand",
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    params = PathJoinSubstitution(
        [
            FindPackageShare("applications"),
            "config",
            "fr3",
            "fr3_control.yaml",
        ]
    )

    # with open("urdf/fr3.urdf", "w") as f:
    #     f.write(robot_description)
    #     f.close()

   
  
    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    
    rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
            )
    
    control_node = Node(
        package="control_node",
        executable="control_node",
        parameters=[params, {"robot_description": robot_description}],
        # arguments=["--ros-args", "--params-file", params],
        output="both",
    )


    robot_monitor = Node(
            package='robot_monitor',
            executable='robot_monitor',
            output="screen",
            )

    arguments = [
            # DeclareLaunchArgument(
            #     simulation_parameter_name,
            #     default_value="true",
            #     description="is simulation ?",
            # ),
            ]


 

    nodes = arguments  + [
            # robot_state_publisher,
            # rviz_node,
            # robot_monitor,
            control_node,
            ]

    return LaunchDescription(nodes)
