cmake_minimum_required(VERSION 3.5)
project(robot_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)


set(msg_files
  msg/VectorData.msg
)

set(srv_files
  srv/ControlCommand.srv
)

set(action_files
  action/RobotMotion.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
