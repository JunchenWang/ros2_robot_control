cmake_minimum_required(VERSION 3.8)
project(robot_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts)

qt_standard_project_setup()

add_executable(robot_monitor 
                  src/main.cpp 
                  src/mainwindow.cpp
                  src/widget.cpp
                  src/widget.h
                  src/mainwindow.h
                  src/mainwindow.ui)


ament_target_dependencies(robot_monitor rclcpp std_msgs sensor_msgs std_srvs)

target_link_libraries(robot_monitor Qt6::Core Qt6::Widgets Qt6::Charts)


install(TARGETS
  robot_monitor
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY
#     launch
#     DESTINATION share/${PROJECT_NAME}
# )

ament_package()
