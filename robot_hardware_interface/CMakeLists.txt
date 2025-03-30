cmake_minimum_required(VERSION 3.8)
project(robot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_math REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
add_library(robot_hardware_interface SHARED
            src/hardware_interface.cpp
            src/robot_interface.cpp
            src/sensor_interface.cpp)

# add_library(hardware_interface::hardware_interface ALIAS hardware_interface)

target_compile_features(robot_hardware_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(robot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


ament_target_dependencies(
  robot_hardware_interface
  "urdf"
  "robot_math"
  "rclcpp_lifecycle"
  "rclcpp"
  "geometry_msgs"
  "pluginlib"
  "realtime_tools"
)

target_compile_definitions(robot_hardware_interface PRIVATE "HARDWARE_INTERFACE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS robot_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  robot_hardware_interface
)
ament_export_targets(
  export_${PROJECT_NAME}
)


ament_package()
