cmake_minimum_required(VERSION 3.8)
project(hardwares)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_math REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(realtime_tools REQUIRED)
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

add_library(hardwares 
            src/simulation_robot.cpp
            src/ft_kunwei_sensor.cpp
            src/ft_ati_sensor.cpp
            src/ur_robot.cpp)

add_library(hardwares::hardwares ALIAS hardwares)
target_compile_features(hardwares PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(hardwares PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  hardwares
  "hardware_interface"
  "pluginlib"
  "urdf"
  "robot_math"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "geometry_msgs"
  "realtime_tools"
)

target_link_libraries(hardwares 
                      ur_rtde::rtde             
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hardwares PRIVATE "HARDWARES_BUILDING_LIBRARY")

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )
install(
  TARGETS hardwares
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


add_executable(test_hardware src/test_hardware.cpp)
target_include_directories(test_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_hardware
  "hardware_interface"
  "pluginlib"
  # "urdf"
  # "robot_math"
  # "rclcpp_lifecycle"
  
)

target_link_libraries(test_hardware 
                      ur_rtde::rtde             
                      )

install(TARGETS test_hardware
        DESTINATION lib/${PROJECT_NAME})


install(
          DIRECTORY launch config urdf
          DESTINATION share/${PROJECT_NAME}
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  hardwares
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
