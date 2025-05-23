cmake_minimum_required(VERSION 3.8)
project(controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  "rclcpp_lifecycle"
  "realtime_tools"
  "robot_hardware_interface"
  "robot_controller_interface"
  "urdf"
  "std_msgs"
  "robot_math"
  "pluginlib"
  "robot_control_msgs"
  "ros2_utility"
  "tf2_ros"
  "visualization_msgs"
  "geometry_msgs"
  "std_srvs"
  "orocos_kdl_vendor"
  "kdl_parser"
  "rclcpp_action"
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


pluginlib_export_plugin_description_file(robot_controller_interface plugins.xml)

file(GLOB_RECURSE srcs src/*.cpp)

add_library(controllers SHARED
            ${srcs})

# add_library(controllers::controllers ALIAS controllers)
target_compile_features(controllers PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  controllers
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(controllers PRIVATE "CONTROLLERS_BUILDING_LIBRARY")

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )
install(
  TARGETS controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(test_controller test/test_controller.cpp)
add_executable(debug_all test/debug_all.cpp)
add_executable(test_joint_controller test/test_joint_controller.cpp)
add_executable(test_cartesian_controller test/test_cartesian_controller.cpp)
add_executable(test_trajectory_controller test/test_trajectory_controller.cpp)
add_executable(movel utility/movel.cpp)
add_executable(movej utility/movej.cpp)
# target_include_directories(test_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(test_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  test_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(
  debug_all
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(
  test_trajectory_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(
  test_joint_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(
  test_cartesian_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(
  movej
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(
  movel
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS test_controller debug_all test_joint_controller test_trajectory_controller movej movel test_cartesian_controller
  DESTINATION lib/${PROJECT_NAME})

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
  controllers
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
