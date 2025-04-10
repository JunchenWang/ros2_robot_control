cmake_minimum_required(VERSION 3.8)
project(controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  "rclcpp_lifecycle"
  "ament_cmake"
  "ament_cmake_ros"
  "realtime_tools"
  "hardware_interface"
  "controller_interface"
  "sensor_msgs"
  "urdf"
  "std_msgs"
  "robot_math"
  "pluginlib"
  "control_msgs"
  "ros2_utility"
  "tf2_ros"
  "visualization_msgs"
  "geometry_msgs"
  "std_srvs"
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

add_library(controllers 
            src/torque_controller.cpp
            src/forward_controller.cpp
            src/realtime_test_controller.cpp
            src/dummy_controller.cpp
            src/force_drag_controller.cpp
            src/force_drag_dev_controller.cpp
            src/admittance_controller.cpp
            src/rcm_controller.cpp
            src/joint_impedance_controller.cpp
            src/cartesian_impedance_pd_controller.cpp
            src/cartesian_impedance_pdplus_controller.cpp
            src/virtual_fixture_plane_controller.cpp
            src/virtual_fixture_line_controller.cpp
            src/virtual_fixture_general_controller.cpp
            src/virtual_fixture_general_pose_controller.cpp
            src/med7_controller.cpp)

add_library(controllers::controllers ALIAS controllers)
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
add_executable(tool_path_planning_test test/tool_path_planning_test.cpp)

target_include_directories(test_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# 将所有需要添加相同依赖的目标定义为一个列表
set(EXECUTABLE_TARGETS
  test_controller
  debug_all
  tool_path_planning_test
)

# 循环遍历列表，为每个目标添加依赖
foreach(TARGET ${EXECUTABLE_TARGETS})
  ament_target_dependencies(${TARGET}
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
endforeach()

install(TARGETS ${EXECUTABLE_TARGETS}
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
