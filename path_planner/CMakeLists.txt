cmake_minimum_required(VERSION 3.8)
project(path_planner)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_ros
  rclcpp_lifecycle
  rclcpp
  urdf
  PCL
  Eigen3
  VTK
  pluginlib
  robot_control_msgs
  tf2_ros
  visualization_msgs
  geometry_msgs
  robot_math
  std_srvs
  ros2_utility
  yaml-cpp 
  noether_gui
  noether_tpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 添加可执行文件
add_executable(path_generator
  test/path_generator.cpp
  src/tpp_utils.cpp
)

add_executable(icp_test
  test/icp_test.cpp
)

add_executable(debug_all
  test/debug_all.cpp
)

set(EXECUTABLE_TARGETS
  path_generator
  icp_test
  debug_all
)

# 循环遍历列表，为每个目标添加依赖
foreach(TARGET ${EXECUTABLE_TARGETS})
  target_include_directories(${TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  ament_target_dependencies(${TARGET}
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
endforeach()

target_link_libraries(path_generator
  yaml-cpp
  noether::noether_tpp
  noether::noether_gui
)

target_link_libraries(debug_all
  OpenCAMLib::ocl
)

install(TARGETS ${EXECUTABLE_TARGETS}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()