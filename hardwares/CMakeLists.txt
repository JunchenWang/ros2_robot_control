cmake_minimum_required(VERSION 3.8)
project(hardwares)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(Franka REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "geometry_msgs"
  "realtime_tools"
  "urdf"
  "robot_math"
  "robot_hardware_interface"
  "pluginlib"
  "orocos_kdl_vendor"
  "kdl_parser"
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

pluginlib_export_plugin_description_file(robot_hardware_interface plugins.xml)

file(GLOB_RECURSE srcs src/*.cpp src/sr/*.cpp)

add_library(hardwares 
            ${srcs})

# add_library(hardwares::hardwares ALIAS hardwares)
target_compile_features(hardwares PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(hardwares PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  hardwares
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(hardwares 
                      ur_rtde::rtde
                      Franka::Franka
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


add_executable(test_hardware test/test_hardware.cpp)
target_include_directories(test_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_hardware
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  
)

target_link_libraries(test_hardware 
                      ur_rtde::rtde 
                      Franka::Franka            
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
