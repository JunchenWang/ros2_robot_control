cmake_minimum_required(VERSION 3.8)
project(control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  "rclcpp"
  "realtime_tools"
  "rclcpp_lifecycle"
  "sensor_msgs"
  "urdf"
  "std_msgs"
  "std_srvs"
  "robot_math"
  "hardware_interface"
  "controller_interface"
  "pluginlib"
  "robot_control_msgs"
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.83.0 COMPONENTS odeint) 

# include_directories(${Boost_INCLUDE_DIRS}) 

# generate_parameter_library(control_node_parameters
#   src/control_node_parameters.yaml
# )

add_executable(control_node 
              src/control_node.cpp
              src/control_manager.cpp)

target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(control_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(control_node PUBLIC
                      ${Boost_LIBRARIES}
)

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch config
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

ament_package()
