cmake_minimum_required(VERSION 3.8)
project(controller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_math REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(controller_interface SHARED
            src/controller_interface.cpp)

add_library(controller_interface::controller_interface ALIAS controller_interface)

target_compile_features(controller_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(controller_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


ament_target_dependencies(controller_interface
    "urdf"
    "robot_math"
    "rclcpp_lifecycle"
    "rclcpp"
    "hardware_interface"
    "geometry_msgs"
    "realtime_tools"
  )

  
  target_compile_definitions(controller_interface PRIVATE "CONTROLLER_INTERFACE_BUILDING_LIBRARY")

  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )
  install(
    TARGETS controller_interface
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  ament_export_include_directories(
    "include/${PROJECT_NAME}"
  )
  ament_export_libraries(
    hardware_interface
  )
  ament_export_targets(
    export_${PROJECT_NAME}
  )



ament_package()
