cmake_minimum_required(VERSION 3.8)
project(kuka_fri)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_hardware_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_math REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(kdl_parser REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# build libfri-2.5
add_definitions(-DHAVE_SOCKLEN_T
                -DPB_SYSTEM_HEADER="pb_syshdr.h"
                -DPB_FIELD_16BIT
                -DHAVE_STDINT_H
                -DHAVE_STDDEF_H
                -DHAVE_STDBOOL_H
                -DHAVE_STDLIB_H
                -DHAVE_STRING_H
                -fPIC)

set(FRI_HEADER
  external/libfri-2.5/include
  external/libfri-2.5/src/protobuf_gen
  external/libfri-2.5/src/nanopb-0.2.8
  external/libfri-2.5/src/protobuf
  external/libfri-2.5/src/connection
  external/libfri-2.5/src/client_lbr
  external/libfri-2.5/src/client_trafo
  external/libfri-2.5/src/base
)

file(GLOB_RECURSE FRI_SRC
  external/libfri-2.5/src/base/*.cpp
  external/libfri-2.5/src/client_lbr/*.cpp
  external/libfri-2.5/src/connection/*.cpp
  external/libfri-2.5/src/protobuf/*.cpp
  external/libfri-2.5/src/protobuf/*.c
  external/libfri-2.5/src/protobuf_gen/*.c
  external/libfri-2.5/src/nanopb-0.2.8/*.c
  external/libfri-2.5/src/client_trafo/*.cpp
)

## COMPILE
add_library(${PROJECT_NAME} SHARED
  ${FRI_SRC}
  src/med7client.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${FRI_HEADER}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
  robot_hardware_interface
  urdf
  robot_math
  geometry_msgs
  realtime_tools
  kdl_parser
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/kuka_fri/
  DESTINATION include
)

install(
  DIRECTORY external/libfri-2.5/include/
  DESTINATION include
)

file(GLOB_RECURSE OTHER_HEADER_FILES "external/libfri-2.5/src/*/*.h")
install(
  FILES ${OTHER_HEADER_FILES}
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  "include"
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  robot_math
  urdf
  geometry_msgs
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
