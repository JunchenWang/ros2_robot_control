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
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_math REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# build libfri
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
  external/libfri/include
  external/libfri/src/protobuf_gen
  external/libfri/src/nanopb-0.2.8
  external/libfri/src/protobuf
  external/libfri/src/connection
  external/libfri/src/client_lbr
  external/libfri/src/client_trafo
  external/libfri/src/base
)

file(GLOB_RECURSE FRI_SRC
  external/libfri/src/base/*.cpp
  external/libfri/src/client_lbr/*.cpp
  external/libfri/src/connection/*.cpp
  external/libfri/src/protobuf/*.cpp
  external/libfri/src/protobuf/*.c
  external/libfri/src/protobuf_gen/*.c
  external/libfri/src/nanopb-0.2.8/*.c
  external/libfri/src/client_trafo/*.cpp
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
  hardware_interface
  urdf
  robot_math
  geometry_msgs
  realtime_tools
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
  DIRECTORY external/libfri/include/
  DESTINATION include
)

file(GLOB_RECURSE OTHER_HEADER_FILES "external/libfri/src/*/*.h")
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
