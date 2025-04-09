cmake_minimum_required(VERSION 3.8)
project(kuka_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# build libFRI
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
  external/libFRI/include
  external/libFRI/src/protobuf_gen
  external/libFRI/src/nanopb-0.2.8
  external/libFRI/src/protobuf
  external/libFRI/src/connection
  external/libFRI/src/client_lbr
  external/libFRI/src/client_trafo
  external/libFRI/src/base
)

file(GLOB_RECURSE FRI_SRC
  external/libFRI/src/base/*.cpp
  external/libFRI/src/client_lbr/*.cpp
  external/libFRI/src/connection/*.cpp
  external/libFRI/src/protobuf/*.cpp
  external/libFRI/src/protobuf/*.c
  external/libFRI/src/protobuf_gen/*.c
  external/libFRI/src/nanopb-0.2.8/*.c
  external/libFRI/src/client_trafo/*.cpp
)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  ${FRI_SRC}
  src/IRDFClient.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${FRI_HEADER}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
)

# EXECUTABLE
add_executable(kuka_fri_test test/kuka_fri_test.cpp)

target_include_directories(kuka_fri_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${FRI_HEADER}
)

ament_target_dependencies(kuka_fri_test
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
)

target_link_libraries(kuka_fri_test
  ${PROJECT_NAME}
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS kuka_fri_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
