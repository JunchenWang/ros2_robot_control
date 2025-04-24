cmake_minimum_required(VERSION 3.8)
project(robot_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  "rclcpp"
  "realtime_tools"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "urdf"
  "std_msgs"
  "std_srvs"
  "robot_math"
  "robot_control_msgs"
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts)

qt_standard_project_setup()

file(GLOB_RECURSE srcs src/*.cpp src/*.ui)
# file(GLOB_RECURSE uis  ui/*.ui)

add_executable(robot_gui 
                  ${srcs}
              )


ament_target_dependencies(robot_gui ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_include_directories(robot_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(robot_gui Qt6::Core Qt6::Widgets)


install(TARGETS
robot_gui
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY
#     launch
#     DESTINATION share/${PROJECT_NAME}
# )

ament_package()
