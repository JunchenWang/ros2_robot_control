cmake_minimum_required(VERSION 3.8)
project(robot_math)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
  "urdf"
)

# find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

# file(GLOB_RECURSE matlab_srcs src/matlab_code/*.cpp)

add_library(robot_math SHARED
           src/robot_math.cpp
           src/OnlineTrajPlanner.cpp
          )

# add_library(robot_math::robot_math ALIAS robot_math)
target_compile_features(robot_math PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(robot_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

 ament_target_dependencies(robot_math PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
 target_link_libraries(robot_math PUBLIC jsoncpp)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(robot_math PRIVATE "ROBOT_MATH_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS robot_math
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


add_executable(test_robot_math 
                  test/main.cpp)

target_compile_features(test_robot_math PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(test_robot_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS test_robot_math
        DESTINATION lib/${PROJECT_NAME})               

#ament_target_dependencies(robot_monitor rclcpp std_msgs sensor_msgs)
#ament_target_dependencies(test_robot_math PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(test_robot_math PUBLIC robot_math)


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
  robot_math
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
