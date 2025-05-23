cmake_minimum_required(VERSION 3.8)
project(task_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Franka REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  "rclcpp"
  "realtime_tools"
  "urdf"
  "robot_math"
  "control_msgs"
  "ament_index_cpp"
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


file(GLOB_RECURSE utils src/utils/*.cpp)


add_executable(task_node 
              src/task_node.cpp
              ${utils})

add_executable(franka_rcm 
              src/franka_rcm.cpp
              ${utils})

target_include_directories(task_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(task_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(task_node PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(task_node PUBLIC
                      ${Boost_LIBRARIES})


target_include_directories(franka_rcm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(franka_rcm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(franka_rcm PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(franka_rcm PUBLIC
                      Franka::Franka
                     )

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

install(TARGETS task_node franka_rcm
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY  config
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
