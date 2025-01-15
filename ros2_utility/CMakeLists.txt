cmake_minimum_required(VERSION 3.8)
project(ros2_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 数学包所需的依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(robot_math REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ur_rtde REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)

# 递归 glob 所有源文件
file(GLOB_RECURSE ALL_SRCS src/*.cpp)
add_library(ros2_utility SHARED ${ALL_SRCS})

# 要求编译时使用 C99 标准 (用于 C 代码) 和 C++17 标准 (用于 C++ 代码)
target_compile_features(ros2_utility PUBLIC c_std_99 cxx_std_17)

# 设置动态链接库头文件的包含路径
target_include_directories(ros2_utility PUBLIC
    ${EIGEN3_INCLUDE_DIR} # 添加 Eigen3 的头文件路径
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(ros2_utility
    rclcpp
    rclcpp_lifecycle
    robot_math
    urdf
    tf2_ros
    geometry_msgs
    visualization_msgs
    Eigen3
    jsoncpp
    ur_rtde
    PkgConfig
    FFTW3
)

# 安装头文件到目标路径，将源代码的 include 目录安装到项目的 include/${PROJECT_NAME} 目录下
install(
    DIRECTORY include/               # 源文件夹
    DESTINATION include/${PROJECT_NAME}  # 目标路径
)

# 安装 ros2_utility 库及其相关文件
install(
    TARGETS ros2_utility                       # 安装目标
    EXPORT export_${PROJECT_NAME}            # 导出目标配置
    ARCHIVE DESTINATION lib                 # 安装静态库到 lib 目录
    LIBRARY DESTINATION lib                 # 安装动态库到 lib 目录
    RUNTIME DESTINATION bin                 # 安装可执行文件到 bin 目录
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

# 导出项目的包含目录，供其他依赖此项目的包使用
ament_export_include_directories(
    "include/${PROJECT_NAME}"  # 导出该项目的头文件目录
)

# 导出库，供其他依赖此库的包使用
ament_export_libraries(
    ros2_utility  # 导出 ros2_utility 库，使其他包可以链接和使用该库
)

# 导出 CMake 目标，以便其他包能够找到和使用项目中的 CMake 目标
ament_export_targets(
    export_${PROJECT_NAME}  # 导出项目的 CMake 目标
)

ament_package()