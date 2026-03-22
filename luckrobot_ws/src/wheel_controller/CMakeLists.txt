cmake_minimum_required(VERSION 3.8)
project(wheel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 编译节点
add_executable(wheel_controller_node src/wheel_controller_node.cpp)
add_executable(lead_screw_motor_node src/lead_screw_motor_node.cpp)
add_executable(danger_command_node src/danger_command_node.cpp)
ament_target_dependencies(danger_command_node rclcpp std_msgs)
ament_target_dependencies(wheel_controller_node 
  rclcpp 
  geometry_msgs 
  std_msgs 
)
ament_target_dependencies(lead_screw_motor_node
  rclcpp
  std_msgs
)
target_include_directories(lead_screw_motor_node
  PUBLIC
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_directories(lead_screw_motor_node
  PUBLIC
  ${LIBSERIAL_LIBRARY_DIRS}
)

target_link_libraries(lead_screw_motor_node
  ${LIBSERIAL_LIBRARIES}
)

# 安装可执行文件
install(
  TARGETS wheel_controller_node lead_screw_motor_node danger_command_node
  DESTINATION lib/${PROJECT_NAME}
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
