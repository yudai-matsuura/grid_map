cmake_minimum_required(VERSION 3.8)
project(traversability_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
# find_package(traversability_msgs REQUIRED)
find_package(lbr_terrain_analysis REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# traversability_publisher_ros2
add_executable(traversability_publisher_ros2 src/traversability_publisher_ros2.cpp)

target_include_directories(traversability_publisher_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  traversability_publisher_ros2
  rclcpp
  grid_map_ros
  grid_map_core
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  # traversability_msgs
  lbr_terrain_analysis)

  # traversability_publisher_for_MADMAX
add_executable(traversability_publisher_for_madmax src/traversability_publisher_for_madmax.cpp)

add_executable(fix_camera_frame src/fix_camera_frame.cpp)
ament_target_dependencies(fix_camera_frame
  rclcpp
  sensor_msgs
)

install(TARGETS
  fix_camera_frame
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(traversability_publisher_for_madmax PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  traversability_publisher_for_madmax
  rclcpp
  grid_map_ros
  grid_map_core
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  lbr_terrain_analysis
)

install(TARGETS
  traversability_publisher_ros2
  traversability_publisher_for_madmax
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
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
