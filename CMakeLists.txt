cmake_minimum_required(VERSION 3.8)
project(probabilistic_robot_lab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------------------------------------------------------
# Filter nodes
# -----------------------------------------------------------------------

add_executable(kf_node src/kf_node.cpp)
ament_target_dependencies(kf_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(kf_node Eigen3::Eigen)

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(ekf_node Eigen3::Eigen)

# Uncomment once pf_node.cpp is committed:
# add_executable(pf_node src/pf_node.cpp)
# ament_target_dependencies(pf_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
# target_link_libraries(pf_node Eigen3::Eigen)

# -----------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------

install(TARGETS kf_node ekf_node
  DESTINATION lib/${PROJECT_NAME})

# Uncomment together with pf_node above:
# install(TARGETS pf_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()