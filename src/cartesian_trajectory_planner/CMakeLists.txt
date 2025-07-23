cmake_minimum_required(VERSION 3.10)
project(cartesian_trajectory_planner)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Main trajectory planner node
add_executable(cartesian_traj_node src/cartesian_traj_node.cpp)
ament_target_dependencies(cartesian_traj_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
  trajectory_msgs
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
)

# Velocity estimator node
add_executable(velocity_estimator_node src/velocity_estimator_node.cpp)
ament_target_dependencies(velocity_estimator_node
  rclcpp
  sensor_msgs
)

# Install executables
install(TARGETS
  cartesian_traj_node
  velocity_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

