cmake_minimum_required(VERSION 3.5)
project(kipp_moveit)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)
find_package(controller_manager REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)


# Export the package
ament_package()
