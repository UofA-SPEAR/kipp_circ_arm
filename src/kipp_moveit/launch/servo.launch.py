import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    print(f"Loading file: {absolute_file_path}")  # Debug absolute path
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        print(f"Failed to load file: {absolute_file_path}")
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    print(f"Loading YAML: {absolute_file_path}")  # Debug absolute path
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        print(f"Failed to load YAML: {absolute_file_path}")
        return None


def generate_launch_description():
    urdf_path = "/home/ayden/kipp_circ_arm/src/kipp_moveit/config/kipps_arm.urdf.xacro"
    srdf_path = "/home/ayden/kipp_circ_arm/src/kipp_moveit/config/kipps_arm.srdf"
    print(f"URDF Path: {urdf_path}")  # Explicitly print absolute paths
    print(f"SRDF Path: {srdf_path}")

    moveit_config = (
        MoveItConfigsBuilder("kipp_moveit", package_name="kipp_moveit")
        .robot_description(file_path=urdf_path)  # Use absolute path
        .robot_description_semantic(file_path=srdf_path)  # Use absolute path
        .to_moveit_configs()
    )

    param_file = os.path.join(get_package_share_directory("kipp_servo"), "config", "kipp_servo.yaml")
    print(f"Servo param file: {param_file}")  # Debug absolute path

    # Publish robot description to the parameter server
    robot_description_publisher = Node(
        package="rclcpp_components",
        executable="component_container",
        name="robot_description_container",
        parameters=[
            {"robot_description": moveit_config.robot_description["robot_description"]},
            {"robot_description_semantic": moveit_config.robot_description_semantic["robot_description_semantic"]},
        ],
        output="screen",
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="arm_servo",
        parameters=[
            param_file,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
        output="screen",
    )

    # Add RViz node to the launch
    rviz_config_path = os.path.join(get_package_share_directory("kipp_moveit"), "config", "moveit.rviz")
    print(f"RViz Config file: {rviz_config_path}")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    return LaunchDescription([
        robot_description_publisher,
        servo_node,
        rviz_node,
    ])
