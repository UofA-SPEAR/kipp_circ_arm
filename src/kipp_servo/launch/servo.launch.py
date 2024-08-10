import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
  
  
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kipp_moveit", package_name="kipp_moveit")
        .robot_description(file_path="config/kipps_arm.urdf.xacro")
        .to_moveit_configs()
    )



    param_file = os.path.join(get_package_share_directory("kipp_servo"), "config", "kipp_servo.yaml")

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name='arm_servo',
        parameters=[
            param_file,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output="screen",
    )

    return LaunchDescription(
        [
            
            servo_node,
        ]
    )