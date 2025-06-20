
import os
from os import pathsep
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    cobot_description = get_package_share_directory("new_cobot_moveit")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(cobot_description, "urdf", "cobot_nextup_arm.urdf.xacro"),
                                      description="Absolute path to the robot urdf file")
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description' : robot_description}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch/gazebo.launch.py')
        ),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "cobot_nextup", "-topic", "robot_description"],
        output = "screen"
    )
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_robot
    ])
