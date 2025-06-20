from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('new_cobot_moveit'),
                        'launch/gazebo.launch.py')
        ),
    )
    
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory('new_cobot_moveit'),
                        'launch/demo.launch.py')
        ),
    )

    move_cartesian_node = Node(
        package='new_cobot_moveit',
        executable='multi_waypoint_cartesian',
        name='move_cartesian_node',
        output='screen'
    )

    ee_velocity_node = Node(
        package='new_cobot_moveit',
        executable='ee_velocity',
        name='ee_velocity_node',
        output='screen'
    )



    return LaunchDescription([
        gazebo_launch,
        demo_launch,
        move_cartesian_node,
        ee_velocity_node
    ])
