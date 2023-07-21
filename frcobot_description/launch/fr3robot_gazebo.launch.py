from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get paths
    package_dir = get_package_share_directory('frcobot_description')
    xacro_path = os.path.join(package_dir, 'urdf', 'fr3_robot.xacro')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[xacro_path]
    )


    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=['-entity', 'fr3_robot', '-topic', 'robot_description'],
        output='screen')

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity
    ])

