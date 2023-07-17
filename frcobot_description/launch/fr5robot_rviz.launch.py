import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Get robot_description via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('frcobot_description'),
        'urdf', 'fr5_robot.xacro')
    robot_description_content = subprocess.check_output(['ros2', 'run', 'xacro', 'xacro', robot_description_path])
    
    rviz_config_file = os.path.join(get_package_share_directory('frcobot_description'), 'rviz', 'fr5robot.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content.decode("utf-8")}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
