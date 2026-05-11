from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    freedrive_controller = LaunchConfiguration("freedrive_controller_name")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout")

    return LaunchDescription([
        DeclareLaunchArgument("freedrive_controller_name", default_value="freedrive_mode_controller"),
        DeclareLaunchArgument("controller_manager_timeout", default_value="30"),

        launch_ros.actions.Node(
            package="controller_manager",
            executable="spawner",
            output="log",
            arguments=[
                freedrive_controller,
                "--inactive",
                "--controller-manager-timeout", controller_manager_timeout,
            ],
        ),
    ])
