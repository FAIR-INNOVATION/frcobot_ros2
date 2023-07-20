from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fr3_robot_v_5_0", package_name="fr3_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
