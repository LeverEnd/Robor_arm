from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur3e_robotiq_2f_85", package_name="moveit_config_ur").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
