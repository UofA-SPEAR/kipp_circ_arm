from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    
    moveit_config = MoveItConfigsBuilder("kipps_arm", package_name="kipp_arm_moveit").to_moveit_configs()
    # Add default move_group_capabilities if missing
    if "capabilities" not in moveit_config.move_group_capabilities:
        moveit_config.move_group_capabilities["capabilities"] = "move_group/MoveGroupCartesianPathService,move_group/MoveGroupExecuteService"
        
    return generate_move_group_launch(moveit_config)
