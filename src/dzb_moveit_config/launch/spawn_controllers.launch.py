from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
from pathlib import Path


def generate_launch_description():
    robot_description_path = Path(get_package_share_directory("dzb_description")) / "urdf" / "dzb_description.urdf.xacro"
    moveit_config = (
        MoveItConfigsBuilder("dzb_description", package_name="dzb_moveit_config")
        .robot_description(file_path=str(robot_description_path))
        .to_moveit_configs()
    )
    return generate_spawn_controllers_launch(moveit_config)
