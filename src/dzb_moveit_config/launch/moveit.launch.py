from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Pass through to dzb_description.urdf.xacro so MoveIt uses the same robot model variant.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        ),
    ]

    robot_description_path = (
        Path(get_package_share_directory("dzb_description"))
        / "urdf"
        / "dzb_description.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("dzb_description", package_name="dzb_moveit_config")
        .robot_description(
            file_path=str(robot_description_path),
            mappings={"use_mock": use_mock},
        )
        .robot_description_semantic(file_path="config/dzb_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines()
        .to_moveit_configs()
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": False,
        "publish_robot_description_semantic": True,
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            move_group,
            rviz,
        ]
    )
