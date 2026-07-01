# 导入必要的模块和类
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock")
    use_sim_time = LaunchConfiguration("use_sim_time")
    startup_enabled_joints = LaunchConfiguration("startup_enabled_joints")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Use the mock driver instead of the physical CAN driver.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        ),
        DeclareLaunchArgument(
            "startup_enabled_joints",
            default_value="pitch_joint",
            description="Comma-separated list of joints to enable on startup.",
        ),
    ]
    
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dzb_description"),
                    "urdf",
                    "dzb_description.urdf.xacro",
                ]
            ),
            " ",
            "use_mock:=",
            use_mock,
            " ",
            "startup_enabled_joints:=",
            startup_enabled_joints,
        ]
    )
    # 定义机器人描述参数
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    # 定义控制器配置文件路径
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("dzb_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    # 创建机器人状态发布器节点
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )
    # 创建ros2_control节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="screen",
    )
    # 创建关节状态广播器和机械臂控制器的spawner节点
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    # 创建机械臂控制器的spawner节点
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ]
    )
