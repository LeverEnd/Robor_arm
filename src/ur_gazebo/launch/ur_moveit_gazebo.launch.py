import os
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def spawn_entities(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('ur_robotiq_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'ur3e_robotiq_2f_85_urdf.xacro')
    tmp_urdf = '/tmp/ur3e_robotiq_2f_85.urdf'
    subprocess.run(['xacro', xacro_file], stdout=open(tmp_urdf, 'w'), check=True)
    spawn_node = Node(
        package='ros_gz_sim', executable='create', name='spawn_ur3e_with_gripper',
        arguments=[
            '-file', tmp_urdf,
            '-name', 'ur3e_with_gripper',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )
    return [spawn_node]

def generate_launch_description():
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    pkg_gazebo = get_package_share_directory('ur_gazebo')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    pkg_moveit_config = get_package_share_directory("moveit_config_ur")

    moveit_config = (
        MoveItConfigsBuilder("ur3e_robotiq_2f_85", package_name="moveit_config_ur")
        .robot_description(file_path="config/ur3e_robotiq_2f_85.urdf.xacro", mappings={
            "use_fake_hardware": "false",
            "use_sim_time": "true",
        })
        .robot_description_semantic(file_path="config/ur3e_robotiq_2f_85.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        pkg_moveit_config,
        "config",
        "ros2_controllers.yaml",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/world/empty/model/ur3e_with_gripper/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '--ros-args', '--log-level', 'info'
        ],
        output='screen'
    )

    spawn = OpaqueFunction(function=spawn_entities)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="both",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": True},
        #    {
        #        "ur3e_with_gripper": {
        #            "ros__parameters": {
        #                "use_sim_time": True,
        #                "ros2_control": {
        #                    "type": "system",
        #                "    hardware_plugin": "gz_ros2_control/GazeboSystem"
        #                }
        #            }
        #        }
        #    }
        ],
        arguments=["--ros-args", "--log-level", "debug"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config_path = os.path.join(
        pkg_moveit_config,
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    sleep_cmd = ExecuteProcess(
        cmd=["sleep", "2"],
        output="screen",
    )

    launch_cmds = [
        gz_sim,
        sleep_cmd,
        robot_state_publisher,
        spawn,
        ExecuteProcess(cmd=["sleep", "2"], output="screen"),
        gazebo_bridge,
        ros2_control_node,
        ExecuteProcess(cmd=["sleep", "5"], output="screen"),
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        ExecuteProcess(cmd=["sleep", "2"], output="screen"),
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(launch_cmds)
