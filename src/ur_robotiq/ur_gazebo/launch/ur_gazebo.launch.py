import os
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def spawn_entities(context, *args, **kwargs):
    pkg_desc = get_package_share_directory('ur_robotiq_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'ur3e_robotiq_2f_85_urdf.xacro')
    tmp_urdf = '/tmp/ur3e_robotiq_2f_85.urdf'

    # Generate URDF from Xacro with controller parameters
    subprocess.run([
        'xacro', xacro_file,
        f'simulation_controllers:={os.path.join(pkg_desc, "config", "ur3e_robotiq_controllers.yaml")}',
        'sim_ignition:=true'
    ], stdout=open(tmp_urdf, 'w'), check=True)

    # Spawn the robot in Ignition Gazebo
    return [
        Node(
            package='ros_gz_sim', executable='create', name='spawn_ur3e_with_gripper',
            arguments=[
                '-file', tmp_urdf,
                '-name', 'ur3e_with_gripper',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ]


def load_controllers():
    return [
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'),
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'], output='screen'),
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'], output='screen')
    ]


def generate_launch_description():
    # Paths to packages and world file
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    pkg_gazebo = get_package_share_directory('ur_gazebo')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.world')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')

    # Launch Ignition Fortress (server + GUI)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-g -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Bridge between ROS2 and Ignition topics/services
    ros_gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['-c', bridge_config],
        output='screen'
    )

    # Spawn robot and delay controller loading
    spawn = OpaqueFunction(function=spawn_entities)
    delayed = TimerAction(period=5.0, actions=load_controllers())

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        spawn,
        delayed,
    ])
