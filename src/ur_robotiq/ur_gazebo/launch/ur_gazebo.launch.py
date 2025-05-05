import os
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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

    spawn = OpaqueFunction(function=spawn_entities)

    return LaunchDescription([
        gz_sim,
        spawn,
    ])
