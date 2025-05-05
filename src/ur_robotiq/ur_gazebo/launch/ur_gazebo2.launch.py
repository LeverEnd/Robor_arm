import os
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def spawn_entities(context, *args, **kwargs):
    # Xacro -> URDF konverzió
    pkg_desc = get_package_share_directory('ur_robotiq_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'ur3e_robotiq_2f_85_urdf.xacro')
    tmp_urdf = '/tmp/ur3e_robotiq_2f_85.urdf'
    subprocess.run(['xacro', xacro_file], stdout=open(tmp_urdf, 'w'), check=True)
    
    # Robot spawn a Fortress-ben (Ignition Gazebo)
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ur3e_with_gripper',
        arguments=[
            '-file', tmp_urdf,
            '-name', 'ur3e_with_gripper',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'  # Kissé magasabbra helyezem, hogy ne legyen ütközés az alappal
        ],
        output='screen'
    )
    
    # Bridge csomópont a Gazebo és ROS2 közötti kommunikációhoz
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        output='screen'
    )
    
    return [spawn_node, bridge]

def generate_launch_description():
    # World és Gazebo indítása
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')
    pkg_gazebo = get_package_share_directory('ur_gazebo')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'empty.world')
    
    # Fortress szerver indítása az empty.world-del
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Controller manager és controllerek indítása
    pkg_config = get_package_share_directory('ur_moveit_config')
    controller_config = os.path.join(pkg_config, 'config', 'ur3e_robotiq_controllers.yaml')
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Joint trajectory controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Gripper controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Spawn entities után indítjuk a controllereket
    spawn = OpaqueFunction(function=spawn_entities)
    
    return LaunchDescription([
        gz_sim,
        spawn,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner
    ])
