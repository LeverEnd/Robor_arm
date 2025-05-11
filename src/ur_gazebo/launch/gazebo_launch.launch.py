import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess # IncludeLaunchDescription már nem használt itt
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue # Importáljuk a ParameterValue-t

def generate_launch_description():
    # --- Konfigurálható argumentumok ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_xacro_path_arg = LaunchConfiguration('robot_xacro_path', default=PathJoinSubstitution([
        FindPackageShare('ur_robotiq_description'), # VAGY a te csomagod neve, ahol a xacro van
        'urdf',
        'ur3e_robotiq_2f_85_urdf.xacro'
    ]))
    # Gazebo világ fájl (opcionális). Hagyd üresen az alapértelmezett üres világhoz.
    # gazebo_world_file = PathJoinSubstitution([
    # FindPackageShare('YOUR_PACKAGE_WITH_WORLD_FILE'),
    # 'worlds',
    # 'YOUR_WORLD_FILE.sdf'
    # ])
    # Az empty.sdf egy alapértelmezett üres világ, amit a gz sim képes betölteni
    gazebo_world_arg = LaunchConfiguration('gazebo_world', default='empty.sdf')


    # Robot neve Gazebo-ban
    robot_name = LaunchConfiguration('robot_name', default='ur3e_robotiq')
    # Robot kezdeti pozíciója Gazebo-ban
    spawn_x_val = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y_val = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z_val = LaunchConfiguration('spawn_z', default='0.0')
    spawn_roll_val = LaunchConfiguration('spawn_roll', default='0.0')
    spawn_pitch_val = LaunchConfiguration('spawn_pitch', default='0.0')
    spawn_yaw_val = LaunchConfiguration('spawn_yaw', default='0.0')

    # --- Vezérlők ---
    arm_controller_name = LaunchConfiguration('arm_controller_name', default='arm_trajectory_controller')
    gripper_controller_name = LaunchConfiguration('gripper_controller_name', default='gripper_controller')

    # --- Fájl elérési utak feldolgozása ---
    # A pkg_ros_gz_sim már nem szükséges itt közvetlenül, de a ros_gz_sim csomag telepítése igen.

    # Robot leírás (URDF) feldolgozása XACRO-ból
    # Javítva: a parancs és argumentumai külön elemek a listában
    robot_description_content = Command([
        'xacro', ' ', # A parancs
        robot_xacro_path_arg, ' ', # A XACRO fájl
        #'use_ros2_control:=true', ' ',
        'sim_gazebo:=true'
        # Itt további XACRO argumentumokat adhatsz meg, pl.:
        # ' initial_positions_file:=path/to/initial_positions.yaml'
    ])

    # --- Node-ok és Processzek ---

    # Gazebo Fortress szerver indítása
    # Javítva: cmd egy lista
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', gazebo_world_arg], # -r kapcsolóval indítjuk a szimulációt
        output='screen'
    )
    # Gazebo kliens (GUI) indítása (opcionális, ha csak headless kell)
    # Javítva: cmd egy lista
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim'],
        output='screen',
        # Indítsd a klienst csak a szerver után (opcionális, de jó gyakorlat)
        # condition=IfCondition(LaunchConfiguration('launch_gui', default='true'))
    )

    # Robot State Publisher: TF transzformációk publikálása
    # Javítva: ParameterValue használata a robot_description-höz
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # Robot spawnolása Gazebo-ba
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_content, # Itt a robot_description_content (Command substitution) közvetlenül használható
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-R', spawn_roll_val,
            '-P', spawn_pitch_val,
            '-Y', spawn_yaw_val
        ]
    )

    # --- ros2_control vezérlők betöltése és indítása ---
    # Joint State Broadcaster: Publikálja a joint állapotokat a /joint_states topic-ra
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robotkar vezérlő spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[arm_controller_name, '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gripper vezérlő spawner
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[gripper_controller_name, '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('robot_xacro_path',
                              default_value=PathJoinSubstitution([
                                  FindPackageShare('ur_robotiq_description'),
                                  'urdf',
                                  'ur3e_robotiq_2f_85_urdf.xacro'
                              ]),
                              description='Path to the robot XACRO file'),
        DeclareLaunchArgument('gazebo_world', default_value='empty.sdf', description='Gazebo world file name or path'),
        DeclareLaunchArgument('robot_name', default_value='ur3e_robotiq', description='Name of the robot to spawn'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='X coordinate for robot spawn'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Y coordinate for robot spawn'),
        DeclareLaunchArgument('spawn_z', default_value='0.0', description='Z coordinate for robot spawn'),
        DeclareLaunchArgument('spawn_roll', default_value='0.0', description='Roll angle for robot spawn'),
        DeclareLaunchArgument('spawn_pitch', default_value='0.0', description='Pitch angle for robot spawn'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Yaw angle for robot spawn'),
        DeclareLaunchArgument('arm_controller_name', default_value='ur_manipulator_controller', description='Name of the arm controller'),
        DeclareLaunchArgument('gripper_controller_name', default_value='robotiq_gripper_controller', description='Name of the gripper controller'),

        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
