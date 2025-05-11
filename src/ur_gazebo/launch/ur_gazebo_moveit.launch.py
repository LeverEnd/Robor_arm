import os
import subprocess # Import subprocess module
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# Import ParameterValue as suggested by the error
from launch_ros.parameter_descriptions import ParameterValue 

# Import get_package_share_directory as it's used within OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import yaml # To load YAML files

def load_yaml(package_name, file_path):
    """Helper function to load YAML file content."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        print(f"Warning: Unable to load YAML file: {absolute_file_path}")
        return None

# OpaqueFunction to handle delayed evaluation and node creation
def launch_setup(context, *args, **kwargs):
    
    # Evaluate LaunchConfigurations needed within this function
    description_package_str = LaunchConfiguration("description_package").perform(context)
    description_file_str = LaunchConfiguration("description_file").perform(context)
    moveit_config_package_str = LaunchConfiguration("moveit_config_package").perform(context)
    controllers_config_str = LaunchConfiguration("controllers_config").perform(context)
    robot_name_arg_str = LaunchConfiguration("robot_name_arg").perform(context) 
    launch_rviz_str = LaunchConfiguration("launch_rviz").perform(context) 
    world_str = LaunchConfiguration("world").perform(context)
    gazebo_paused_str = LaunchConfiguration("gazebo_paused").perform(context)

    # --- Gazebo ---
    pkg_ros_gz_sim_path = get_package_share_directory('ros_gz_sim')
    pkg_ur_gazebo_path = get_package_share_directory('ur_gazebo')

    world_file_path = os.path.join(pkg_ur_gazebo_path, 'worlds', world_str)
    gz_sim_launch_file_path = os.path.join(pkg_ros_gz_sim_path, 'launch', 'gz_sim.launch.py')

    gazebo_launch_args = {
        'gz_args': ['-v4 ', world_file_path], 
        'paused': gazebo_paused_str,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file_path), 
        launch_arguments=gazebo_launch_args.items(),
    )

    bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

    # --- Load MoveIt Configuration ---
    # Construct paths using evaluated strings first
    robot_description_path = os.path.join(
        get_package_share_directory(description_package_str), "urdf", description_file_str
    )
    controllers_path_sub = PathJoinSubstitution([ # Keep as Substitution for xacro mapping
        FindPackageShare(moveit_config_package_str), "config", controllers_config_str
    ])
    srdf_path_sub = PathJoinSubstitution([ # Keep as Substitution
        FindPackageShare(moveit_config_package_str), "config", "ur3e_robotiq_2f_85.srdf"
    ])
    rviz_config_file_sub = PathJoinSubstitution( # Keep as Substitution
        [FindPackageShare(moveit_config_package_str), "config", "moveit.rviz"]
    )

    # Generate robot_description parameter using xacro command (as Substitution)
    robot_description_content_sub = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        robot_description_path, " ", # Use evaluated path here
        "use_fake_hardware:=false", " ",
        "sim_ignition:=true", " ",
        "sim_gazebo:=false", " ",
        "simulation_controllers:=", controllers_path_sub, # Pass Substitution
    ])
    
    # Wrap the Command substitution in ParameterValue for nodes that need it
    robot_description_param = {"robot_description": ParameterValue(robot_description_content_sub, value_type=str)}
    robot_description_semantic_param = {"robot_description_semantic": ParameterValue(Command(['cat ', srdf_path_sub]), value_type=str)} # Load SRDF content as string

    # Load YAML files using evaluated package name string
    kinematics_yaml = load_yaml(moveit_config_package_str, "config/kinematics.yaml")
    ompl_planning_yaml = load_yaml(moveit_config_package_str, "config/ompl_planning.yaml")
    moveit_controllers_yaml = load_yaml(moveit_config_package_str, "config/moveit_controllers.yaml")
    
    if kinematics_yaml is None: raise RuntimeError(f"Could not load kinematics config file")
    if ompl_planning_yaml is None: raise RuntimeError(f"Could not load ompl config file")   
    if moveit_controllers_yaml is None: raise RuntimeError(f"Could not load moveit_controllers config file")

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml) 

    # Evaluate the robot description string specifically for spawn_entity
    evaluated_robot_description = robot_description_content_sub.perform(context)

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', evaluated_robot_description, 
            '-name', robot_name_arg_str, # Use evaluated name
            '-allow_renaming', 'true',
        ],
    )

    # === ROS 2 Nodes ===
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {'use_sim_time': True}], # Use ParameterValue wrapped version
    )

    # MoveGroup Node parameters
    move_group_params = [
        robot_description_param, # Use ParameterValue wrapped version
        robot_description_semantic_param, # Use ParameterValue wrapped version
        {"planning_pipelines": ["ompl"]}, 
        ompl_planning_pipeline_config, 
        kinematics_yaml, 
        moveit_controllers, 
        {'use_sim_time': True},
    ]
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz Node parameters
    rviz_params = [
        robot_description_param, # Use ParameterValue wrapped version
        robot_description_semantic_param, # Use ParameterValue wrapped version
        {"planning_pipelines": ["ompl"]}, 
        ompl_planning_pipeline_config, 
        kinematics_yaml, 
        {'use_sim_time': True},
    ]
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file_sub], # Pass Substitution for path
        parameters=rviz_params,
        condition=IfCondition(launch_rviz_str), 
    )
    
    # Spawner Nodes
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    # Delay spawners
    delay_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity, 
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    nodes_to_return = [
        gazebo, 
        bridge_node, 
        spawn_entity,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        delay_jsb_spawner, 
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
    ]
    return nodes_to_return


def generate_launch_description():

    declared_arguments = []
    # ... (Argument declarations remain the same) ...
    declared_arguments.append( DeclareLaunchArgument( "description_package", default_value="ur_robotiq_description", description="Package with robot description URDF/XACRO files.",))
    declared_arguments.append( DeclareLaunchArgument( "description_file", default_value="ur3e_robotiq_2f_85_urdf.xacro", description="URDF/XACRO description file with the robot.",))
    declared_arguments.append( DeclareLaunchArgument( "moveit_config_package", default_value="moveit_config_ur", description="Package with the MoveIt configuration files.",))
    declared_arguments.append( DeclareLaunchArgument( "controllers_config", default_value="ros2_controllers.yaml", description="YAML file with the controllers configuration relative to moveit_config_package.",))
    declared_arguments.append( DeclareLaunchArgument( "world", default_value="empty.world", description="Name of the world file in ur_gazebo/worlds.",))
    declared_arguments.append( DeclareLaunchArgument( "robot_name_arg", default_value="ur3e_with_gripper", description="Name of the robot model in Gazebo.",))
    declared_arguments.append( DeclareLaunchArgument( "launch_rviz", default_value="true", description="Launch RViz?"))
    declared_arguments.append( DeclareLaunchArgument( "gazebo_paused", default_value="true", description="Start Gazebo paused?"))
    
    opaque_function = OpaqueFunction(function=launch_setup)

    ld = LaunchDescription()

    for arg in declared_arguments:
        ld.add_action(arg)
    
    ld.add_action(opaque_function)

    return ld
