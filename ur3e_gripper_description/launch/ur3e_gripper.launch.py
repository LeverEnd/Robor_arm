import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Elérési utak
    pkg_share = FindPackageShare("ur3e_gripper_description").find("ur3e_gripper_description")

    xacro_file = os.path.join(pkg_share, "urdf", "ur3e_with_gripper.urdf.xacro")
    gazebo_controllers = os.path.join(pkg_share, "config", "ur3e_gripper_controllers.yaml")

    # RViz fájl (opcionális)
    rviz_config = os.path.join(pkg_share, "rviz", "ur3e_gripper.rviz")

    # URDF generálása a xacro-ból
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_file,
        " ",
        " ".join([
            'name:=ur3e_with_gripper',
            'tf_prefix:=""',
            'joint_limits_parameters_file:=$(find ur_description)/config/ur3e/joint_limits.yaml',
            'kinematics_parameters_file:=$(find ur_description)/config/ur3e/default_kinematics.yaml',
            'physical_parameters_file:=$(find ur_description)/config/ur3e/physical_parameters.yaml',
            'visual_parameters_file:=$(find ur_description)/config/ur3e/visual_parameters.yaml',
            'use_fake_hardware:=true',
            'sim_gazebo:=true'
        ])
    ])

    robot_description = {"robot_description": robot_description_content}

    # Node-ok
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, gazebo_controllers],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_controllers", "-c", "/controller_manager"],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        output="screen"
    )

    # Gazebo indítása
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros")]),
            "/launch/gazebo.launch.py"
        ])
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "ur3e_with_gripper"
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        gazebo_launch,
        spawn_robot
    ])
