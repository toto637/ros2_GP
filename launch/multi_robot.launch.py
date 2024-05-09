import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description(): 

    # Declare launch arguments for both robot models
    model_arg1 = DeclareLaunchArgument(
        name="model1",
        default_value=os.path.join(get_package_share_directory("my_robot_description"), "urdf", "robot_description.urdf.xacro"),
        description="Absolute path to first robot URDF file"
    )

    model_arg2 = DeclareLaunchArgument(
        name="model2",
        default_value=os.path.join(get_package_share_directory("my_robot_description"), "urdf", "robot_description2.urdf.xacro"),
        description="Absolute path to second robot URDF file"
    )

    # Define parameters for both robot descriptions
    robot_description1 = ParameterValue(Command(["xacro ", LaunchConfiguration("model1")]), value_type=str)
    robot_description2 = ParameterValue(Command(["xacro ", LaunchConfiguration("model2")]), value_type=str)

    # Node to publish state for the first robot
    robot1_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description1}]
    )

    # Node to publish state for the second robot
    robot2_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description2}]
    )

    # Set GAZEBO_MODEL_PATH environment variable
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("my_robot_description"), "share"))

    # Include Gazebo server launch file
    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
    ))

    # Include Gazebo client launch file
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
    ))

    # Spawn the first robot in Gazebo
    spawn_robot1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bumperbot1", "-topic", "robot_description1"],
        output="screen"
    )

    # Spawn the second robot in Gazebo
    spawn_robot2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bumperbot2", "-topic", "robot_description2"],
        output="screen"
    )

    # Launch RViz to visualize both robots
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        model_arg1,
        model_arg2,
        robot1_state_publisher,
        robot2_state_publisher,
        env_var,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot1,
        spawn_robot2,
        rviz2_node
    ])