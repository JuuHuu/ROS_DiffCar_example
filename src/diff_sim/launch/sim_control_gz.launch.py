from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the package share directory
    robot_description = get_package_share_directory("robot_description")
    simulation_package = get_package_share_directory("diff_sim")

    # Define the path to RViz, and SDF files
    rviz_file = os.path.join(simulation_package, "config", "rviz_config.rviz")
    gazebo_launch_file = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    sdf_file = os.path.join(robot_description, "sdf", "my_diff_car_control_gz.sdf")

    # Path to controller file
    controller_config_path = os.path.join(
        simulation_package,
        "config",
        "diff_drive_controller.yaml",
    )

    # Read the robot description file
    robot_description_content = open(sdf_file, "r").read()

    # replace the PARAMETERS_CONFIG_FILE_PLACEHOLDER with controller
    robot_description_content = (
        open(sdf_file, "r")
        .read()
        .replace("PARAMETERS_CONFIG_FILE_PLACEHOLDER", controller_config_path)
    )

    robot_description = {"robot_description": robot_description_content}

    # bridge config file
    bridge_params = os.path.join(
        simulation_package,
        "config",
        "bridge_config_control_gz.yaml",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            controller_config_path,
        ],
    )

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Define the RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
    )

    world = "empty.world"
    world_file = os.path.join(simulation_package, "worlds", world)

    # Define the Gazebo node server
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "gz_args": [" -r -s -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Gazebo GUI
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file)),
        launch_arguments={"gz_args": "-g -v4 "}.items(),
    )

    # Define the spawn_entity node to spawn the robot in Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "robot",
        ],
        output="screen",
    )

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            robot_state_publisher_node,
            robot_controller_spawner,
            gazebo_node,
            gzclient_cmd,
            spawn_entity_node,
            start_gazebo_ros_bridge_cmd,
            delay_joint_state_broadcaster_after_robot_controller_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
        ]
    )
