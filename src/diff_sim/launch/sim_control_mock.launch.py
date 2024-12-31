from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the package share directory
    robot_description = get_package_share_directory("robot_description")
    simulation_package = get_package_share_directory("diff_sim")

    # Define the path to RViz, and SDF files
    rviz_file = os.path.join(simulation_package, "config", "rviz_config.rviz")
    sdf_file = os.path.join(robot_description, "sdf", "my_diff_car_control_mock.sdf")

    # Path to controller file
    controller_config_path = os.path.join(
        simulation_package,
        "config",
        "diff_drive_controller_mock.yaml",
    )

    # Read the robot description file
    robot_description_content = open(sdf_file, "r").read()

    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        output="both",
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
            control_node,
            robot_state_publisher_node,
            robot_controller_spawner,
            delay_joint_state_broadcaster_after_robot_controller_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
        ]
    )
