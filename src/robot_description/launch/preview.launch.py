import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_description_path = (
        get_package_share_directory("robot_description") + "/sdf/my_diff_car.sdf"
    )
    rviz_config_path = (
        get_package_share_directory("robot_description") + "/config/rviz_config.rviz"
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": open(robot_description_path).read()}],
            ),
            launch_ros.actions.Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )
